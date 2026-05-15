#include "unitree_guide_controller/control/ConvexMpcSolver.h"

#include <unitree_guide_controller/common/mathTools.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>

extern "C" {
#include <hpipm_common.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_d_ocp_qp_ipm.h>
}

// 叉乘矩阵工具
static inline Eigen::Matrix3d skew3(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
      -v.y(), v.x(), 0.0;
  return S;
}

// 3x3矩阵求逆工具：避免任何第三方宏污染 Eigen::Matrix::inverse()
static inline Eigen::Matrix3d inverse3x3Safe(const Eigen::Matrix3d& M) {
  const double a00 = M(0,0), a01 = M(0,1), a02 = M(0,2);
  const double a10 = M(1,0), a11 = M(1,1), a12 = M(1,2);
  const double a20 = M(2,0), a21 = M(2,1), a22 = M(2,2);

  const double c00 =  (a11 * a22 - a12 * a21);
  const double c01 = -(a10 * a22 - a12 * a20);
  const double c02 =  (a10 * a21 - a11 * a20);

  const double c10 = -(a01 * a22 - a02 * a21);
  const double c11 =  (a00 * a22 - a02 * a20);
  const double c12 = -(a00 * a21 - a01 * a20);

  const double c20 =  (a01 * a12 - a02 * a11);
  const double c21 = -(a00 * a12 - a02 * a10);
  const double c22 =  (a00 * a11 - a01 * a10);

  const double det = a00 * c00 + a01 * c01 + a02 * c02;

  if (std::abs(det) < 1e-12 || !std::isfinite(det)) {
    return Eigen::Matrix3d::Identity();
  }

  Eigen::Matrix3d adj;
  adj << c00, c10, c20,
         c01, c11, c21,
         c02, c12, c22;

  return adj / det;
}

class MemoryBlock {
public:
  explicit MemoryBlock(size_t size = 0) {
    reserve(size);
  }

  ~MemoryBlock() noexcept {
    std::free(ptr_);
  }

  void reserve(size_t size) {
    if (size > size_) {
      std::free(ptr_);
      ptr_ = std::malloc(size);
      if (ptr_ == nullptr) {
        throw std::bad_alloc();
      }
      size_ = size;
    }
  }

  void* get() {
    return ptr_;
  }

  MemoryBlock(const MemoryBlock&) = delete;
  MemoryBlock& operator=(const MemoryBlock&) = delete;

private:
  void* ptr_ = nullptr;
  size_t size_ = 0;
};

struct ConvexMpcSolver::HpipmWorkspace {
  int cached_N = -1;
  int cached_nx = -1;
  int cached_nu = -1;
  int cached_ng = -1;
  bool initialized = false;

  std::vector<int> nx;
  std::vector<int> nu;
  std::vector<int> nbx;
  std::vector<int> nbu;
  std::vector<int> ng;
  std::vector<int> nsbx;
  std::vector<int> nsbu;
  std::vector<int> nsg;

  MemoryBlock dimMem;
  d_ocp_qp_dim dim;

  MemoryBlock qpMem;
  d_ocp_qp qp;

  MemoryBlock qpSolMem;
  d_ocp_qp_sol qpSol;

  MemoryBlock ipmArgMem;
  d_ocp_qp_ipm_arg arg;

  MemoryBlock ipmMem;
  d_ocp_qp_ipm_ws workspace;

  void resizeIfNeeded(const ConvexMpcSettings& settings, int N, int nx_aug, int nu_stage, int ng_stage) {
    if (initialized &&
        cached_N == N &&
        cached_nx == nx_aug &&
        cached_nu == nu_stage &&
        cached_ng == ng_stage) {
      applySettings(settings);
      return;
    }

    cached_N = N;
    cached_nx = nx_aug;
    cached_nu = nu_stage;
    cached_ng = ng_stage;

    nx.assign(N + 1, nx_aug);
    nu.assign(N + 1, nu_stage);
    nbx.assign(N + 1, 0);
    nbu.assign(N + 1, 0);
    ng.assign(N + 1, ng_stage);
    nsbx.assign(N + 1, 0);
    nsbu.assign(N + 1, 0);
    nsg.assign(N + 1, 0);

    // HPIPM OCP-QP：第0阶段状态x0由外部给定，不作为决策变量
    nx[0] = 0;
    nu[N] = 0;
    ng[N] = 0;

    const int dim_size = d_ocp_qp_dim_memsize(N);
    dimMem.reserve(dim_size);
    d_ocp_qp_dim_create(N, &dim, dimMem.get());

    d_ocp_qp_dim_set_all(
        nx.data(),
        nu.data(),
        nbx.data(),
        nbu.data(),
        ng.data(),
        nsbx.data(),
        nsbu.data(),
        nsg.data(),
        &dim
    );

    const int qp_size = d_ocp_qp_memsize(&dim);
    qpMem.reserve(qp_size);
    d_ocp_qp_create(&dim, &qp, qpMem.get());

    const int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
    qpSolMem.reserve(qp_sol_size);
    d_ocp_qp_sol_create(&dim, &qpSol, qpSolMem.get());

    const int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
    ipmArgMem.reserve(ipm_arg_size);
    d_ocp_qp_ipm_arg_create(&dim, &arg, ipmArgMem.get());

    applySettings(settings);

    const int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
    ipmMem.reserve(ipm_size);
    d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipmMem.get());

    initialized = true;
  }

  void applySettings(const ConvexMpcSettings& settings) {
      d_ocp_qp_ipm_arg_set_default(hpipm_mode::SPEED, &arg);

      int iter_max = settings.hpipm_iter_max;
      double alpha_min = settings.hpipm_alpha_min;
      double mu0 = settings.hpipm_mu0;
      double tol_stat = settings.hpipm_tol_stat;
      double tol_eq = settings.hpipm_tol_eq;
      double tol_ineq = settings.hpipm_tol_ineq;
      double tol_comp = settings.hpipm_tol_comp;
      double reg_prim = settings.hpipm_reg_prim;
      int warm_start = settings.hpipm_warm_start;
      int pred_corr = settings.hpipm_pred_corr;
      int ric_alg = settings.hpipm_ric_alg;

      d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
      d_ocp_qp_ipm_arg_set_alpha_min(&alpha_min, &arg);
      d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
      d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
      d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
      d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
      d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
      d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
      d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &arg);
      d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr, &arg);
      d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &arg);
  }
};

ConvexMpcSolver::ConvexMpcSolver(const ConvexMpcSettings& settings)
  : settings_(settings),
    hpipm_(new HpipmWorkspace()) {
  last_u0_.setZero(12);

  // 默认给一套合理权重（你可以后续从 task.info 读）
  // state: [p v rpy w]
  if (settings_.Q.isZero(0)) {
    ConvexMpcSettings tmp = settings_;
    tmp.Q.diagonal() << 50, 50, 200,    5, 5, 10,    200, 200, 20,    2, 2, 2;
    tmp.R.diagonal() << 1,1,1,  1,1,1,  1,1,1,  1,1,1;
    tmp.S.diagonal() << 0.1,0.1,0.1,  0.1,0.1,0.1,  0.1,0.1,0.1,  0.1,0.1,0.1;
    settings_ = tmp;
  }
}

ConvexMpcSolver::~ConvexMpcSolver() = default;

// 允许在运行时改 MPC 参数
void ConvexMpcSolver::setSettings(const ConvexMpcSettings& settings) {
  settings_ = settings;
}

void ConvexMpcSolver::reset() {
  last_u0_.setZero(12);
  has_last_solution_ = false;
}

// 推断 trot base phase：只靠 contact_now 的最小策略
// FR FL RR RL：若 FR+RL 为支撑（1 0 0 1）则 base=0；若 FL+RR 为支撑（0 1 1 0）则 base=0.5
double ConvexMpcSolver::inferBasePhaseFromContact(const VecInt4& contact_now) {
  const int fr = contact_now(0);
  const int fl = contact_now(1);
  const int rr = contact_now(2);
  const int rl = contact_now(3);

  if (fr == 1 && fl == 0 && rr == 0 && rl == 1) {
    return 0.0;
  }
  if (fr == 0 && fl == 1 && rr == 1 && rl == 0) {
    return 0.5;
  }
  // fallback：未知模式则默认0
  return 0.0;
}

Vec34 ConvexMpcSolver::makeFallbackForces(const VecInt4& contact_now) const {
  Vec34 force;
  force.setZero();

  int stance_count = 0;
  for (int leg = 0; leg < 4; ++leg) {
    if (contact_now(leg) == 1) {
      stance_count++;
    }
  }

  // 极端情况下如果没有支撑腿，就按四腿均分，避免直接输出全0
  const bool no_stance = (stance_count == 0);
  const int effective_stance_count = no_stance ? 4 : stance_count;

  const double fz_nominal = settings_.mass * std::abs(settings_.g(2)) / static_cast<double>(effective_stance_count);
  const double fz = std::min(settings_.fzMax, std::max(settings_.fzMin, fz_nominal));

  for (int leg = 0; leg < 4; ++leg) {
    if (no_stance || contact_now(leg) == 1) {
      force(2, leg) = fz;
    }
  }

  return force;
}

// 改名：solveMpc（避免 solve 被宏替换成 lu_solve）
// 这里实现 MIT Cheetah3 Convex MPC：SRBD + friction pyramid + contact schedule + HPIPM OCP-QP
ConvexMpcOutputOcs2 ConvexMpcSolver::solveMpc(const ConvexMpcInputOcs2& in) {
  ConvexMpcOutputOcs2 out;
  out.u0.resize(12);
  out.u0.setZero();
  out.success = false;
  out.hpipm_status = -1;
  out.hpipm_iter = -1;

  const int N = settings_.N;
  const double dt = settings_.dt;

  constexpr int nx_phys = 12;     // [p v rpy w]
  constexpr int nu_stage = 12;    // [f_FR f_FL f_RR f_RL]
  constexpr int nx_aug = 24;      // [p v rpy w u_prev]
  constexpr int rows_per_leg = 7; // fx/fy/fz + 4 friction rows
  constexpr int ng_stage = 4 * rows_per_leg;

  // ====== 输入检查 ======
  if (in.x0.size() != nx_phys) {
    return out;
  }
  if ((int)in.xRef.size() < N + 1) {
    return out;
  }
  if ((int)in.contact.size() < N) {
    return out;
  }
  if ((int)in.rFeet.size() < N) {
    return out;
  }
  for (int k = 0; k <= N; ++k) {
    if (in.xRef[k].size() != nx_phys) {
      return out;
    }
  }

  // ====== 线性离散 SRBD 模型 ======
  // x = [p(3), v(3), rpy(3), w(3)]
  // u = [f_FR, f_FL, f_RR, f_RL]  (12)
  // p_{k+1} = p_k + dt*v_k
  // v_{k+1} = v_k + dt*(g + sum(f)/m)
  // rpy_{k+1} = rpy_k + dt*w_k
  // w_{k+1} = w_k + dt*Iw_inv*sum(r_i x f_i)
  Eigen::Matrix<double,nx_phys,nx_phys> A_phys = Eigen::Matrix<double,nx_phys,nx_phys>::Identity();
  A_phys.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
  A_phys.block<3,3>(6,9) = Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double,nx_phys,1> b_phys = Eigen::Matrix<double,nx_phys,1>::Zero();
  b_phys.segment<3>(3) = dt * settings_.g;

  // 每步 B_k（12x12）
  std::vector<Eigen::Matrix<double,nx_phys,nu_stage>> B_phys(N);
  const Eigen::Matrix3d Iw_inv = in.Iw_inv;  // 世界系惯量逆（由 solveFromDogWrench 计算）

  for (int k = 0; k < N; ++k) {
    B_phys[k].setZero();

    // v update: dt/m * f
    for (int leg = 0; leg < 4; ++leg) {
      B_phys[k].block<3,3>(3, 3*leg) = (dt / settings_.mass) * Eigen::Matrix3d::Identity();
    }

    // w update: dt * Iw_inv * (r x f)
    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector3d& r = in.rFeet[k][leg];
      B_phys[k].block<3,3>(9, 3*leg) = dt * Iw_inv * skew3(r);
    }
  }

  // ====== OCP-QP增强状态：x_aug = [x_phys; u_prev] ======
  // 用 u_prev 状态精确表达 force-rate cost: (u_k - u_{k-1})^T S (u_k - u_{k-1})
  Eigen::VectorXd x0_aug(nx_aug);
  x0_aug.setZero();
  x0_aug.segment(0, nx_phys) = in.x0;
  x0_aug.segment(nx_phys, nu_stage) = last_u0_;

  std::vector<Eigen::MatrixXd> A_aug(N);
  std::vector<Eigen::MatrixXd> B_aug(N);
  std::vector<Eigen::VectorXd> b_aug(N);

  for (int k = 0; k < N; ++k) {
    A_aug[k] = Eigen::MatrixXd::Zero(nx_aug, nx_aug);
    B_aug[k] = Eigen::MatrixXd::Zero(nx_aug, nu_stage);
    b_aug[k] = Eigen::VectorXd::Zero(nx_aug);

    A_aug[k].block(0, 0, nx_phys, nx_phys) = A_phys;

    B_aug[k].block(0, 0, nx_phys, nu_stage) = B_phys[k];
    B_aug[k].block(nx_phys, 0, nu_stage, nu_stage) = Eigen::Matrix<double,nu_stage,nu_stage>::Identity();

    b_aug[k].segment(0, nx_phys) = b_phys;
  }

  // HPIPM第0阶段不把x0作为决策变量，因此需要吸收初始状态：
  // x1 = A0*x0 + B0*u0 + b0 = B0*u0 + (A0*x0 + b0)
  Eigen::VectorXd b0_absorbed = A_aug[0] * x0_aug + b_aug[0];

  // ====== Stage cost ======
  // HPIPM cost convention:
  // 0.5*x^T Q*x + 0.5*u^T R*u + u^T S*x + q^T*x + r^T*u
  std::vector<Eigen::MatrixXd> Q_stage(N + 1);
  std::vector<Eigen::MatrixXd> R_stage(N + 1);
  std::vector<Eigen::MatrixXd> S_stage(N + 1);
  std::vector<Eigen::VectorXd> q_stage(N + 1);
  std::vector<Eigen::VectorXd> r_stage(N + 1);

  // k = 0: x0已经被消掉，只保留u0相关代价
  R_stage[0] = Eigen::MatrixXd::Zero(nu_stage, nu_stage);
  R_stage[0].noalias() = 2.0 * (settings_.R + settings_.S);

  r_stage[0] = Eigen::VectorXd::Zero(nu_stage);
  r_stage[0].noalias() = -2.0 * settings_.S * last_u0_;

  // k = 1 ... N-1
  for (int k = 1; k < N; ++k) {
    Q_stage[k] = Eigen::MatrixXd::Zero(nx_aug, nx_aug);
    R_stage[k] = Eigen::MatrixXd::Zero(nu_stage, nu_stage);
    S_stage[k] = Eigen::MatrixXd::Zero(nu_stage, nx_aug);
    q_stage[k] = Eigen::VectorXd::Zero(nx_aug);
    r_stage[k] = Eigen::VectorXd::Zero(nu_stage);

    // state tracking: (x_phys - xRef)^T Q (x_phys - xRef)
    Q_stage[k].block(0, 0, nx_phys, nx_phys).noalias() = 2.0 * settings_.Q;
    q_stage[k].segment(0, nx_phys).noalias() = -2.0 * settings_.Q * in.xRef[k];

    // force-rate state part: u_prev^T S u_prev
    Q_stage[k].block(nx_phys, nx_phys, nu_stage, nu_stage).noalias() = 2.0 * settings_.S;

    // input regularization + force-rate input part
    R_stage[k].noalias() = 2.0 * (settings_.R + settings_.S);

    // force-rate cross term: -2 u_k^T S u_prev
    S_stage[k].block(0, nx_phys, nu_stage, nu_stage).noalias() = -2.0 * settings_.S;
  }

  // k = N terminal cost，只跟踪状态，不再有输入
  Q_stage[N] = Eigen::MatrixXd::Zero(nx_aug, nx_aug);
  q_stage[N] = Eigen::VectorXd::Zero(nx_aug);
  Q_stage[N].block(0, 0, nx_phys, nx_phys).noalias() = 2.0 * settings_.Q;
  q_stage[N].segment(0, nx_phys).noalias() = -2.0 * settings_.Q * in.xRef[N];

  // Hessian regularization
  for (int k = 1; k <= N; ++k) {
    Q_stage[k].diagonal().array() += settings_.regularization;
  }
  for (int k = 0; k < N; ++k) {
    R_stage[k].diagonal().array() += settings_.regularization;
  }

  // ====== Constraints: lg <= C*x + D*u <= ug ======
  // 每条腿固定7行：
  // 0: fx
  // 1: fy
  // 2: fz
  // 3: -fx + mu*fz
  // 4:  fx + mu*fz
  // 5: -fy + mu*fz
  // 6:  fy + mu*fz
  const double INF = 1e19;

  std::vector<Eigen::MatrixXd> C_stage(N + 1);
  std::vector<Eigen::MatrixXd> D_stage(N + 1);
  std::vector<Eigen::VectorXd> lg_stage(N + 1);
  std::vector<Eigen::VectorXd> ug_stage(N + 1);

  for (int k = 0; k < N; ++k) {
    const int nx_k = (k == 0) ? 0 : nx_aug;

    C_stage[k] = Eigen::MatrixXd::Zero(ng_stage, nx_k);
    D_stage[k] = Eigen::MatrixXd::Zero(ng_stage, nu_stage);
    lg_stage[k] = Eigen::VectorXd::Constant(ng_stage, -INF);
    ug_stage[k] = Eigen::VectorXd::Constant(ng_stage,  INF);

    for (int leg = 0; leg < 4; ++leg) {
      const int row = rows_per_leg * leg;
      const int col = 3 * leg;

      // fx
      D_stage[k](row + 0, col + 0) = 1.0;
      // fy
      D_stage[k](row + 1, col + 1) = 1.0;
      // fz
      D_stage[k](row + 2, col + 2) = 1.0;

      // -fx + mu*fz >= 0
      D_stage[k](row + 3, col + 0) = -1.0;
      D_stage[k](row + 3, col + 2) =  settings_.mu;

      // fx + mu*fz >= 0
      D_stage[k](row + 4, col + 0) =  1.0;
      D_stage[k](row + 4, col + 2) =  settings_.mu;

      // -fy + mu*fz >= 0
      D_stage[k](row + 5, col + 1) = -1.0;
      D_stage[k](row + 5, col + 2) =  settings_.mu;

      // fy + mu*fz >= 0
      D_stage[k](row + 6, col + 1) =  1.0;
      D_stage[k](row + 6, col + 2) =  settings_.mu;

      if (in.contact[k][leg] == 0) {
        // swing: fx=fy=fz=0
        lg_stage[k](row + 0) = 0.0;
        ug_stage[k](row + 0) = 0.0;

        lg_stage[k](row + 1) = 0.0;
        ug_stage[k](row + 1) = 0.0;

        lg_stage[k](row + 2) = 0.0;
        ug_stage[k](row + 2) = 0.0;

        // friction rows are inactive for swing
        lg_stage[k](row + 3) = -INF;
        ug_stage[k](row + 3) =  INF;

        lg_stage[k](row + 4) = -INF;
        ug_stage[k](row + 4) =  INF;

        lg_stage[k](row + 5) = -INF;
        ug_stage[k](row + 5) =  INF;

        lg_stage[k](row + 6) = -INF;
        ug_stage[k](row + 6) =  INF;
      } else {
        // stance: fx/fy themselves unbounded, fz bounded
        lg_stage[k](row + 0) = -INF;
        ug_stage[k](row + 0) =  INF;

        lg_stage[k](row + 1) = -INF;
        ug_stage[k](row + 1) =  INF;

        lg_stage[k](row + 2) = settings_.fzMin;
        ug_stage[k](row + 2) = settings_.fzMax;

        // friction rows active
        lg_stage[k](row + 3) = 0.0;
        ug_stage[k](row + 3) = INF;

        lg_stage[k](row + 4) = 0.0;
        ug_stage[k](row + 4) = INF;

        lg_stage[k](row + 5) = 0.0;
        ug_stage[k](row + 5) = INF;

        lg_stage[k](row + 6) = 0.0;
        ug_stage[k](row + 6) = INF;
      }
    }
  }

  // terminal: no constraints
  C_stage[N] = Eigen::MatrixXd::Zero(0, nx_aug);
  D_stage[N] = Eigen::MatrixXd::Zero(0, 0);
  lg_stage[N] = Eigen::VectorXd::Zero(0);
  ug_stage[N] = Eigen::VectorXd::Zero(0);

  // ====== Resize HPIPM memory ======
  hpipm_->resizeIfNeeded(settings_, N, nx_aug, nu_stage, ng_stage);

  // ====== HPIPM pointer arrays ======
  std::vector<double*> AA(N, nullptr);
  std::vector<double*> BB(N, nullptr);
  std::vector<double*> bb(N, nullptr);

  std::vector<double*> QQ(N + 1, nullptr);
  std::vector<double*> RR(N + 1, nullptr);
  std::vector<double*> SS(N + 1, nullptr);
  std::vector<double*> qq(N + 1, nullptr);
  std::vector<double*> rr(N + 1, nullptr);

  std::vector<double*> CC(N + 1, nullptr);
  std::vector<double*> DD(N + 1, nullptr);
  std::vector<double*> llg(N + 1, nullptr);
  std::vector<double*> uug(N + 1, nullptr);

  // dynamics
  BB[0] = B_aug[0].data();
  bb[0] = b0_absorbed.data();

  for (int k = 1; k < N; ++k) {
    AA[k] = A_aug[k].data();
    BB[k] = B_aug[k].data();
    bb[k] = b_aug[k].data();
  }

  // costs
  RR[0] = R_stage[0].data();
  rr[0] = r_stage[0].data();

  for (int k = 1; k < N; ++k) {
    QQ[k] = Q_stage[k].data();
    RR[k] = R_stage[k].data();
    SS[k] = S_stage[k].data();
    qq[k] = q_stage[k].data();
    rr[k] = r_stage[k].data();
  }

  QQ[N] = Q_stage[N].data();
  qq[N] = q_stage[N].data();

  // constraints
  for (int k = 0; k < N; ++k) {
    CC[k] = (k == 0) ? nullptr : C_stage[k].data();
    DD[k] = D_stage[k].data();
    llg[k] = lg_stage[k].data();
    uug[k] = ug_stage[k].data();
  }

  // unused box/slack constraints
  int** hidxbx = nullptr;
  double** hlbx = nullptr;
  double** hubx = nullptr;

  int** hidxbu = nullptr;
  double** hlbu = nullptr;
  double** hubu = nullptr;

  double** hZl = nullptr;
  double** hZu = nullptr;
  double** hzl = nullptr;
  double** hzu = nullptr;

  int** hidxs = nullptr;
  double** hlls = nullptr;
  double** hlus = nullptr;

  // ====== Set and solve HPIPM OCP-QP ======
  d_ocp_qp_set_all(
      AA.data(), BB.data(), bb.data(),
      QQ.data(), SS.data(), RR.data(), qq.data(), rr.data(),
      hidxbx, hlbx, hubx,
      hidxbu, hlbu, hubu,
      CC.data(), DD.data(), llg.data(), uug.data(),
      hZl, hZu, hzl, hzu,
      hidxs, hlls, hlus,
      &hpipm_->qp
  );

  d_ocp_qp_ipm_solve(&hpipm_->qp, &hpipm_->qpSol, &hpipm_->arg, &hpipm_->workspace);

  int hpipm_status = -1;
  d_ocp_qp_ipm_get_status(&hpipm_->workspace, &hpipm_status);

  int hpipm_iter = -1;
  d_ocp_qp_ipm_get_iter(&hpipm_->workspace, &hpipm_iter);

  out.hpipm_status = hpipm_status;
  out.hpipm_iter = hpipm_iter;

  if (settings_.hpipm_verbose) {
    std::fprintf(stderr, "[ConvexMpcSolver][HPIPM] status=%d, iter=%d\n", hpipm_status, hpipm_iter);
  }

  // HPIPM SUCCESS 通常为0
  if (hpipm_status != 0) {
    if (settings_.enableFallbackToLast && has_last_solution_) {
      out.u0 = last_u0_;
      out.success = true;
      return out;
    }

    out.success = false;
    return out;
  }

  d_ocp_qp_sol_get_u(0, &hpipm_->qpSol, out.u0.data());

  out.success = out.u0.allFinite();

  // 更新 warm start
  if (out.success) {
    last_u0_ = out.u0;
    has_last_solution_ = true;
  }

  return out;
}

Vec34 ConvexMpcSolver::solveFromDogWrench(
    const Vec3& dd_pcd_G,         // 机身期望加速度（世界系）
    const Vec3& d_wbd_G,          // 机身期望角加速度（世界系）
    const RotMat& B2P_RotMat,
    const Vec34& rFeet_P,         // body→foot 的相对位矢
    const VecInt4& contact_now,   // 当前 4 足接触（0/1）
    const Vec3& p_body_G,         // 机身位置（世界系）
    const Vec3& v_body_G,         // 机身速度（世界系）
    const RotMat& R_GB,           // 机身姿态（旋转矩阵）
    const Vec3& gyro_G,           // 机身角速度（世界系）
    const RotMat& Rd_GB,          // 机身期望姿态
    const Vec3& v_ref_G) {        // 机身期望速度

  (void)B2P_RotMat;
  (void)d_wbd_G;

  // ====== 1) 构造 OCS2 输入 in ======
  ConvexMpcInputOcs2 in;
  in.t = 0.0;

  // x0 = [p v rpy w]
  in.x0.resize(12);
  in.x0.setZero();
  in.x0.segment<3>(0) << p_body_G(0), p_body_G(1), p_body_G(2);
  in.x0.segment<3>(3) << v_body_G(0), v_body_G(1), v_body_G(2);

  // 注意：rotMatToExp 不是严格 rpy，但你当前阶段够用；后续建议换成 estimator roll/pitch/yaw
  Vec3 rpy_now = rotMatToExp(R_GB);
  in.x0.segment<3>(6) << rpy_now(0), rpy_now(1), rpy_now(2);
  in.x0.segment<3>(9) << gyro_G(0), gyro_G(1), gyro_G(2);

  // xRef (N+1)
  in.xRef.resize(settings_.N + 1);
  Vec3 rpy_ref = rotMatToExp(Rd_GB);
  for (int k = 0; k <= settings_.N; ++k) {
    const double t_k = static_cast<double>(k) * settings_.dt;

    in.xRef[k].resize(12);
    in.xRef[k].setZero();

    // 位置参考：当前点 + 期望速度积分 + 期望加速度前馈项
    in.xRef[k].segment<3>(0) << 
        p_body_G(0) + v_ref_G(0) * t_k + 0.5 * dd_pcd_G(0) * t_k * t_k,
        p_body_G(1) + v_ref_G(1) * t_k + 0.5 * dd_pcd_G(1) * t_k * t_k,
        p_body_G(2) + v_ref_G(2) * t_k + 0.5 * dd_pcd_G(2) * t_k * t_k;

    in.xRef[k].segment<3>(3) <<
        v_ref_G(0),
        v_ref_G(1),
        v_ref_G(2);

    in.xRef[k].segment<3>(6) << rpy_ref(0), rpy_ref(1), rpy_ref(2);
    in.xRef[k].segment<3>(9) << 0.0, 0.0, 0.0;
  }

  // ====== contact schedule (N) ======
  // 0.5s horizon 内一定存在切换，所以这里必须预测 contact schedule
  in.contact.resize(settings_.N);

  if (settings_.enableContactPrediction) {
    // 最小可用：trot 预测器（对角交替）
    // 如果你后续要完全对齐 wave_generator 预测的 phase，把下面 basePhase 换成 wave_generator_->phase_(...) 即可
    const double basePhase = inferBasePhaseFromContact(contact_now);

    for (int k = 0; k < settings_.N; ++k) {
      const double t_k = static_cast<double>(k) * settings_.dt;
      const double phase = std::fmod(basePhase + t_k / settings_.gaitPeriod, 1.0);

      std::array<int,4> ck;
      for (int leg = 0; leg < 4; ++leg) {
        const double ph_leg = std::fmod(phase + settings_.phaseOffset(leg), 1.0);
        ck[leg] = (ph_leg < settings_.stanceRatio) ? 1 : 0;
      }
      in.contact[k] = ck;
    }
  } else {
    // 如果你未来从外部传入 contact schedule，可以关闭 enableContactPrediction
    for (int k = 0; k < settings_.N; ++k) {
      in.contact[k] = { contact_now(0), contact_now(1), contact_now(2), contact_now(3) };
    }
  }

  // ====== rFeet schedule (N) ======
  // body→foot 的相对位矢，转换为 COM→foot 的力臂：r = r_body->foot - R*pcb
  in.rFeet.resize(settings_.N);

  // inertia transform：Iw = R * Ib * R^T
  in.R_GB = Eigen::Matrix3d(R_GB);
  const Eigen::Matrix3d Iw = in.R_GB * settings_.Ib * in.R_GB.transpose();
  in.Iw_inv = inverse3x3Safe(Iw);

  const Eigen::Vector3d r_body_to_com_world = in.R_GB * settings_.pcb_B;

  // 第一版：力臂在 horizon 内近似常量（MIT 论文常见做法：在短时域内固定）
  for (int k = 0; k < settings_.N; ++k) {
    std::array<Eigen::Vector3d,4> rk;
    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector3d r_body_to_foot(
          rFeet_P(0,leg), rFeet_P(1,leg), rFeet_P(2,leg));

      // r_com_to_foot_world = r_body_to_foot_world - r_body_to_com_world
      rk[leg] = r_body_to_foot - r_body_to_com_world;
    }
    in.rFeet[k] = rk;
  }

  // ====== 2) 调用 MPC ======
  ConvexMpcOutputOcs2 out = solveMpc(in);

  // ====== 3) 输出回 Vec34 ======
  Vec34 force_feet_P;
  force_feet_P.setZero();

  if (!out.success || out.u0.size() != 12 || !out.u0.allFinite()) {
    return makeFallbackForces(contact_now);
  }

  for (int leg = 0; leg < 4; ++leg) {
    force_feet_P(0,leg) = out.u0(3*leg + 0);
    force_feet_P(1,leg) = out.u0(3*leg + 1);
    force_feet_P(2,leg) = out.u0(3*leg + 2);
  }

  // warm start / debug
  last_u0_ = out.u0;
  has_last_solution_ = true;

  return force_feet_P;
}