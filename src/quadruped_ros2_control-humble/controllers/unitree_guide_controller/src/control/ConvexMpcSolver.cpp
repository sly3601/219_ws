#include "unitree_guide_controller/control/ConvexMpcSolver.h"
#include "quadProgpp/QuadProg++.hh"

#include <unitree_guide_controller/common/mathTools.h>

#include <cmath>
#include <limits>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// 叉乘矩阵工具
static inline Eigen::Matrix3d skew3(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
      -v.y(), v.x(), 0.0;
  return S;
}

// 3x3矩阵求逆工具：避免 QuadProg++ 的 inverse 宏污染 Eigen::Matrix::inverse()
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

// QuadProg++ 求解：
// min 1/2 x^T G x + x^T g0
// s.t. CE^T x + ce0 = 0
//      CI^T x + ci0 >= 0
static bool solveDenseQpQuadProgpp(const Eigen::MatrixXd& H,
                                  const Eigen::VectorXd& g,
                                  const Eigen::MatrixXd& C,
                                  const Eigen::VectorXd& c_lower,
                                  const Eigen::VectorXd& c_upper,
                                  Eigen::VectorXd& U_opt) {
  const long n = (long)H.rows();
  const long nC = (long)C.rows();

  constexpr double kInf = 1e19;
  constexpr double kEqTol = 1e-10;

  std::vector<Eigen::RowVectorXd> Aeq_rows;
  std::vector<double> beq_values;
  std::vector<Eigen::RowVectorXd> Aineq_rows;
  std::vector<double> bineq_values;

  Aeq_rows.reserve(nC);
  beq_values.reserve(nC);
  Aineq_rows.reserve(2 * nC);
  bineq_values.reserve(2 * nC);

  for (long row = 0; row < nC; ++row) {
    const bool has_lower = std::isfinite(c_lower(row)) && c_lower(row) > -kInf;
    const bool has_upper = std::isfinite(c_upper(row)) && c_upper(row) <  kInf;

    if (has_lower && has_upper && std::abs(c_upper(row) - c_lower(row)) < kEqTol) {
      // C_i U = c_i
      Aeq_rows.push_back(C.row(row));
      beq_values.push_back(c_lower(row));
      continue;
    }

    if (has_upper) {
      // C_i U <= c_upper_i
      Aineq_rows.push_back(C.row(row));
      bineq_values.push_back(c_upper(row));
    }

    if (has_lower) {
      // C_i U >= c_lower_i  ->  -C_i U <= -c_lower_i
      Aineq_rows.push_back(-C.row(row));
      bineq_values.push_back(-c_lower(row));
    }
  }

  const long m = (long)Aeq_rows.size();
  const long p = (long)Aineq_rows.size();

  quadprogpp::Matrix<double> QG, QCE, QCI;
  quadprogpp::Vector<double> Qg0, Qce0, Qci0, Qx;

  QG.resize(n, n);
  QCE.resize(n, m);
  QCI.resize(n, p);
  Qg0.resize(n);
  Qce0.resize(m);
  Qci0.resize(p);
  Qx.resize(n);

  // G
  for (long i = 0; i < n; ++i) {
    for (long j = 0; j < n; ++j) {
      QG[i][j] = H(i, j);
    }
  }

  // g0
  for (long i = 0; i < n; ++i) {
    Qg0[i] = g(i);
  }

  // Aeq U = beq  ->  CE^T x + ce0 = 0
  // CE = Aeq^T, ce0 = -beq
  for (long i = 0; i < n; ++i) {
    for (long j = 0; j < m; ++j) {
      QCE[i][j] = Aeq_rows[j](i);
    }
  }
  for (long j = 0; j < m; ++j) {
    Qce0[j] = -beq_values[j];
  }

  // Aineq U <= bineq  ->  (-Aineq) U + bineq >= 0
  // CI = (-Aineq)^T, ci0 = bineq
  for (long i = 0; i < n; ++i) {
    for (long j = 0; j < p; ++j) {
      QCI[i][j] = -Aineq_rows[j](i);
    }
  }
  for (long j = 0; j < p; ++j) {
    Qci0[j] = bineq_values[j];
  }

  const double res = solve_quadprog(QG, Qg0, QCE, Qce0, QCI, Qci0, Qx);
  if (!std::isfinite(res)) {
    return false;
  }

  U_opt.resize(n);
  for (long i = 0; i < n; ++i) {
    U_opt(i) = Qx[i];
  }

  return U_opt.allFinite();
}

ConvexMpcSolver::ConvexMpcSolver(const ConvexMpcSettings& settings)
  : settings_(settings) {
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

// 允许在运行时改 MPC 参数
void ConvexMpcSolver::setSettings(const ConvexMpcSettings& settings) {
  settings_ = settings;
}

void ConvexMpcSolver::reset() {
  last_u0_.setZero(12);
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

// 改名：solveMpc（避免 solve 被宏替换成 lu_solve）
// 这里实现 MIT Cheetah3 Convex MPC：SRBD + friction pyramid + contact schedule + QP
ConvexMpcOutputOcs2 ConvexMpcSolver::solveMpc(const ConvexMpcInputOcs2& in) {
  ConvexMpcOutputOcs2 out;
  out.u0.resize(12);
  out.u0.setZero();
  out.success = false;

  const int N = settings_.N;
  const double dt = settings_.dt;

  // ====== 输入检查 ======
  if (in.x0.size() != 12) {
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

  // ====== 线性离散 SRBD 模型 ======
  // x = [p(3), v(3), rpy(3), w(3)]
  // u = [f_FR, f_FL, f_RR, f_RL]  (12)
  // p_{k+1} = p_k + dt*v_k
  // v_{k+1} = v_k + dt*(g + sum(f)/m)
  // rpy_{k+1} = rpy_k + dt*w_k
  // w_{k+1} = w_k + dt*Iw_inv*sum(r_i x f_i)
  Eigen::Matrix<double,12,12> A_k = Eigen::Matrix<double,12,12>::Identity();
  A_k.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
  A_k.block<3,3>(6,9) = Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double,12,1> d_k_const = Eigen::Matrix<double,12,1>::Zero();
  d_k_const.segment<3>(3) = dt * settings_.g;

  // 每步 B_k（12x12）
  std::vector<Eigen::Matrix<double,12,12>> B_k(N, Eigen::Matrix<double,12,12>::Zero());
  const Eigen::Matrix3d Iw_inv = in.Iw_inv;  // 世界系惯量逆（由 solveFromDogWrench 计算）

  for (int k = 0; k < N; ++k) {
    // v update: dt/m * f
    for (int leg = 0; leg < 4; ++leg) {
      B_k[k].block<3,3>(3, 3*leg) = (dt / settings_.mass) * Eigen::Matrix3d::Identity();
    }

    // w update: dt * Iw_inv * (r x f)
    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector3d& r = in.rFeet[k][leg];
      B_k[k].block<3,3>(9, 3*leg) = dt * Iw_inv * skew3(r);
    }
  }

  // ====== 预测矩阵：X = A_qp x0 + B_qp U + d_qp ======
  // U = [u0..u_{N-1}] ∈ R^{12N}
  const int nX = 12;
  const int nU = 12;
  const int nU_stack = nU * N;
  const int nX_stack = nX * N;

  Eigen::MatrixXd A_qp = Eigen::MatrixXd::Zero(nX_stack, nX);
  Eigen::MatrixXd B_qp = Eigen::MatrixXd::Zero(nX_stack, nU_stack);
  Eigen::VectorXd d_qp = Eigen::VectorXd::Zero(nX_stack);
  Eigen::VectorXd D = Eigen::VectorXd::Zero(nX_stack);

  std::vector<Eigen::Matrix<double,12,12>> Phi(N + 1);
  std::vector<Eigen::MatrixXd> B_qp_row(N + 1);
  std::vector<Eigen::Matrix<double,12,1>> d_pred(N + 1);

  Phi[0].setIdentity();
  B_qp_row[0] = Eigen::MatrixXd::Zero(nX, nU_stack);
  d_pred[0].setZero();

  for (int k = 0; k < N; ++k) {
    Phi[k + 1] = A_k * Phi[k];

    B_qp_row[k + 1] = A_k * B_qp_row[k];
    B_qp_row[k + 1].block(0, k * nU, nX, nU) += B_k[k];

    d_pred[k + 1] = A_k * d_pred[k] + d_k_const;

    A_qp.block(k * nX, 0, nX, nX) = Phi[k + 1];
    B_qp.block(k * nX, 0, nX, nU_stack) = B_qp_row[k + 1];
    d_qp.segment(k * nX, nX) = d_pred[k + 1];

    D.segment(k * nX, nX) = in.xRef[k + 1];
  }

  // ====== 构造 QP：min 0.5 U^T H U + U^T g ======
  Eigen::MatrixXd Q_qp = Eigen::MatrixXd::Zero(nX_stack, nX_stack);
  Eigen::MatrixXd R_qp = Eigen::MatrixXd::Zero(nU_stack, nU_stack);

  for (int k = 0; k < N; ++k) {
    Q_qp.block(k * nX, k * nX, nX, nX) = settings_.Q;
    R_qp.block(k * nU, k * nU, nU, nU) = settings_.R;
  }

  const Eigen::VectorXd E = A_qp * in.x0 + d_qp - D;

  Eigen::MatrixXd H = 2.0 * (B_qp.transpose() * Q_qp * B_qp + R_qp);
  Eigen::VectorXd g = 2.0 * B_qp.transpose() * Q_qp * E;

  // (3) 力变化率：sum (u_k-u_{k-1})^T S (u_k-u_{k-1})
  // k>=1 的差分
  for (int k = 1; k < N; ++k) {
    // (u_k-u_{k-1})^T S (u_k-u_{k-1})
    // expand -> add blocks
    H.block(k*nU, k*nU, nU, nU).noalias()           += 2.0 * settings_.S;
    H.block((k-1)*nU, (k-1)*nU, nU, nU).noalias()   += 2.0 * settings_.S;
    H.block(k*nU, (k-1)*nU, nU, nU).noalias()       += -2.0 * settings_.S;
    H.block((k-1)*nU, k*nU, nU, nU).noalias()       += -2.0 * settings_.S;
  }

  // k=0 与 last_u0_ 的差分
  // (u0-last)^T S (u0-last) => add 2S to H00, add -2S*last to g0
  H.block(0, 0, nU, nU).noalias() += 2.0 * settings_.S;
  g.segment(0, nU).noalias()      += -2.0 * settings_.S * last_u0_;

  // Hessian regularization
  H.diagonal().array() += settings_.regularization;

  // ====== 约束：contact schedule 必须预测 ======
  // swing leg: f=0 (eq)
  // stance leg: friction pyramid + fz bounds (ineq)
  int nC = 0;
  for (int k = 0; k < N; ++k) {
    for (int leg = 0; leg < 4; ++leg) {
      if (in.contact[k][leg] == 0) {
        nC += 3;
      } else {
        nC += 5;
      }
    }
  }

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(nC, nU_stack);
  Eigen::VectorXd c_lower = Eigen::VectorXd::Constant(nC, -1e20);
  Eigen::VectorXd c_upper = Eigen::VectorXd::Constant(nC,  1e20);

  int cRow = 0;

  for (int k = 0; k < N; ++k) {
    for (int leg = 0; leg < 4; ++leg) {
      const int col = k*nU + 3*leg;

      if (in.contact[k][leg] == 0) {
        // swing: fx=fy=fz=0
        C(cRow + 0, col + 0) = 1.0;
        C(cRow + 1, col + 1) = 1.0;
        C(cRow + 2, col + 2) = 1.0;

        c_lower(cRow + 0) = 0.0;
        c_lower(cRow + 1) = 0.0;
        c_lower(cRow + 2) = 0.0;

        c_upper(cRow + 0) = 0.0;
        c_upper(cRow + 1) = 0.0;
        c_upper(cRow + 2) = 0.0;

        cRow += 3;
      } else {
        // stance friction pyramid
        // -fx + mu fz >= 0
        C(cRow, col+0) = -1.0;
        C(cRow, col+2) =  settings_.mu;
        c_lower(cRow) = 0.0;
        c_upper(cRow) = 1e20;
        cRow++;

        // -fy + mu fz >= 0
        C(cRow, col+1) = -1.0;
        C(cRow, col+2) =  settings_.mu;
        c_lower(cRow) = 0.0;
        c_upper(cRow) = 1e20;
        cRow++;

        // fx + mu fz >= 0
        C(cRow, col+0) =  1.0;
        C(cRow, col+2) =  settings_.mu;
        c_lower(cRow) = 0.0;
        c_upper(cRow) = 1e20;
        cRow++;

        // fy + mu fz >= 0
        C(cRow, col+1) =  1.0;
        C(cRow, col+2) =  settings_.mu;
        c_lower(cRow) = 0.0;
        c_upper(cRow) = 1e20;
        cRow++;

        // fzMin <= fz <= fzMax
        C(cRow, col+2) = 1.0;
        c_lower(cRow) = settings_.fzMin;
        c_upper(cRow) = settings_.fzMax;
        cRow++;
      }
    }
  }

  // ====== 解 QP ======
  Eigen::VectorXd U_opt;
  U_opt.setZero(nU_stack);

  const bool ok = solveDenseQpQuadProgpp(H, g, C, c_lower, c_upper, U_opt);
  if (!ok) {
    // QP失败：可选回退上一帧
    if (settings_.enableFallbackToLast) {
      out.u0 = last_u0_;
      out.success = true;
      return out;
    }
    out.success = false;
    return out;
  }

  // 取第0步 u0
  out.u0 = U_opt.segment(0, nU);
  out.success = out.u0.allFinite();

  // 更新 warm start
  if (out.success) {
    last_u0_ = out.u0;
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

  (void)dd_pcd_G;
  (void)d_wbd_G;
  (void)B2P_RotMat;

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
    in.xRef[k].resize(12);
    in.xRef[k].setZero();
    in.xRef[k].segment<3>(0) << p_body_G(0), p_body_G(1), p_body_G(2);
    in.xRef[k].segment<3>(3) << v_ref_G(0), v_ref_G(1), v_ref_G(2);
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
      const double t_k = k * settings_.dt;
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
    return force_feet_P;
  }

  for (int leg = 0; leg < 4; ++leg) {
    force_feet_P(0,leg) = out.u0(3*leg + 0);
    force_feet_P(1,leg) = out.u0(3*leg + 1);
    force_feet_P(2,leg) = out.u0(3*leg + 2);
  }

  // warm start / debug
  last_u0_ = out.u0;
  return force_feet_P;
}