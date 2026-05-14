#include "unitree_guide_controller/control/ConvexMpcSolver.h"
#include "quadProgpp/QuadProg++.hh"

#include <unitree_guide_controller/common/mathTools.h>

#include <cmath>
#include <limits>

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

// QuadProg++ 求解：
// min 1/2 x^T G x + g0^T x
// s.t. CE^T x + ce0 = 0
//      CI^T x + ci0 >= 0
static bool solveDenseQpQuadProgpp(const Eigen::MatrixXd& G,
                                  const Eigen::VectorXd& g0,
                                  const Eigen::MatrixXd& Aeq,
                                  const Eigen::VectorXd& beq,
                                  const Eigen::MatrixXd& Aineq,
                                  const Eigen::VectorXd& bineq,
                                  Eigen::VectorXd& xOpt) {
  const long n = (long)G.rows();
  const long m = (long)Aeq.rows();
  const long p = (long)Aineq.rows();

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
      QG[i][j] = G(i, j);
    }
  }
  // g0
  for (long i = 0; i < n; ++i) {
    Qg0[i] = g0(i);
  }

  // Aeq z = beq  ->  CE^T x + ce0 = 0
  // CE = Aeq^T, ce0 = -beq
  for (long i = 0; i < n; ++i) {
    for (long j = 0; j < m; ++j) {
      QCE[i][j] = Aeq(j, i);
    }
  }
  for (long j = 0; j < m; ++j) {
    Qce0[j] = -beq(j);
  }

  // Aineq z <= bineq  ->  (-Aineq) z + bineq >= 0
  // CI = (-Aineq)^T, ci0 = bineq
  for (long i = 0; i < n; ++i) {
    for (long j = 0; j < p; ++j) {
      QCI[i][j] = -Aineq(j, i);
    }
  }
  for (long j = 0; j < p; ++j) {
    Qci0[j] = bineq(j);
  }

  const double res = solve_quadprog(QG, Qg0, QCE, Qce0, QCI, Qci0, Qx);
  if (!std::isfinite(res)) {
    return false;
  }

  xOpt.resize(n);
  for (long i = 0; i < n; ++i) {
    xOpt(i) = Qx[i];
  }
  return xOpt.allFinite();
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
  Eigen::Matrix<double,12,12> A = Eigen::Matrix<double,12,12>::Identity();
  A.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
  A.block<3,3>(6,9) = Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double,12,1> c = Eigen::Matrix<double,12,1>::Zero();
  c.segment<3>(3) = dt * settings_.g;

  // 每步 B_k（12x12）
  std::vector<Eigen::Matrix<double,12,12>> Bk(N, Eigen::Matrix<double,12,12>::Zero());
  const Eigen::Matrix3d Iw_inv = in.Iw_inv;  // 世界系惯量逆（由 solveFromDogWrench 计算）

  for (int k = 0; k < N; ++k) {
    // v update: dt/m * f
    for (int leg = 0; leg < 4; ++leg) {
      Bk[k].block<3,3>(3, 3*leg) = (dt / settings_.mass) * Eigen::Matrix3d::Identity();
    }

    // w update: dt * Iw_inv * (r x f)
    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector3d& r = in.rFeet[k][leg];
      Bk[k].block<3,3>(9, 3*leg) = dt * Iw_inv * skew3(r);
    }
  }

  // ====== 预测矩阵：x_k = Phi_k x0 + S_k z + d_k ======
  // z = [u0..u_{N-1}] ∈ R^{12N}
  const int nX = 12;
  const int nU = 12;
  const int nZ = nU * N;

  std::vector<Eigen::Matrix<double,12,12>> Phi(N+1);
  std::vector<Eigen::MatrixXd> Sk(N+1);
  std::vector<Eigen::Matrix<double,12,1>> dk(N+1);

  Phi[0].setIdentity();
  Sk[0] = Eigen::MatrixXd::Zero(nX, nZ);
  dk[0].setZero();

  for (int k = 0; k < N; ++k) {
    Phi[k+1] = A * Phi[k];
    Sk[k+1] = A * Sk[k];
    Sk[k+1].block(0, k*nU, nX, nU) += Bk[k];
    dk[k+1] = A * dk[k] + c;
  }

  // ====== 构造 QP：min 0.5 z^T G z + g^T z ======
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(nZ, nZ);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(nZ);

  // (1) 状态跟踪代价：sum_{k=1..N} (x_k-xRef_k)^T Q (x_k-xRef_k)
  for (int k = 1; k <= N; ++k) {
    Eigen::Matrix<double,12,1> e = Phi[k] * in.x0 + dk[k] - in.xRef[k];

    // cost = (S z + e)^T Q (S z + e)
    // => G += 2 S^T Q S
    // => g += 2 S^T Q e
    G.noalias() += 2.0 * Sk[k].transpose() * settings_.Q * Sk[k];
    g.noalias() += 2.0 * Sk[k].transpose() * (settings_.Q * e);
  }

  // (2) 力正则：sum u_k^T R u_k
  for (int k = 0; k < N; ++k) {
    G.block(k*nU, k*nU, nU, nU).noalias() += 2.0 * settings_.R;
  }

  // (3) 力变化率：sum (u_k-u_{k-1})^T S (u_k-u_{k-1})
  // k>=1 的差分
  for (int k = 1; k < N; ++k) {
    // (u_k-u_{k-1})^T S (u_k-u_{k-1})
    // expand -> add blocks
    G.block(k*nU, k*nU, nU, nU).noalias()           += 2.0 * settings_.S;
    G.block((k-1)*nU, (k-1)*nU, nU, nU).noalias()   += 2.0 * settings_.S;
    G.block(k*nU, (k-1)*nU, nU, nU).noalias()       += -2.0 * settings_.S;
    G.block((k-1)*nU, k*nU, nU, nU).noalias()       += -2.0 * settings_.S;
  }
  // k=0 与 last_u0_ 的差分
  // (u0-last)^T S (u0-last) => add 2S to G00, add -2S*last to g0
  G.block(0, 0, nU, nU).noalias() += 2.0 * settings_.S;
  g.segment(0, nU).noalias()      += -2.0 * settings_.S * last_u0_;

  // Hessian regularization
  G.diagonal().array() += settings_.regularization;

  // ====== 约束：contact schedule 必须预测 ======
  // swing leg: f=0 (eq)
  // stance leg: friction pyramid + fz bounds (ineq)
  int nEq = 0;
  int nIneq = 0;
  for (int k = 0; k < N; ++k) {
    for (int leg = 0; leg < 4; ++leg) {
      if (in.contact[k][leg] == 0) {
        nEq += 3;
      } else {
        nIneq += 6;
      }
    }
  }

  Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(nEq, nZ);
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(nEq);

  Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(nIneq, nZ);
  Eigen::VectorXd bineq = Eigen::VectorXd::Zero(nIneq);

  int eqRow = 0;
  int ineqRow = 0;

  for (int k = 0; k < N; ++k) {
    for (int leg = 0; leg < 4; ++leg) {
      const int col = k*nU + 3*leg;

      if (in.contact[k][leg] == 0) {
        // swing: fx=fy=fz=0
        Aeq(eqRow+0, col+0) = 1.0;
        Aeq(eqRow+1, col+1) = 1.0;
        Aeq(eqRow+2, col+2) = 1.0;
        // beq=0
        eqRow += 3;
      } else {
        // stance friction pyramid
        // fx - mu fz <= 0
        Aineq(ineqRow, col+0) =  1.0;
        Aineq(ineqRow, col+2) = -settings_.mu;
        bineq(ineqRow) = 0.0;
        ineqRow++;

        // -fx - mu fz <= 0
        Aineq(ineqRow, col+0) = -1.0;
        Aineq(ineqRow, col+2) = -settings_.mu;
        bineq(ineqRow) = 0.0;
        ineqRow++;

        // fy - mu fz <= 0
        Aineq(ineqRow, col+1) =  1.0;
        Aineq(ineqRow, col+2) = -settings_.mu;
        bineq(ineqRow) = 0.0;
        ineqRow++;

        // -fy - mu fz <= 0
        Aineq(ineqRow, col+1) = -1.0;
        Aineq(ineqRow, col+2) = -settings_.mu;
        bineq(ineqRow) = 0.0;
        ineqRow++;

        // fz >= fzMin  -> -fz <= -fzMin
        Aineq(ineqRow, col+2) = -1.0;
        bineq(ineqRow) = -settings_.fzMin;
        ineqRow++;

        // fz <= fzMax
        Aineq(ineqRow, col+2) =  1.0;
        bineq(ineqRow) =  settings_.fzMax;
        ineqRow++;
      }
    }
  }

  // ====== 解 QP ======
  Eigen::VectorXd zOpt;
  zOpt.setZero(nZ);

  const bool ok = solveDenseQpQuadProgpp(G, g, Aeq, beq, Aineq, bineq, zOpt);
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
  out.u0 = zOpt.segment(0, nU);
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
  in.Iw_inv = Iw.inverse();

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