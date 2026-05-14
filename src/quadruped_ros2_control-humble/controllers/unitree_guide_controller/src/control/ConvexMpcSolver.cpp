#include "unitree_guide_controller/control/ConvexMpcSolver.h"
#include "quadProgpp/QuadProg++.hh"

#include <unitree_guide_controller/common/mathTools.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static inline Eigen::Matrix3d skew3(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
      -v.y(), v.x(), 0.0;
  return S;
}

ConvexMpcSolver::ConvexMpcSolver(const ConvexMpcSettings& settings)
  : settings_(settings) {
  last_u0_.setZero(12);
}

void ConvexMpcSolver::setSettings(const ConvexMpcSettings& settings) {
  settings_ = settings;
}

void ConvexMpcSolver::reset() {
  last_u0_.setZero(12);
}

// 改名：solveMpc（避免 solve 被宏替换成 lu_solve）
ConvexMpcOutputOcs2 ConvexMpcSolver::solveMpc(const ConvexMpcInputOcs2& in) {
  (void)in;

  ConvexMpcOutputOcs2 out;
  out.u0.setZero(12);
  out.success = false;

  // TODO: 这里填 convex MPC QP 拼装
  // 临时先返回 last_u0_
  out.u0 = last_u0_;
  out.success = true;
  return out;
}

Vec34 ConvexMpcSolver::solveFromUnitreeWrench(
    const Vec3& dd_pcd_G,
    const Vec3& d_wbd_G,
    const RotMat& B2P_RotMat,
    const Vec34& rFeet_P,
    const VecInt4& contact_now,
    const Vec3& p_body_G,
    const Vec3& v_body_G,
    const RotMat& R_GB,
    const Vec3& gyro_G,
    const RotMat& Rd_GB,
    const Vec3& v_ref_G) {

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

  // contact (N)
  in.contact.resize(settings_.N);
  for (int k = 0; k < settings_.N; ++k) {
    in.contact[k] = { contact_now(0), contact_now(1), contact_now(2), contact_now(3) };
  }

  // rFeet (N)
  in.rFeet.resize(settings_.N);
  for (int k = 0; k < settings_.N; ++k) {
    std::array<Eigen::Vector3d,4> rk;
    for (int leg = 0; leg < 4; ++leg) {
      rk[leg] = Eigen::Vector3d(rFeet_P(0,leg), rFeet_P(1,leg), rFeet_P(2,leg));
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

  last_u0_ = out.u0;
  return force_feet_P;
}