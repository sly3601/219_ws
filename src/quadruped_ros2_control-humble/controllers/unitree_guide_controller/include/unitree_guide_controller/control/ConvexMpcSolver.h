#pragma once

#include <ocs2_core/Types.h>          // ocs2::scalar_t, vector_t, matrix_t
#include <Eigen/Core>
#include <array>
#include <vector>

#include <unitree_guide_controller/common/mathTypes.h>  // Vec3 Vec34 RotMat VecInt4 等

struct ConvexMpcSettings {
  int N = 10;             // horizon steps
  double dt = 0.02;       // horizon dt [s]

  double mass = 40.5;
  Eigen::Matrix3d Ib = Eigen::Matrix3d::Identity();

  double mu = 0.4;
  double fzMin = 0.0;
  double fzMax = 350.0;

  // cost weights（先留接口）
  Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double,12,12>::Zero();
  Eigen::Matrix<double, 12, 12> R = Eigen::Matrix<double,12,12>::Zero();
  Eigen::Matrix<double, 12, 12> S = Eigen::Matrix<double,12,12>::Zero();

  double regularization = 1e-6;
};

struct ConvexMpcInputOcs2 {
  ocs2::scalar_t t = 0.0;
  ocs2::vector_t x0;                               // 12x1: [p v rpy w] in G
  std::vector<ocs2::vector_t> xRef;                // size N+1, each 12x1
  std::vector<std::array<int,4>> contact;          // size N, 0/1
  std::vector<std::array<Eigen::Vector3d,4>> rFeet;// size N, r_i = p_foot - p_com in G
};

struct ConvexMpcOutputOcs2 {
  ocs2::vector_t u0; // 12x1: [f_FR, f_FL, f_RR, f_RL]
  bool success = false;
};

class ConvexMpcSolver {
public:
  explicit ConvexMpcSolver(const ConvexMpcSettings& settings);

  void setSettings(const ConvexMpcSettings& settings);
  const ConvexMpcSettings& settings() const { return settings_; }

  void reset();

  // 改名：避免 solve 宏污染
  ConvexMpcOutputOcs2 solveMpc(const ConvexMpcInputOcs2& in);

  Vec34 solveFromDogWrench(
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
      const Vec3& v_ref_G
  );

private:
  ConvexMpcSettings settings_;
  ocs2::vector_t last_u0_;  // warm start / debug
};