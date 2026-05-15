#pragma once

#include <ocs2_core/Types.h>          // ocs2::scalar_t, vector_t, matrix_t
#include <Eigen/Core>
#include <Eigen/Dense>

#include <array>
#include <memory>
#include <vector>

#include <unitree_guide_controller/common/mathTypes.h>  // Vec3 Vec34 RotMat VecInt4 等

struct ConvexMpcSettings {
  // ====== MPC horizon ======
  // horizon steps
  int N = 25;              // 0.5s horizon with dt=0.02
  // horizon dt [s]
  double dt = 0.02;

  // ====== physical ======
  double mass = 40.5;

  // Body inertia about COM in BODY frame (Ib)
  Eigen::Matrix3d Ib = Eigen::Matrix3d::Identity();

  // Body->COM offset in BODY frame (pcb_B): COM = body + R * pcb_B
  // 与你 BalanceCtrl 里的 pcb_ 含义一致：机身坐标系下质心偏置
  Eigen::Vector3d pcb_B = Eigen::Vector3d::Zero();

  // gravity (world)
  Eigen::Vector3d g = Eigen::Vector3d(0.0, 0.0, -9.81);

  // ====== contact / friction ======
  double mu = 0.4;
  double fzMin = 0.0;
  double fzMax = 350.0;

  // ====== gait schedule prediction (for 0.5s horizon 必须有) ======
  // 这里为了不改你现有接口，默认用“trot 对角交替预测器”生成 contact schedule
  // 若你后续要完全对齐 wave_generator_->phase_(i)，只需要用 wave_generator 的相位替换 basePhase 推断即可（见 cpp 里注释）
  double gaitPeriod = 0.5;     // gait period [s]
  double stanceRatio = 0.6;    // stance ratio in one period
  // trot phase offsets: FR, FL, RR, RL
  // FR+RL 同相；FL+RR 反相（0.5）
  Eigen::Matrix<double, 4, 1> phaseOffset = (Eigen::Matrix<double,4,1>() << 0.0, 0.5, 0.5, 0.0).finished();

  // ====== cost weights（先留接口） ======
  // state x = [p(3), v(3), rpy(3), w(3)]
  Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double,12,12>::Zero();
  // input u = [f_FR, f_FL, f_RR, f_RL]
  Eigen::Matrix<double, 12, 12> R = Eigen::Matrix<double,12,12>::Zero();
  // force-rate cost (u_k - u_{k-1})
  Eigen::Matrix<double, 12, 12> S = Eigen::Matrix<double,12,12>::Zero();

  double regularization = 1e-8;

  // solver safety
  bool enableFallbackToLast = true;    // QP失败时是否回退上一帧 last_u0_
  bool enableContactPrediction = true; // 若你未来传入外部 schedule，可关掉

  // ====== HPIPM settings ======
  int hpipm_iter_max = 30;
  double hpipm_alpha_min = 1e-12;
  double hpipm_mu0 = 1e1;
  double hpipm_tol_stat = 1e-6;
  double hpipm_tol_eq = 1e-8;
  double hpipm_tol_ineq = 1e-8;
  double hpipm_tol_comp = 1e-8;
  double hpipm_reg_prim = 1e-12;
  int hpipm_warm_start = 0;
  int hpipm_pred_corr = 1;
  int hpipm_ric_alg = 0;       // 0: square-root Riccati; 和 OCS2 默认一致
  bool hpipm_verbose = false;
};

// 纯 OCS2 风格输入（你可以先用最小集合）
struct ConvexMpcInputOcs2 {
  ocs2::scalar_t t = 0.0;
  ocs2::vector_t x0;                               // 12x1: [p v rpy w] in G
  std::vector<ocs2::vector_t> xRef;                // size N+1, each 12x1
  std::vector<std::array<int,4>> contact;          // size N, 0/1
  std::vector<std::array<Eigen::Vector3d,4>> rFeet;// size N, r_i = p_foot - p_com in G

  // 为了 SRBD 里 I_world = R * Ib * R^T
  Eigen::Matrix3d R_GB = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Iw_inv = Eigen::Matrix3d::Identity();
};

struct ConvexMpcOutputOcs2 {
  ocs2::vector_t u0; // 12x1: [f_FR, f_FL, f_RR, f_RL]
  bool success = false;

  int hpipm_status = -1;
  int hpipm_iter = -1;
};

class ConvexMpcSolver {
public:
  explicit ConvexMpcSolver(const ConvexMpcSettings& settings);
  ~ConvexMpcSolver();

  // 允许在运行时改 MPC 参数
  void setSettings(const ConvexMpcSettings& settings);
  const ConvexMpcSettings& settings() const { return settings_; }

  void reset();

  // 改名：避免 solve 宏污染
  ConvexMpcOutputOcs2 solveMpc(const ConvexMpcInputOcs2& in);

  Vec34 solveFromDogWrench(
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
      const Vec3& v_ref_G           // 机身期望速度
  );

private:
  struct HpipmWorkspace;

  ConvexMpcSettings settings_;
  ocs2::vector_t last_u0_;  // warm start / debug
  bool has_last_solution_ = false;

  std::unique_ptr<HpipmWorkspace> hpipm_;

  // 内部工具：根据当前contact推断trot相位（无 phase 输入时的最小可用方案）
  static double inferBasePhaseFromContact(const VecInt4& contact_now);

  Vec34 makeFallbackForces(const VecInt4& contact_now) const;
};