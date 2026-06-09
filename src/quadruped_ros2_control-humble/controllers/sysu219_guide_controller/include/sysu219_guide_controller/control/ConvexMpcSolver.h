#pragma once

#include <array>
#include <memory>
#include <vector>

#include <sysu219_guide_controller/common/mathTypes.h>

// 论文标准 MPC 状态：
// x = [Theta(3), p_com(3), omega(3), v_com(3), g_z(1)]
// 总维度 = 13
using Vec13 = Eigen::Matrix<double, 13, 1>;
using Mat13 = Eigen::Matrix<double, 13, 13>;
using Mat1312 = Eigen::Matrix<double, 13, 12>;

// MPC 输入：不再强制使用 OCS2 类型
// ConvexMpcInput 是 solveMpc() 的直接输入。
// 这里的数据应该在 solveFromDogWrench() 里提前构造好。
struct ConvexMpcInput {
  int N = 0;                                // 当前实际 MPC horizon steps，经过计算得到最终的实际步数
  double dt = 0.0;                          // dt = delta time，离散时间步长，单位 s。
  Vec13 x0 = Vec13::Zero();                 // x0 = initial state，当前初始状态。
  std::vector<Vec13> xRef;                  // xRef = reference state trajectory，参考状态轨迹，即论文中的D
  std::vector<std::array<int, 4>> contact;  // contact[k][leg] 表示第 k 个预测步第 leg 条腿是否支撑。顺序是 FR, FL, RR, RL，1 支撑，0 摆动。
  std::vector<std::array<Vec3, 4>> rFeet;   // 力臂r
  Mat3 Iw_inv = Mat3::Identity();           // 世界系下的机身转动惯量逆矩阵
};

// MPC 输出
struct ConvexMpcOutput {
  Vec12 u0 = Vec12::Zero(); // 12x1: [f_FR, f_FL, f_RR, f_RL]
  bool success = false;
};

class ConvexMpcSolver {
public:
  explicit ConvexMpcSolver();
  ~ConvexMpcSolver();

  void reset();

  ConvexMpcOutput solveMpc(const ConvexMpcInput& in);
  
    Vec34 solveFromDogWrench(
      const Vec3& dd_pcd_G,         // 机身期望加速度（世界系）
      const Vec34& foot_hold_G,     // 每条腿最近一次落地/支撑时记录的 G 系足底接触点
      const Vec34& foot_end_G,      // 当前周期摆动腿最终落脚点（G系）
      const VecInt4& contact_now,   // 当前 4 足接触状态，1 支撑，0 摆动
      const Vec4& phase_now,        // 当前 4 足支撑/摆动相内部进度 [0,1]
      double control_dt,            // 主控制周期
      double gait_period,           // wave_generator_->get_t()
      double stance_ratio,          // wave_generator_->get_t_stance() / wave_generator_->get_t()
      const Vec3& p_body_G,         // 机身位置（世界系）
      const Vec3& v_body_G,         // 机身速度（世界系）
      const RotMat& R_GB,           // 机身姿态（旋转矩阵）
      const Vec3& gyro_G,           // 机身角速度（世界系）
      const RotMat& Rd_GB,          // 机身期望姿态
      const Vec3& v_ref_G           // 机身期望速度
  );

private:
  struct HpipmWorkspace;

  void rebuildFixedMatrices();

  const int nx_stage;       // 每个状态维度
  const int nu_stage;       // 每个输入维度
  const int rows_per_leg;   // 每条腿的行数
  const int ng_stage;       // 每步一般线性约束行数
  const double INF;

  int N;
  double mass;
  Mat3 Ib;
  Vec3 pcb_B;
  Vec3 g;
  double mu;
  double fzMin;
  double fzMax;
  bool enforceHalfGaitHorizon;
  Mat13 Q = Mat13::Zero();
  Mat12 R = Mat12::Zero();
  Mat12 S = Mat12::Zero();
  double regularization;
  bool enableFallbackToLast;
  // hpipm_verbose = 是否打印 HPIPM 调试信息。
  // true 时会输出 status、iter、N、dt 等。
  // 实机运行时建议 false，调试求解失败时可以 true。
  bool hpipm_verbose = false;

  // last_u0_ 现在用于 HPIPM 失败时 fallback，也用于输入变化率项的上一帧输入。
  Vec12 last_u0_ = Vec12::Zero();
  bool has_last_solution_ = false;

  std::unique_ptr<HpipmWorkspace> hpipm_;

  MatX Q_cost_;
  MatX R_cost_;
  MatX R_cost_rate_;
  VecX r_cost_zero_;
  VecX r_cost_rate_;

  MatX C_zero_constraint_;
  MatX D_force_constraint_;

  VecX lower_bound_stance_;
  VecX upper_bound_stance_;
  VecX lower_bound_swing_;
  VecX upper_bound_swing_;

  Vec34 makeFallbackForces(const VecInt4& contact_now) const;
};