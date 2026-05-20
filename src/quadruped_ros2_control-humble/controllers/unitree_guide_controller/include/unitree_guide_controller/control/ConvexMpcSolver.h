#pragma once

#include <array>
#include <memory>
#include <vector>

#include <unitree_guide_controller/common/mathTypes.h>

// 论文标准 MPC 状态：
// x = [Theta(3), p_com(3), omega(3), v_com(3), g_z(1)]
// 总维度 = 13
using Vec13 = Eigen::Matrix<double, 13, 1>;
using Mat13 = Eigen::Matrix<double, 13, 13>;
using Mat1312 = Eigen::Matrix<double, 13, 12>;

struct ConvexMpcSettings {
  // ====== MPC horizon ======
  // MPC = Model Predictive Control，模型预测控制。
  // horizon = 预测时域。
  // N = 最大预测步数，不是最终一定使用的预测步数。
  // 实际使用的 N 会在 .cpp 里根据 control_dt 和 gait_period 裁剪。
  // 例如 N=25, dt=0.02 时，理论预测时域是 25*0.02=0.5s。
  // 但如果 enforceHalfGaitHorizon=true，实际时域不能超过半个步态周期。
  int N = 25;

  // ====== physical ======
  // mass = 机器人机身等效质量，单位 kg。
  // MPC 在质心动力学里用它计算：
  // v_dot = e_z * g_z + sum(f_i) / mass
  double mass = 40.5;

  // Body inertia about COM in BODY frame (Ib)
  // Ib = Inertia of body，机身绕质心 COM 的惯量矩阵。
  // COM = Center of Mass，质心。
  // BODY frame = 机身坐标系，通常记作 B 系。
  // 这里 Ib 是 B 系下表达的 3x3 转动惯量矩阵。
  // 在 .cpp 中会通过 Iw = R_GB * Ib * R_GB.transpose() 转成世界系惯量。
  Mat3 Ib = Mat3::Identity();

  // Body->COM offset in BODY frame (pcb_B): COM = body + R * pcb_B
  // pcb_B 表示“从机身原点 body 到质心 COM 的偏移向量”，在 B 系下表达。
  //
  // p = position，位置
  // c = center of mass，质心
  // b = body，机身原点
  // B = BODY frame，机身坐标系
  //
  // 所以 pcb_B 可以理解为：
  // p_cb^B 或者 p_COM_from_body_in_B
  //
  // 与你 BalanceCtrl 里的 pcb_ 含义一致：机身坐标系下质心偏置。
  // 如果质心和机身原点重合，就是零向量。
  Vec3 pcb_B = Vec3::Zero();

  // gravity (world)
  // g = gravity，重力加速度向量，世界系表达。
  // 论文状态里只使用 g(3)，也就是 z 方向重力标量。
  // 默认世界系 z 轴向上，所以 g_z = -9.81。
  Vec3 g = Vec3(0.0, 0.0, -9.81);

  // ====== contact / friction ======
  // mu = friction coefficient，摩擦系数。
  // 用于摩擦锥/摩擦金字塔约束：
  // |fx| <= mu * fz
  // |fy| <= mu * fz
  double mu = 0.4;

  // fzMin = normal force minimum，法向支撑力下限。
  // fz 是足底接触力在 z 方向的分量。
  // 支撑腿通常要求 fz >= fzMin。
  double fzMin = 0.0;

  // fzMax = normal force maximum，法向支撑力上限。
  // 防止单条腿输出过大的竖直力。
  double fzMax = 350.0;

  // 严格使用参考论文的 r_i 近似时，预测时域不能超过半个步态周期
  // 即避免同一条腿在预测域内经历“支撑 -> 摆动 -> 再次支撑”
  //
  // enforce = 强制执行
  // HalfGaitHorizon = 半个步态周期长度的预测时域限制
  //
  // true:
  //   .cpp 中实际 N 会被限制为：
  //   N <= floor(0.5 * gait_period / control_dt)
  //
  // false:
  //   实际 N 直接使用 settings.N。
  bool enforceHalfGaitHorizon = true;

  // ====== cost weights ======
  // state x = [Theta(3), p_com(3), omega(3), v_com(3), g_z(1)]
  //
  // Theta = [roll, pitch, yaw]
  // p_com = 质心位置
  // omega = 机身角速度
  // v_com = 质心线速度
  // g_z   = z 方向重力标量
  //
  // Q 是状态误差权重矩阵。
  // Q 越大，MPC 越强烈地追踪参考状态 xRef。
  Mat13 Q = Mat13::Zero();

  // input u = [f_FR, f_FL, f_RR, f_RL]
  //
  // u 是 MPC 控制输入，共 12 维：
  // f_FR = front right leg force，右前腿三维足底力
  // f_FL = front left  leg force，左前腿三维足底力
  // f_RR = rear  right leg force，右后腿三维足底力
  // f_RL = rear  left  leg force，左后腿三维足底力
  //
  // R 是输入力大小惩罚矩阵。
  // R 越大，MPC 越不愿意使用过大的足底力。
  Mat12 R = Mat12::Zero();

  // S 是输入变化率惩罚矩阵。
  // 当前采用最小改动版本：
  // 只惩罚当前真正会执行的第一步输入变化率：
  // (u0 - last_u0)^T S (u0 - last_u0)
  //
  // 这样不需要增广状态，也不引入 u_k 与 u_{k-1} 的跨 stage 耦合。
  Mat12 S = Mat12::Zero();

  // regularization = 数值正则化系数。
  // 加到 QP Hessian 对角线上，避免矩阵病态或半正定导致求解器不稳定。
  // 数值太大：会影响原始优化目标。
  // 数值太小：可能起不到稳定作用。
  double regularization = 1e-8;

  // solver safety
  // enableFallbackToLast = 求解失败时是否回退到上一帧成功求解的足底力。
  //
  // true:
  //   如果 HPIPM 求解失败，但 last_u0_ 有有效值，则输出 last_u0_。
  //
  // false:
  //   求解失败直接返回失败，外层再走 makeFallbackForces()。
  bool enableFallbackToLast = true;

  // ====== HPIPM settings ======
  // HPIPM = High-Performance Interior Point Method。
  // 它是面向最优控制和 MPC 的高性能内点法 QP 求解器。

  // hpipm_iter_max = HPIPM 最大迭代次数。
  // 迭代次数越大，求解器越有机会收敛，但耗时也可能增加。
  int hpipm_iter_max = 30;

  // hpipm_alpha_min = HPIPM 线搜索最小步长。
  // alpha 是内点法线搜索中的步长参数。
  // 太大可能提前失败，太小可能允许非常小的步长继续迭代。
  double hpipm_alpha_min = 1e-12;

  // hpipm_mu0 = 初始 barrier parameter，内点法初始障碍参数。
  // mu 越大，初始点更偏向严格可行域内部。
  double hpipm_mu0 = 1e1;

  // hpipm_tol_stat = stationarity tolerance，驻点残差容忍度。
  // stat = stationarity，表示 KKT 条件里的梯度平衡误差。
  double hpipm_tol_stat = 1e-6;

  // hpipm_tol_eq = equality constraint tolerance，等式约束残差容忍度。
  // eq = equality。
  // 对应动力学等式约束 x_{k+1}=A_k*x_k+B_k*u_k 的满足精度。
  double hpipm_tol_eq = 1e-8;

  // hpipm_tol_ineq = inequality constraint tolerance，不等式约束残差容忍度。
  // ineq = inequality。
  // 对应摩擦锥、fz上下限等不等式约束的满足精度。
  double hpipm_tol_ineq = 1e-8;

  // hpipm_tol_comp = complementarity tolerance，互补条件残差容忍度。
  // comp = complementarity。
  // 内点法 KKT 条件中的互补松弛误差。
  double hpipm_tol_comp = 1e-8;

  // hpipm_reg_prim = primal regularization，原始变量正则化。
  // prim = primal，指 QP 中的原始优化变量。
  // 用于改善数值稳定性。
  double hpipm_reg_prim = 1e-12;

  // hpipm_warm_start = 是否启用 HPIPM 内部 warm start。
  // warm start = 热启动，用上一次求解结果辅助本次求解。
  // 这里 0 表示不启用 HPIPM 内部热启动。
  int hpipm_warm_start = 0;

  // hpipm_pred_corr = predictor-corrector 开关。
  // pred = predictor，预测步
  // corr = corrector，校正步
  // 1 通常表示启用 Mehrotra predictor-corrector 类型策略。
  int hpipm_pred_corr = 1;

  // hpipm_ric_alg = Riccati algorithm，Riccati 递推算法类型。
  // ric = Riccati。
  // 0: square-root Riccati；和 OCS2 默认一致。
  // square-root Riccati 数值稳定性通常更好，但对正定性要求更严格。
  int hpipm_ric_alg = 0;       // 0: square-root Riccati; 和 OCS2 默认一致

  // hpipm_verbose = 是否打印 HPIPM 调试信息。
  // true 时会输出 status、iter、N、dt 等。
  // 实机运行时建议 false，调试求解失败时可以 true。
  bool hpipm_verbose = false;
};

// MPC 输入：不再强制使用 OCS2 类型
// ConvexMpcInput 是 solveMpc() 的直接输入。
// 这里的数据应该在 solveFromDogWrench() 里提前构造好。
// solveMpc() 尽量保持为纯数学求解函数，不再负责大量工程合法性检查。
struct ConvexMpcInput {
  // 当前实际 MPC horizon steps
  //
  // N = 本次真正送入 HPIPM 的预测步数。
  // 注意：这里的 N 不是 settings.N。
  // settings.N 是最大允许步数；
  // 这里的 N 是经过半步态周期限制后得到的实际步数。
  int N = 0;

  // 当前控制周期
  //
  // dt = delta time，离散时间步长，单位 s。
  // 它来自主控制周期 control_dt。
  // 离散动力学中所有 dt 都使用这个值。
  double dt = 0.0;

  // 13x1: [Theta, p_com, omega, v_com, g_z] in G/P
  //
  // x0 = initial state，当前初始状态。
  //
  // Theta = [roll, pitch, yaw]
  // p_com = 质心位置
  // omega = 机身角速度
  // v_com = 质心线速度
  // g_z   = z方向重力标量
  Vec13 x0 = Vec13::Zero();

  // size N+1, each 13x1
  //
  // xRef = state reference trajectory，状态参考轨迹。
  // 长度必须是 N+1。
  //
  // 原因：
  // N 个控制输入 u_0 ... u_{N-1}
  // 会对应 N+1 个状态点 x_0 ... x_N。
  std::vector<Vec13> xRef;

  // size N, 0/1
  //
  // contact = contact schedule，接触时序表。
  // contact[k][leg] 表示第 k 个预测步第 leg 条腿是否支撑。
  //
  // 1 = stance，支撑腿，允许施加足底力，并受摩擦约束/fz约束限制。
  // 0 = swing，摆动腿，足底力强制为 0。
  //
  // leg 顺序沿用你的工程：
  // 0: FR，右前腿
  // 1: FL，左前腿
  // 2: RR，右后腿
  // 3: RL，左后腿
  std::vector<std::array<int, 4>> contact;

  // size N, r_i = p_foot - p_com in G/P
  //
  // rFeet = 每个预测步、每条腿的力臂向量。
  //
  // r_i = p_foot_i - p_com
  //
  // p_foot_i = 第 i 条腿的足底固定接触点
  // p_com    = 参考质心位置
  std::vector<std::array<Vec3, 4>> rFeet; // 力臂r

  // 为了 SRBD 里 I_world = R * Ib * R^T
  //
  // Iw_inv = inverse of world-frame inertia。
  // Iw     = world-frame inertia，世界系下的机身转动惯量。
  // inv    = inverse，矩阵逆。
  //
  // Iw = R_GB * Ib * R_GB^T
  // Iw_inv = Iw^{-1}
  //
  // 它用于角速度动力学：
  // omega_dot = Iw_inv * sum(r_i × f_i)
  Mat3 Iw_inv = Mat3::Identity();
};

struct ConvexMpcOutput {
  // u0 = 当前时刻要执行的第一步足底力。
  //
  // u = input，MPC 控制输入。
  // 0 = 第 0 个预测步，也就是当前控制周期实际拿来用的力。
  //
  // 12x1: [f_FR, f_FL, f_RR, f_RL]
  //
  // 每条腿 3 维：
  // f_leg = [fx, fy, fz]
  Vec12 u0 = Vec12::Zero(); // 12x1: [f_FR, f_FL, f_RR, f_RL]

  // success = 求解是否成功。
  //
  // true:
  //   HPIPM 正常返回，并且 u0 有限。
  //
  // false:
  //   求解失败或输出无效。
  bool success = false;
};

class ConvexMpcSolver {
public:
  explicit ConvexMpcSolver(const ConvexMpcSettings& settings);
  ~ConvexMpcSolver();

  void setSettings(const ConvexMpcSettings& settings);
  const ConvexMpcSettings& settings() const { return settings_; }

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

  ConvexMpcSettings settings_;

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