#pragma once

#include "sysu219_guide_controller/control/WbcController.h"

namespace sysu219::wbc {

// 主依据：北航《基于稳定性的仿生四足机器人控制系统设计》第4.3节。
// 输入MPC足底力和WBC加速度，只松弛基座6维加速度与支撑足力，保留12维关节加速度。
// 阅读顺序：输入检查 → 支撑足打包 → 目标函数 → 等式/不等式约束 → 求解 → 恢复输出。
// C是动力学偏置力，Q_WBC/Q_MPC是权重，G/C_E/c_e/C_I/c_i沿用论文的QP命名。
// O为世界系；腿顺序、18维广义量和加速度约定均与WbcInput保持一致。

// 最多4只支撑足：变量6+3*n_c，最多18维；基座动力学等式6条，足力不等式最多24条。
// 动态尺寸只改变有效范围，Eigen数据空间随对象预留，接触切换时不重新分配。
using RelaxationMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::ColMajor, 18, 18>;
using RelaxationVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 18, 1>;
using RelaxationInequalityMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                                Eigen::ColMajor, 18, 24>;
using RelaxationInequalityVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 24, 1>;
using ContactConstraintMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                             Eigen::ColMajor, 20, 12>;
using ContactConstraintVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 20, 1>;


struct ContactForceLimits {
  // 用于生成论文中的单足C_i及其上下界；接入时与MPC使用相同参数。
  double mu = 0.35;        // 摩擦系数，四棱面线性近似。
  double f_z_min = 0.0;    // 接触面法向力下界，单位N。
  double f_z_max = 350.0;  // 法向力上界；卸载支撑足时由外部调整。

  // 接触力约束的坐标变换：三列为世界系下的两条切向和法向，单位阵对应水平地面。
  Eigen::Matrix3d R_OC = Eigen::Matrix3d::Identity();
};


struct RelaxationSettings {
  // Q_WBC仅惩罚基座6维加速度修正，关节加速度不参与松弛。
  // Q_MPC统一惩罚所有支撑足的力修正。
  // 权重越大，对应修正越不容易发生；两种量的单位不同，不代表信任百分比。
  double Q_WBC = 1.0;
  double Q_MPC = 0.005;

  // 实现所需的数值检查参数，不属于论文中的优化变量。
  double absolute_feasibility_tolerance = 1e-6;
  double relative_feasibility_tolerance = 1e-9;
};


struct RelaxationInput {
  // 论文式(4.34)：完整18维浮动基模型，不能用MPC单刚体模型代替。
  Mat18 M = Mat18::Constant(kUnset);          // 质量矩阵，18×18。
  Vec18 C = Vec18::Constant(kUnset);          // 重力、科氏力等偏置力，18×1。
  Vec18 qddot_cmd = Vec18::Constant(kUnset);  // WBC输出的广义加速度指令。

  // 世界系MPC足底反力；每列对应一只脚，后续只读取支撑足。
  // 传入地面对机器人的力。现有calcTau会取反，不能用其取反后的变量填这里。
  Feet f_MPC_O = Feet::Constant(kUnset);
  Contact contact{{1, 1, 1, 1}};
  std::array<FootJacobian, 4> J_f_O{{
      FootJacobian::Constant(kUnset), FootJacobian::Constant(kUnset),
      FootJacobian::Constant(kUnset), FootJacobian::Constant(kUnset)}};
  std::array<ContactForceLimits, 4> force_limits{};

  // 仅用于记录松弛后的支撑足加速度，不作为QP约束或成功判据。
  Feet Jdot_f_qdot_O = Feet::Constant(kUnset);

  // 式(4.62)的关节PD由接口提供，不参与优化；使用电机内部PD时保持零，避免重复叠加。
  Vec12 tau_PD_j = Vec12::Zero();
};


struct RelaxationQp {
  // X = [delta_qddot_base(6); delta_f(3*n_c)]，不包含关节加速度修正。
  // 论文式(4.58)：以下矩阵直接按QuadProg++的列约束格式存储。
  RelaxationMatrix G;              // 二次目标矩阵，n_X×n_X。
  RelaxationMatrix C_E;            // n_X×6，仅基座动力学等式。
  RelaxationVector c_e;            // 等式常量，放在等式左侧。
  RelaxationInequalityMatrix C_I;  // 不等式系数，n_X×n_I；每列是一条不等式。
  RelaxationInequalityVector c_i;  // 不等式常量，放在不等式左侧。

  // 论文式(4.51)～(4.53)：只包含当前支撑足的力及其约束。
  WbcTaskMatrix J_c;               // 支撑足雅可比，3*n_c×18。
  WbcTaskVector f_MPC_O;           // 压紧排列后的MPC支撑足反力，3*n_c×1。
  ContactConstraintMatrix C_A;     // 单足C_i按对角块排列，5*n_c×3*n_c。
  ContactConstraintVector c_A_lower; // C_A各行的下界，5*n_c×1。
  ContactConstraintVector c_A_upper; // C_A各行的上界，无穷表示不限制。

  int n_c = 0;                    // 当前支撑足数量。
  std::array<int, 4> stance_legs{{-1, -1, -1, -1}}; // 只有前n_c个编号有效。
};


struct RelaxationOutput {
  bool success = false;
  WbcMessage message;             // 固定容量提示，返回结果时不分配字符串。

  Vec6 delta_qddot = Vec6::Zero(); // 论文式(4.43)：基座加速度修正量。
  Feet delta_f_O = Feet::Zero();  // 足底力修正，解包回四足列顺序。
  Vec18 qddot = Vec18::Zero();    // 仅前6维被修正，后12维保持WBC输出。
  Feet f_c_O = Feet::Zero();      // 修正后的世界系反力，摆动足为零。
  Vec12 tau_j = Vec12::Zero();    // 论文式(4.61)：逆动力学求出的实体关节力矩。
  Vec12 tau_cmd_j = Vec12::Zero(); // 控制接口：tau_j加外部PD后的总力矩。

  // 下列量只用于检查结果，不参与优化；success表示求解完成，仍应检查残差。
  Vec6 floating_base_dynamics_residual = Vec6::Zero();
  Feet stance_acceleration_residual_O = Feet::Zero(); // 诊断量，不要求松弛后仍为零。
  double equality_residual_inf = 0.0;
  double minimum_inequality_margin = std::numeric_limits<double>::infinity();
  double objective = 0.0;
};


class MpcWbcRelaxation {
 public:
  explicit MpcWbcRelaxation(RelaxationSettings settings = {});

  const RelaxationSettings& settings() const noexcept { return settings_; }
  // 仅控制线程调用；在线调参后台已校验两项权重为有限正数。
  void updateCostWeights(double q_wbc, double q_mpc) noexcept {
    settings_.Q_WBC = q_wbc;
    settings_.Q_MPC = q_mpc;
  }

  // 按论文顺序构造QP，复用内部固定容量缓存；下次调用会覆盖返回的内容。
  // 同一实例仅供一个控制线程使用，正常构造路径不向堆申请Eigen内存。
  const RelaxationQp& buildQp(const RelaxationInput& input) const;

  // 外部填一帧RelaxationInput后直接调用；失败时返回清零结果及原因。
  // 现有QuadProg++的适配容器和内部求解工作区仍会动态分配，未修改已有库。
  RelaxationOutput solve(const RelaxationInput& input) const;

 private:
  void validate(const RelaxationInput& input) const;

  RelaxationSettings settings_;
  mutable RelaxationQp qp_;
};

}  // namespace sysu219::wbc
