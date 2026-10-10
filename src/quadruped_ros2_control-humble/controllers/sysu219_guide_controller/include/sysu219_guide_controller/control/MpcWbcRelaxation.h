#pragma once

#include "sysu219_guide_controller/control/WbcController.h"

namespace sysu219::wbc {

// 主依据：北航《基于稳定性的仿生四足机器人控制系统设计》第4.3节。
// 输入MPC足底力和WBC加速度，只修正基座加速度与支撑足力，再求实体关节力矩。
// 阅读顺序：输入检查 → 支撑足打包 → 目标函数 → 等式/不等式约束 → 求解 → 恢复输出。
// C是动力学偏置力，Q_WBC/Q_MPC是权重，G/C_E/c_e/C_I/c_i沿用论文的QP命名。
// O为世界系；腿顺序、18维广义量和加速度约定均与WbcInput保持一致。

// 最多4只支撑足：优化变量最多18维，足底力约束最多24条，可选力矩约束最多24条。
// 动态尺寸只改变有效范围，Eigen数据空间随对象预留，接触切换时不重新分配。
using RelaxationInequalityMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                                Eigen::ColMajor, 18, 48>;
using RelaxationInequalityVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 48, 1>;
using ContactConstraintMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                             Eigen::ColMajor, 20, 12>;
using ContactConstraintVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 20, 1>;


struct ContactForceLimits {
  // 用于生成论文中的单足C_i及其上下界；接入时与MPC使用相同参数。
  double mu = 0.35;        // 摩擦系数，四棱面线性近似。
  double f_z_min = 0.0;    // 接触面法向力下界，单位N。
  double f_z_max = 350.0;  // 法向力上界；卸载支撑足时由外部调整。

  // [英文补充] Bellicoso 2017第三节：可使用倾斜接触面。
  // 三列为世界系下的两条切向和法向；单位阵对应水平地面。
  Eigen::Matrix3d R_OC = Eigen::Matrix3d::Identity();
};


struct RelaxationSettings {
  // 论文式(4.44)：Q_WBC对应Q_1，惩罚基座加速度修正；Q_MPC对应Q_2，惩罚足底力修正。
  // 权重越大，对应修正越不容易发生；两种量的单位不同，不代表信任百分比。
  Eigen::Matrix<double, 6, 6> Q_WBC = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 12, 12> Q_MPC =
      0.005 * Eigen::Matrix<double, 12, 12>::Identity();

  // 实现所需的数值检查参数，不属于论文中的优化变量。
  double absolute_feasibility_tolerance = 1e-6;
  double relative_feasibility_tolerance = 1e-9;

  // [英文补充] Bellicoso 2017第三节：可选关节总力矩约束，默认关闭。
  bool enforce_torque_limits = false;
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

  // 仅用于检查松弛后支撑脚的加速度；不增加论文QP中的约束。
  Feet Jdot_f_qdot_O = Feet::Constant(kUnset);

  // [英文补充] 仅开启力矩约束时读取上下界，无穷界表示该方向不限制。
  Vec12 tau_min_j = Vec12::Constant(-std::numeric_limits<double>::infinity());
  Vec12 tau_max_j = Vec12::Constant(std::numeric_limits<double>::infinity());

  // [控制接口] 外部提供的关节PD力矩，不是第4.3节的松弛变量。
  // 使用电机内部PD时保持零，避免重复叠加；可选力矩约束检查叠加后的总力矩。
  Vec12 tau_PD_j = Vec12::Zero();
};


struct RelaxationQp {
  // 论文式(4.45)：X前6项是delta_qddot，其余是按支撑足排列的delta_f。
  // 论文式(4.58)：以下矩阵直接按QuadProg++的列约束格式存储。
  WbcWorkMatrix G;                 // 二次目标矩阵，n_X×n_X。
  WbcWorkMatrix C_E;               // 等式系数，n_X×6；每列是一条等式。
  Vec6 c_e = Vec6::Zero();         // 等式常量，放在等式左侧。
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
  Vec18 qddot = Vec18::Zero();    // 修正后的广义加速度，后12维仍取WBC结果。
  Feet f_c_O = Feet::Zero();      // 修正后的世界系反力，摆动足为零。
  Vec12 tau_j = Vec12::Zero();    // 论文式(4.61)：逆动力学求出的实体关节力矩。
  Vec12 tau_cmd_j = Vec12::Zero(); // 控制接口：tau_j加外部PD后的总力矩。

  // 下列量只用于检查结果，不参与优化；success表示求解完成，仍应检查残差。
  Vec6 floating_base_dynamics_residual = Vec6::Zero();
  Feet stance_acceleration_residual_O = Feet::Zero();
  double equality_residual_inf = 0.0;
  double minimum_inequality_margin = std::numeric_limits<double>::infinity();
  double objective = 0.0;
};


class MpcWbcRelaxation {
 public:
  explicit MpcWbcRelaxation(RelaxationSettings settings = {});

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
