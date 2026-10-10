#include "sysu219_guide_controller/control/WbcController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

// 具名命名空间：WBC的数据类型和控制器都属于sysu219::wbc。
namespace sysu219::wbc {
namespace { // 匿名命名空间。里面的辅助函数只在当前 .cpp 文件中可用

// 统一检查条件。条件不满足就抛异常，message说明是哪项数据有问题。
// 构造时的配置错误会直接抛出；求解时的输入错误由solve捕获并返回失败。
void require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

// 求矩阵伪逆：把任务空间的误差转换为广义空间的运动修正。
// 支持非方阵和秩亏矩阵；settings控制哪些过小的奇异值应舍弃。
// rank是可选输出指针，记录保留下来的独立运动方向数量。
const WbcWorkMatrix& pseudoInverse(const WbcWorkMatrix& matrix,
                                  const WbcSettings& settings,
                                  WbcSvdWorkspace& workspace, int* rank = nullptr) {
  // 调用者传入rank地址时，先清零；没有传入时不访问它。
  if (rank) *rank = 0;
  // 某类足数量为零时，任务矩阵没有行；返回对应尺寸的空伪逆即可。
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    workspace.J_pinv.setZero(matrix.cols(), matrix.rows());
    return workspace.J_pinv;
  }
  require(matrix.allFinite(), "WBC伪逆输入含无效数值");
  // SVD将矩阵分解为独立方向及各方向的强弱，Thin只保留构造伪逆所需的部分。
  // 实时优化：复用SVD对象，尺寸变化只改变有效范围，所有存储都有固定容量。
  auto& svd = workspace.svd;
  svd.compute(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  require(svd.info() == Eigen::Success, "WBC奇异值分解失败");
  // 奇异值由大到小排列。这里先复制，随后在同一个向量中存它们的倒数。
  auto& sigma_inv = workspace.sigma_inv;
  sigma_inv = svd.singularValues();
  // 绝对阈值排除接近零的方向，相对阈值随最大奇异值调整；取更严格的那个。
  // 这样不会重新放大已被高级任务消除的运动方向。
  const double threshold = std::max(settings.svd_absolute_tolerance,
      settings.svd_relative_tolerance * sigma_inv(0));
  for (Eigen::Index i = 0; i < sigma_inv.size(); ++i) {
    if (sigma_inv(i) > threshold) {
      // 有效方向取倒数，用于把任务误差反解成运动量。
      sigma_inv(i) = 1.0 / sigma_inv(i);
      if (rank) ++*rank;
    } else {
      // 病态或不可用的方向置零，避免除以极小值生成巨大指令。
      sigma_inv(i) = 0.0;
    }
  }
  // asDiagonal把倒数向量放到对角线上；再利用SVD的方向矩阵重建伪逆。
  // 两次乘法分开写入预留空间，避免动态尺寸的临时乘积。
  workspace.V_sigma_inv.noalias() = svd.matrixV() * sigma_inv.asDiagonal();
  workspace.J_pinv.noalias() = workspace.V_sigma_inv * svd.matrixU().transpose();
  return workspace.J_pinv;
}

// 每周期清空并填写有效行，容量不变；没有接触的任务仍保留零行语义。
void resetTask(WbcTask& task, int rows) {
  task.J.setZero(rows, 18);
  task.e.setZero(rows);
  task.xdot_d.setZero(rows);
  task.xddot_cmd.setZero(rows);
  task.Jdot_qdot.setZero(rows);
}

// 创建一个指定行数的任务，所有目标和偏置先清零，后续按任务类型填充。
// 每个任务的雅可比都保留18列，分别对应机身6维和关节12维。
WbcTask emptyTask(const char* name, int rows) 
{
  WbcTask task;
  task.name = name;
  task.J = WbcTaskMatrix::Zero(rows, 18);            // 雅可比矩阵，18列描述广义运动对任务的作用
  task.e = WbcTaskVector::Zero(rows);          // 位置级目标误差，支撑任务保持零
  task.xdot_d = WbcTaskVector::Zero(rows);      // 期望速度，支撑任务保持零
  task.xddot_cmd = WbcTaskVector::Zero(rows);    // 目标加速度
  task.Jdot_qdot = WbcTaskVector::Zero(rows);       // 运动产生的偏置，求解时从目标中扣除
  return task;
}

}  // namespace

// 构造函数：保存控制参数，并立即拒绝无效增益或SVD阈值。
// 冒号后是成员初始化；std::move允许用移动构造将参数交给settings_保存。
WbcController::WbcController(WbcSettings settings) : settings_(std::move(settings)) 
{

  // 花括号里列出六个增益向量的地址，循环逐个检查数值是否合法。
  for (const Vec3* gain : {&settings_.kp_orientation, &settings_.kd_orientation,
                          &settings_.kp_body_position, &settings_.kd_body_position,
                          &settings_.kp_swing, &settings_.kd_swing}) 
  {
    // allFinite检查三个分量均不是NaN或无穷大。
    // array允许逐元素比较，末尾all要求三个分量都非负；零增益是合法配置。
    require(gain->allFinite() && (gain->array() >= 0.0).all(),    // require函数检查条件是否满足，如果不满足就抛出异常
            "WBC增益必须有限且非负");
  }
  // 两个阈值都必须有限且为正，相对阈值还必须小于1。
  // 否则可能放大接近零的奇异值，或把所有有效方向都舍弃。
  require(std::isfinite(settings_.svd_absolute_tolerance) &&
              settings_.svd_absolute_tolerance > 0.0 &&
              std::isfinite(settings_.svd_relative_tolerance) &&
              settings_.svd_relative_tolerance > 0.0 &&
              settings_.svd_relative_tolerance < 1.0,
          "WBC奇异值阈值无效");


  constexpr int stance_count = 4; // 构造时无法知道支撑足数量，先按最大值分配空间，后续buildTasks会根据实际接触状态调整任务行数。
  // 新建4个任务
  tasks_ =
  {
    {
      emptyTask("stance", 3 * stance_count),          // 支撑足任务，行数随支撑足数量变化
      emptyTask("orientation", 3),                    // 机身转动任务，固定3行    
      emptyTask("body_position", 3),                  // 机身平动任务，固定3行      
      emptyTask("swing", 3 * (4 - stance_count))      // 摆动足任务，行数随摆动足数量变化
    }
  };
}



// 一个结构体包含本周期的全部输入，Eigen矩阵和四足数组也会一起复制。
// J_f_O等模型量由外部提供，这里保留原算法的数据约定，不另建机器人模型。
void WbcController::updateInput(const WbcInput& input)
{
  // 机身当前姿态与期望姿态。
  input_.R_OB = input.R_OB;          // 当前机身到世界系的旋转矩阵。
  input_.Theta = input.Theta;        // 当前横滚、俯仰、偏航。
  input_.Theta_d = input.Theta_d;    // 期望横滚、俯仰、偏航。

  // 机身位置、速度目标和加速度前馈。
  input_.p_com_O = input.p_com_O;                    // 当前世界系位置。
  input_.p_com_d_O = input.p_com_d_O;                // 期望世界系位置。
  input_.pdot_com_d_O = input.pdot_com_d_O;          // 期望世界系线速度。
  input_.omega_d_O = input.omega_d_O;                // 期望世界系角速度。
  input_.pddot_com_d_O = input.pddot_com_d_O;        // 期望世界系线加速度。
  input_.alpha_d_O = input.alpha_d_O;                // 期望世界系角加速度。

  // 关节状态、广义速度和四足接触状态。
  input_.q_j = input.q_j;            // 当前12个关节角。
  input_.qdot = input.qdot;          // 机身角速度、机身线速度、12个关节速度。
  input_.contact = input.contact;    // 四足接触状态：1支撑，0摆动。

  // 足端状态与摆动轨迹；每个矩阵的四列依次对应FR、FL、RR、RL。
  input_.x_f_O = input.x_f_O;                // 当前足端世界系位置。
  input_.x_f_d_O = input.x_f_d_O;            // 期望足端世界系位置。
  input_.xdot_f_d_O = input.xdot_f_d_O;      // 期望足端世界系速度。
  input_.xddot_f_d_O = input.xddot_f_d_O;    // 期望足端世界系加速度。

  // 外部模型提供四足雅可比，每个矩阵为3行18列。
  input_.J_f_O[0] = input.J_f_O[0];    // FR：右前足。
  input_.J_f_O[1] = input.J_f_O[1];    // FL：左前足。
  input_.J_f_O[2] = input.J_f_O[2];    // RR：右后足。
  input_.J_f_O[3] = input.J_f_O[3];    // RL：左后足。

  // 外部模型提供加速度偏置，原求解算法会从任务目标中扣除。
  input_.Jdot_f_qdot_O = input.Jdot_f_qdot_O;                        // 四足偏置。
  input_.Jdot_orientation_qdot_O = input.Jdot_orientation_qdot_O;    // 机身转动偏置。
  input_.Jdot_position_qdot_O = input.Jdot_position_qdot_O;          // 机身平动偏置。
}

WbcOutput WbcController::solve() const
{
  return solve(input_);
}

// 求解前检查输入。这里只检查数据和坐标约定，不检查轨迹是否可达。
// const表示检查过程不修改控制器内部配置，in也以只读引用传入。
void WbcController::validate(const WbcInput& in) const 
{
  // 旋转矩阵必须有限、正交，且行列式接近1，排除缩放和镜像变换。
  // norm衡量矩阵整体误差，比较时留出浮点计算的容差。
  require(in.R_OB.allFinite() &&                                                              // 检查矩阵的 9 个数都正常，不能出现 NaN 或 inf
              (in.R_OB.transpose() * in.R_OB - Eigen::Matrix3d::Identity()).norm() < 1e-7 &&  // R.transpose() * R ≈ I
              std::abs(in.R_OB.determinant() - 1.0) < 1e-7,                                   // det(R) ≈ 1
          "WBC姿态矩阵不是有效旋转");

  require(in.Theta.allFinite() && in.Theta_d.allFinite(), "WBC当前或期望欧拉角无效");

  // 按ZYX顺序从当前欧拉角重建姿态；AngleAxisd分别表示绕指定轴旋转。
  // 再与输入旋转矩阵比较，防止欧拉角和矩阵来自不同姿态或采用不同旋转顺序。
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(in.Theta(2), Vec3::UnitZ()) *
       Eigen::AngleAxisd(in.Theta(1), Vec3::UnitY()) *
       Eigen::AngleAxisd(in.Theta(0), Vec3::UnitX())).toRotationMatrix();
  require((rotation - in.R_OB).norm() < 1e-7, "WBC欧拉角与旋转矩阵不一致");

  // 机身状态、轨迹、基座偏置和关节状态均须已填写，不能保留默认NaN。
  require(in.p_com_O.allFinite() && in.p_com_d_O.allFinite() &&
              in.pdot_com_d_O.allFinite() && in.omega_d_O.allFinite() &&
              in.pddot_com_d_O.allFinite() && in.alpha_d_O.allFinite() &&
              in.Jdot_orientation_qdot_O.allFinite() && in.Jdot_position_qdot_O.allFinite() &&
              in.q_j.allFinite() && in.qdot.allFinite(),
          "WBC状态、目标或基座偏置含无效数值");

  for (int leg = 0; leg < 4; ++leg) {
    // 每条腿只能是支撑或摆动；两类任务都会用到该腿的雅可比和偏置。
    require(in.contact[leg] == 0 || in.contact[leg] == 1, "足端接触状态只能为0或1");
    require(in.J_f_O[leg].allFinite() &&
                in.Jdot_f_qdot_O.col(leg).allFinite(), "足端雅可比或加速度偏置无效");
    // 支撑脚没有一个位置目标，所以中文论文的支撑足任务不加位置PD，因此只检查摆动足轨迹。
    if (in.contact[leg] == 0) 
    {
      require(in.x_f_O.col(leg).allFinite() &&
                  in.x_f_d_O.col(leg).allFinite() &&
                  in.xdot_f_d_O.col(leg).allFinite() &&
                  in.xddot_f_d_O.col(leg).allFinite(),
              "摆动足状态或轨迹无效");
    }
  }
}



const std::array<WbcTask, 4>& WbcController::buildTasks(const WbcInput& in) const 
{
  // 先检查输入，再根据本周期接触状态决定两个足端任务的大小。
  validate(in);
  // 下标0到3固定对应四级优先级。每足提供xyz三个任务分量，机身任务各三维。
  // 实时优化：初始化已移到构造函数，这里只清空缓存并更新各任务的有效尺寸。
  auto& tasks = tasks_;
  // 四个任务独立构造；这里只组织优先级，不混合各任务的计算。
  buildStanceTask(in, tasks[0]);
  buildOrientationTask(in, tasks[1]);
  buildBodyPositionTask(in, tasks[2]);
  buildSwingTask(in, tasks[3]);
  return tasks;
}

void WbcController::buildStanceTask(const WbcInput& in, WbcTask& task) const
{
  int stance_count = 0;                                                   // 统计支撑足数量，决定支撑任务的行数。
  for (int contact : in.contact) stance_count += contact;                 // 统计支撑足数量
  resetTask(task, 3 * stance_count);

  int stance_row = 0;  // 下一条支撑足数据写入的起始行。
  for (int leg = 0; leg < 4; ++leg) 
  {
    // 这里只处理支撑足，摆动足由buildSwingTask单独处理。
    if (in.contact[leg] == 0)
    {
      continue;
    }
    // 把各只支撑脚的数据拼起来。每条支撑足占连续三行，按FR、FL、RR、RL顺序写入自己的任务。
    task.J.middleRows(stance_row, 3) = in.J_f_O[leg];                   // J_F_O，J是雅可比，F是足端，O是世界系，J_F_O是足端在世界系的雅可比矩阵
    task.Jdot_qdot.segment(stance_row, 3) = in.Jdot_f_qdot_O.col(leg);  // Jdot_f_qdot_O，足端雅可比未覆盖的加速度项，包含运动导致的速度变化。
    // 支撑足保持不动：不加位置弹簧，速度目标为零，加速度抵消运动偏置。
    // resetTask已把位置误差、速度目标和加速度目标清零，这里无需额外赋值。
    stance_row += 3;
  }
}

// 第二优先级：机身转动任务
void WbcController::buildOrientationTask(const WbcInput& in, WbcTask& orientation) const
{
  resetTask(orientation, 3);
  // 第二优先级：机身转动。leftCols只填前三列，其他自由度的列保持零。
  // 当前广义角速度在B系，旋转后才能与世界系的角速度目标比较。
  orientation.J.leftCols(3) = in.R_OB;      // R_OB是当前机身到世界系的旋转矩阵，左乘B系角速度得到世界系角速度
  Vec3 e_Theta = in.Theta_d - in.Theta;     // 欧拉角误差，单位rad。
  // 取最近角差，避免跨越正负pi时绕远路。
  for (int i = 0; i < 3; ++i)
  {
    e_Theta(i) = std::atan2(std::sin(e_Theta(i)), std::cos(e_Theta(i)));
  }
  const double theta = in.Theta(1);  // 当前俯仰角，单位rad。
  const double psi   = in.Theta(2);  // 当前偏航角，单位rad。
  // 欧拉角误差转成世界系的小转动误差；大姿态误差或接近俯仰奇异点时需另行处理。
  Eigen::Matrix3d T_Theta_to_omega_O;
  T_Theta_to_omega_O << std::cos(theta) * std::cos(psi), -std::sin(psi), 0.0,
                    std::cos(theta) * std::sin(psi),  std::cos(psi), 0.0,
                    -std::sin(theta), 0.0, 1.0;
  // 此处生成位置级递推使用的小转动目标，不是把欧拉角误差当成角速度。
  orientation.e = T_Theta_to_omega_O * e_Theta;
  orientation.xdot_d = in.omega_d_O;
  // 偏置留给加速度递推扣除，避免将其与轨迹前馈混在一起。
  orientation.Jdot_qdot = in.Jdot_orientation_qdot_O;
  // head取广义速度的前三项，再转到世界系；用期望与实际角速度差做阻尼反馈。
  orientation.xddot_cmd = in.alpha_d_O +
      settings_.kp_orientation.cwiseProduct(orientation.e) +
      settings_.kd_orientation.cwiseProduct(
          in.omega_d_O - in.R_OB * in.qdot.head<3>());
}

void WbcController::buildBodyPositionTask(const WbcInput& in, WbcTask& translation) const
{
  resetTask(translation, 3);
  // 第三优先级：机身平动，跟踪输入约定的基座原点世界系位置。
  // block从第0行、第3列写入三行三列旋转矩阵，只关联基座平动自由度。
  translation.J.block<3, 3>(0, 3) = in.R_OB;
  translation.e = in.p_com_d_O - in.p_com_O;
  translation.xdot_d = in.pdot_com_d_O;
  translation.Jdot_qdot = in.Jdot_position_qdot_O;
  // segment从第3项取三个基座线速度分量；转到世界系后与期望线速度比较。
  translation.xddot_cmd = in.pddot_com_d_O +
      settings_.kp_body_position.cwiseProduct(translation.e) +
      settings_.kd_body_position.cwiseProduct(
          in.pdot_com_d_O - in.R_OB * in.qdot.segment<3>(3));
}

void WbcController::buildSwingTask(const WbcInput& in, WbcTask& task) const
{
  // 摆动足任务自己统计有效足数，不依赖支撑足任务的计数或写入位置。
  int swing_count = 0;
  for (int contact : in.contact)
  {
    if (contact == 0) ++swing_count;
  }
  resetTask(task, 3 * swing_count);

  int swing_row = 0;   // 下一条摆动足数据写入的起始行。
  for (int leg = 0; leg < 4; ++leg)
  {
    if (in.contact[leg] == 1)
    {
      continue;
    }
    // 每条摆动足占连续三行，按FR、FL、RR、RL顺序写入自己的任务。
    task.J.middleRows(swing_row, 3) = in.J_f_O[leg];
    task.Jdot_qdot.segment(swing_row, 3) = in.Jdot_f_qdot_O.col(leg);
    // 摆动足跟踪轨迹，前馈加速度与位置/速度PD共同生成加速度目标。
    task.e.segment(swing_row, 3) =
        in.x_f_d_O.col(leg) - in.x_f_O.col(leg);
    task.xdot_d.segment(swing_row, 3) = in.xdot_f_d_O.col(leg);
    // 用实际广义速度求当前足端世界系速度，作为速度PD的反馈。
    const Vec3 xdot_f_O = in.J_f_O[leg] * in.qdot;
    // cwiseProduct是逐分量相乘，给xyz三个方向分别施加增益。
    // 三项依次是规划加速度前馈、位置误差纠偏和速度误差纠偏。
    task.xddot_cmd.segment(swing_row, 3) =
        in.xddot_f_d_O.col(leg) +
        settings_.kp_swing.cwiseProduct(task.e.segment(swing_row, 3)) +
        settings_.kd_swing.cwiseProduct(in.xdot_f_d_O.col(leg) - xdot_f_O);
    swing_row += 3;  // 移到下一条摆动足的写入位置。
  }
}

// 完整求解入口：构造任务，按优先级递推，再提取关节参考并计算残差。
// 此处生成的是松弛优化之前的运动学结果，不负责求接触力或电机力矩。
WbcOutput WbcController::solve(const WbcInput& in) const 
{
  // 默认输出清零且成功标志为false；只有全部计算完成才标记成功。
  WbcOutput out;
  try 
  {
    // 四个任务。
    const auto& tasks = buildTasks(in);  // 先检查输入并构造四级任务，失败时抛异常。

    // 论文式(4.36)：任务1单独初始化，位置增量和速度指令均为零。
    out.delta_q.setZero();
    out.qdot_d.setZero();
    const auto& stance = tasks[0];
    const auto& J_1_pinv = pseudoInverse(stance.J, settings_, workspace_.svd,
                                       &out.tasks[0].projected_rank);
    // 支撑任务的目标为零，因此第一步就在尝试抵消足端偏置加速度。
    out.qddot_d.noalias() = J_1_pinv * (-stance.Jdot_qdot);

    // 只堆叠当前任务之前的高级任务，循环内先计算它们的共同零空间。
    auto& J_A = workspace_.J_A;
    J_A.resize(0, 18);
    auto& N_A = workspace_.N_A;

    // i沿用论文任务编号2、3、4；数组下标对应i-1。
    for (int i = 2; i <= 4; ++i) 
    {
      const auto& previous_task = tasks[i - 2];
      const Eigen::Index old_rows = J_A.rows();
      // 扩大行数时保留原数据，再把上一任务的雅可比追加到最后几行。
      J_A.conservativeResize(old_rows + previous_task.J.rows(), 18);
      J_A.bottomRows(previous_task.J.rows()) = previous_task.J;
      // 根据全部已处理任务重算零空间，使当前任务不会覆盖先前任何一级的实现值。
      N_A.noalias() = Mat18::Identity() -
          pseudoInverse(J_A, settings_, workspace_.svd) * J_A;

      const auto& task = tasks[i - 1];
      // 先把当前任务限制在高级任务的零空间内，保证它不会改变高级任务的结果。
      auto& J_i_N_A = workspace_.J_i_N_A;
      J_i_N_A.noalias() = task.J * N_A; // J_i * N_{i-1}^{A}
      // 对受限任务求伪逆，同时把还能完成多少独立方向写入对应任务的诊断信息。
      auto& J_i_N_A_pinv = workspace_.J_i_N_A_pinv;
      J_i_N_A_pinv = pseudoInverse(J_i_N_A, settings_, workspace_.svd, &out.tasks[i - 1].projected_rank);
      // 位置递推：先算已有增量已完成多少目标，再补足当前任务剩余的位置误差。
      out.delta_q += J_i_N_A_pinv *
          (task.e - task.J * out.delta_q);
      // 速度递推：从期望任务速度扣除已有广义速度参考的贡献，再增加剩余修正。
      out.qdot_d += J_i_N_A_pinv *
          (task.xdot_d - task.J * out.qdot_d);
      // 加速度递推：先扣除运动偏置和已有加速度的贡献，再反解需要增加的部分。
      out.qddot_d += J_i_N_A_pinv *
          (task.xddot_cmd - task.Jdot_qdot -
           task.J * out.qddot_d);
    }

    // 数值异常立即返回失败，防止将溢出或NaN当成正常参考值。
    require(out.delta_q.allFinite() && out.qdot_d.allFinite() &&
                out.qddot_d.allFinite(), "WBC递推结果含无效数值");
    // tail取最后12项实体关节：当前位置加本次增量得到位置参考，速度直接提取。
    // 增量是当前构型附近的逆解，不乘控制周期、不累积积分；也不更新基座欧拉角。
    out.q_j_d = in.q_j + out.delta_q.tail<12>();
    out.qdot_j_d = out.qdot_d.tail<12>();
    require(out.q_j_d.allFinite(), "WBC关节位置目标含无效数值");
    // 全部低优先级任务处理完后，再回头检查每一级最终实际实现的效果。
    for (int i = 0; i < 4; ++i) 
    {
      const auto& task = tasks[i];
      // 三种残差分别对应位置、速度、加速度；norm得到该任务整体误差的大小。
      // 加速度残差包含偏置，才能表示足点或机身真正的加速度目标偏差。
      out.tasks[i].position_residual =
          (task.J * out.delta_q - task.e).norm();
      out.tasks[i].velocity_residual =
          (task.J * out.qdot_d - task.xdot_d).norm();
      out.tasks[i].acceleration_residual = (task.J * out.qddot_d +
          task.Jdot_qdot - task.xddot_cmd).norm();
      require(std::isfinite(out.tasks[i].position_residual) &&
                  std::isfinite(out.tasks[i].velocity_residual) &&
                  std::isfinite(out.tasks[i].acceleration_residual), "WBC任务残差含无效数值");
    }
    // 成功仅表示计算完成；不可达或冲突的任务仍有残差，需要调用者检查。
    out.success = true;
    out.message = "WBC递推完成，任务残差可用于诊断";
  } catch (const std::exception& error) {
    // 失败时清空本次结果；是否执行保底由未来的控制流程决定。
    out = WbcOutput{};
    out.message = error.what();  // 将检查或计算失败的具体原因交给调用者。
  }
  return out;
}

}  // namespace sysu219::wbc
