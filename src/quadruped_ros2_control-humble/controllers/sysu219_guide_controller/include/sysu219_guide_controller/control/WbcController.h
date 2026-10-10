#pragma once

#include <Eigen/Dense>
#include <array>
#include <limits>
#include <string>
#include <string_view>

namespace sysu219::wbc {

// 主依据：于宪元，北航《基于稳定性的仿生四足机器人控制系统设计》，第四章。
// 先按任务优先级求关节参考和广义加速度，再交给独立的松弛优化模块。
// StateTrotting在calcQQd调用本模块，run随后调用松弛优化；机器人运行效果仍需调试验证。
// 四足顺序为 FR、FL、RR、RL；广义量依次为机身转动3维、平动3维、关节12维。
// 基座速度用 B 系，任务和接触力用世界 G 系；R_GB 将 B 系向量旋转到 G 系。
// 动力学、雅可比和加速度偏置必须使用同一原点、列顺序与加速度约定。
// 命名沿用笔记：O是世界系，与原注释中的G系相同；原R_GB现命名为R_OB。
// dot/ddot表示速度/加速度，d表示期望值，delta表示增量或修正量，pinv表示伪逆。
// qdot、qddot采用笔记中的广义量定义，前三维不是欧拉角的一阶、二阶导数。
using Vec3 = Eigen::Vector3d;                       // 单个三维量，如位置或角速度。
using Vec6 = Eigen::Matrix<double, 6, 1>;          // 浮动基座的六个分量。
using Vec12 = Eigen::Matrix<double, 12, 1>;        // 四腿共12个关节分量。
using Vec18 = Eigen::Matrix<double, 18, 1>;        // 基座6维和关节12维的广义量。
using Mat18 = Eigen::Matrix<double, 18, 18>;       // 完整浮动基质量矩阵的尺寸。
using FootJacobian = Eigen::Matrix<double, 3, 18>; // 单足雅可比：输出xyz，输入18维广义量。
using Feet = Eigen::Matrix<double, 3, 4>;          // 每列是一只脚，三行依次是xyz。
using Contact = std::array<int, 4>;               // 四足当前接触状态。
constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();  // kUnset是double类型的常量NaN，表示未设置或无效值。

// 实时存储：尺寸可以变化，但数据空间随对象一次性预留，不向堆申请内存。
using WbcTaskMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                   Eigen::ColMajor, 12, 18>;
using WbcTaskVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 12, 1>;
using WbcWorkMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                   Eigen::ColMajor, 18, 18>;
using WbcWorkVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 18, 1>;

// 提示文本放在固定缓冲区中，成功和失败返回都不需要分配字符串内存。
class WbcMessage {
 public:
  WbcMessage& operator=(std::string_view text) noexcept {
    size_ = text.size() < storage_.size() - 1 ? text.size() : storage_.size() - 1;
    if (size_ != 0) std::char_traits<char>::copy(storage_.data(), text.data(), size_);
    storage_[size_] = '\0';
    return *this;
  }
  const char* c_str() const noexcept { return storage_.data(); }
  const char* data() const noexcept { return storage_.data(); }
  std::size_t size() const noexcept { return size_; }
  bool empty() const noexcept { return size_ == 0; }
  operator std::string_view() const noexcept { return {storage_.data(), size_}; }

 private:
  std::array<char, 192> storage_{};
  std::size_t size_ = 0;
};

// SVD以及伪逆乘法都复用这些有容量上限的缓冲区。
struct WbcSvdWorkspace {
  Eigen::JacobiSVD<WbcWorkMatrix> svd{18, 18, Eigen::ComputeThinU | Eigen::ComputeThinV};
  WbcWorkVector sigma_inv{18};
  WbcWorkMatrix V_sigma_inv{18, 18};
  WbcWorkMatrix J_pinv{18, 18};
};

// 必填模型量默认 NaN，漏填时明确报错；现有单腿 KDL 接口不能直接代替浮动基模型。
struct WbcInput {
  Eigen::Matrix3d R_OB = Eigen::Matrix3d::Constant(kUnset);  // 当前机身到世界系的旋转。
  Vec3 Theta = Vec3::Constant(kUnset);  // 当前横滚、俯仰、偏航，单位rad，与R_GB一致。
  Vec3 Theta_d = Vec3::Constant(kUnset);  // 期望机身欧拉角，采用相同ZYX旋转约定。
  // 当前接入以URDF的base_name为基座原点，与估计器、M和J一致。
  // p_com保留原命名，但此处表示基座原点的位置，不是随腿运动变化的整机质心。
  Vec3 p_com_O = Vec3::Constant(kUnset);  // 当前世界系位置，单位m。
  Vec3 p_com_d_O = Vec3::Constant(kUnset);  // 期望世界系位置。
  Vec3 pdot_com_d_O = Vec3::Zero();  // 期望世界系线速度，单位m/s。
  Vec3 omega_d_O = Vec3::Zero();  // 期望世界系角速度，单位rad/s。
  // 保留论文中的加速度前馈入口；论文未规划加速度，因此默认零。
  Vec3 pddot_com_d_O = Vec3::Zero();  // 期望世界系线加速度，单位m/s²。
  Vec3 alpha_d_O = Vec3::Zero();  // 期望世界系角加速度，单位rad/s²。
  Vec12 q_j = Vec12::Constant(kUnset);  // 当前12个关节角，单位rad。
  // 当前实测速度：先机身B系角速度，再机身B系线速度，最后12个关节速度。
  Vec18 qdot = Vec18::Constant(kUnset);
  Contact contact{{1, 1, 1, 1}};  // 1支撑、0摆动，与松弛模块使用同一时刻的接触状态。
  Feet x_f_O = Feet::Constant(kUnset);  // 当前各足世界系位置，单位m。
  Feet x_f_d_O = Feet::Constant(kUnset);  // 摆动足期望位置，支撑足不使用。
  Feet xdot_f_d_O = Feet::Zero();  // 摆动足期望世界系线速度。
  Feet xddot_f_d_O = Feet::Zero();  // 摆动足加速度前馈，未规划时保持零。
  
  // 四足各自的完整雅可比，映射广义速度到该足世界系线速度。
  // J_F_O，J是雅可比，F是足端，O是世界系，J_F_O是足端在世界系的雅可比矩阵
  std::array<FootJacobian, 4> J_f_O{{
      FootJacobian::Constant(kUnset), FootJacobian::Constant(kUnset),
      FootJacobian::Constant(kUnset), FootJacobian::Constant(kUnset)}};
  // 足端雅可比未覆盖的加速度项，包含运动导致的速度变化。
  // 用空间加速度递推计算时，需补上角速度与线速度的叉乘，再转到世界系。
  Feet Jdot_f_qdot_O = Feet::Constant(kUnset);
  // 保留中文论文的零基座偏置默认，接模型时仍需核对后端约定。
  // 若基座线加速度表示局部速度的导数，线性偏置需填机身角/线速度叉乘的世界系值。
  Vec3 Jdot_orientation_qdot_O = Vec3::Zero();  // 转动任务的加速度偏置。
  Vec3 Jdot_position_qdot_O = Vec3::Zero();   // 平动任务的加速度偏置。
};

struct WbcSettings {
  // 接入前按本机器人设置增益；默认零时仍计算位置/速度参考，但不做加速度PD纠偏。
  Vec3 kp_orientation = Vec3::Zero();    // 转动任务的位置纠偏增益，三个世界系方向。
  Vec3 kd_orientation = Vec3::Zero();    // 转动任务的角速度阻尼增益。
  Vec3 kp_body_position = Vec3::Zero();  // 平动任务的位置纠偏增益，依次xyz。
  Vec3 kd_body_position = Vec3::Zero();  // 平动任务的线速度阻尼增益。
  Vec3 kp_swing = Vec3::Zero();          // 摆动足位置纠偏增益，四腿共用。
  Vec3 kd_swing = Vec3::Zero();          // 摆动足线速度阻尼增益，四腿共用。
  // 截断微小奇异值，避免放大噪声；阈值需按模型尺度校核。
  double svd_absolute_tolerance = 1e-10;  // 不随矩阵大小改变的奇异值下限。
  double svd_relative_tolerance = 1e-9;   // 相对于最大奇异值的舍弃比例。小于等于最大奇异值十亿分之一的奇异值，被舍弃。
};

struct WbcTask {
  // 优先级从高到低：支撑足、机身转动、机身平动、摆动足。
  // 足端任务的行数随足数量变化，因此这里使用动态尺寸矩阵和向量。
  std::string_view name;          // 任务名，用于辨认四个任务。
  WbcTaskMatrix J;                // 18列，描述广义运动对任务的作用。
  WbcTaskVector e;                // 位置级目标误差；支撑任务保持零。
  WbcTaskVector xdot_d;           // 期望任务速度；支撑任务保持零。
  WbcTaskVector xddot_cmd;        // 前馈和任务PD生成的加速度目标。
  // Jdot_qdot是雅可比导数乘当前实测广义速度的结果，存的是向量，不是矩阵。
  WbcTaskVector Jdot_qdot;        // 运动产生的偏置，求解时从目标中扣除。
};

struct WbcTaskDiagnostics {
  int projected_rank = 0;               // 高级任务限制后，本任务还能实现的独立方向数量。
  double position_residual = 0.0;       // 最终位置级目标的剩余误差大小。
  double velocity_residual = 0.0;       // 最终任务速度与期望速度的偏差大小。
  double acceleration_residual = 0.0;   // 最终任务加速度与目标的偏差大小，已计入偏置。
};

struct WbcOutput {
  bool success = false;  // 计算是否完成；不代表所有任务均可达。
  WbcMessage message;  // 成功说明或失败的具体原因。
  // 局部位置增量：仅后12维用于更新关节角，前三维不能直接加到欧拉角。
  Vec18 delta_q = Vec18::Zero();
  Vec18 qdot_d = Vec18::Zero();  // 四级递推后的广义速度参考。
  Vec18 qddot_d = Vec18::Zero();  // 交给松弛优化的广义加速度。
  Vec12 q_j_d = Vec12::Zero();  // 当前关节角加局部增量，供关节位置控制。
  Vec12 qdot_j_d = Vec12::Zero();  // 广义速度参考的关节部分。
  std::array<WbcTaskDiagnostics, 4> tasks{};  // 按四级优先级顺序保存诊断信息。
};

class WbcController {
 public:
  explicit WbcController(WbcSettings settings = {});  // 保存并检查配置；默认使用零任务增益。
  // 构造四级任务；没有支撑足或摆动足时，对应任务为空。
  // 返回内部任务缓存的只读引用；下一次调用会覆盖内容，同一实例仅供一个控制线程使用。
  const std::array<WbcTask, 4>& buildTasks(const WbcInput& input) const;
  // 分别递推位置增量、速度和加速度；失败时返回原因，不产生控制接口写入。
  // 正常求解路径无需堆分配；无效输入保留原异常机制，不保证异常路径无分配。
  WbcOutput solve(const WbcInput& input) const;

  // 外部把本周期的姿态、状态、轨迹、雅可比和偏置一次传入，复制到内部输入缓存。
  // 这里只赋值；数据检查和任务计算仍由原来的solve完成。
  void updateInput(const WbcInput& input);
  // 使用刚更新的输入计算，并通过WbcOutput返回结果。
  WbcOutput solve() const;

 private:
  WbcSettings settings_;                        // 本控制器使用的任务增益和伪逆阈值。
  void validate(const WbcInput& input) const;   // 输入检查，数据无效时抛出异常。

  // 每个函数只清空并填写自己的任务缓存；优先级由buildTasks中的调用顺序决定。
  void buildStanceTask(const WbcInput& input, WbcTask& task) const;       // 支撑足任务。
  void buildOrientationTask(const WbcInput& input, WbcTask& task) const;  // 机身转动任务。
  void buildBodyPositionTask(const WbcInput& input, WbcTask& task) const; // 机身平动任务。
  void buildSwingTask(const WbcInput& input, WbcTask& task) const;        // 摆动足任务。

  // mutable仅用于更新计算缓存，控制参数仍保持只读。
  mutable std::array<WbcTask, 4> tasks_;
  struct Workspace {
    WbcWorkMatrix J_A{0, 18};             // 已处理任务的堆叠雅可比。
    Mat18 N_A = Mat18::Identity();        // 已处理任务的共同零空间投影。
    WbcWorkMatrix J_i_N_A{0, 18};         // 当前任务雅可比乘高级任务零空间投影。
    WbcWorkMatrix J_i_N_A_pinv{18, 0};    // 上一项的伪逆。
    WbcSvdWorkspace svd;
  };
  mutable Workspace workspace_;

  WbcInput input_;  // 保存外部传入的一整帧数据；未更新时保留默认NaN。

};

}  // namespace sysu219::wbc
