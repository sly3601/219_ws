#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace sysu219::tuning {

// 已接入 StateTrotting / Sysu219GuideController，随控制器激活自动打开窗口。
// Linux；窗口用独立 python3/Tk 进程，无 ROS 服务、参数服务器或配置文件读写。
//
// 接入约定：
// 1. 在算法开始运行前，先执行原有 MPC/WBC 的全部赋值，再把最终值按行复制进
//    Parameters；尤其必须包含 S 的总 fz 耦合项和整体倍率，不可只复制 diagonal。
// 2. 创建 OnlineTuning(initial)。窗口没有默认控制参数，也不会在打开时提交参数。
// 3. 控制周期开头调用 tryReadUpdate；成功时由唯一的控制线程应用整组值，再调用
//    finishUpdate(revision, true)。应用前若拒绝，传 false（不能留下部分应用）。
//    MPC 还须同步刷新 Q_cost_/R_cost_/R_cost_rate_，不能只改 Q/R/S；WBC 须更新
//    已有实例的任务增益。不得在 GUI/后台线程直接写入求解器或重建控制器。
// 4. 不调参时不调用任何参数写入接口。窗口关闭后沿用最后一次已应用值。
// 5. 停用时仅 requestStop（原子写），后台回收子进程。对象只在算法停用后析构。
//
// tryReadUpdate/finishUpdate 只允许同一个控制线程调用。它们无互斥锁、等待、
// 重试循环、堆分配、文件/网络操作；无更新时只有一次原子读取。
// 新增请求在下一次调用 tryReadUpdate 时可用；StateTrotting 在 MPC 权重变化时
// 令该周期重新求解。此处保证交互路径不阻塞控制线程，实际耗时仍需运行验证。
struct Parameters {
  static constexpr std::size_t kQSize = 13 * 13;
  static constexpr std::size_t kForceSize = 12 * 12;
  static constexpr std::size_t kWbcOffset = kQSize + 2 * kForceSize;
  static constexpr std::size_t kRelaxationOffset = kWbcOffset + 21;
  static constexpr std::size_t kMotorOffset = kRelaxationOffset + 3;
  static constexpr std::size_t kCount = kMotorOffset + 4;

  // 行优先：Q[0..168]、R[169..312]、S[313..456]，完整矩阵，含非对角项。
  // WBC[457..477]：kp_orientation.xyz、kd_orientation.xyz、
  // kp_body_position.xyz、kd_body_position.xyz、kp_swing.xyz、kd_swing.xyz、
  // swing_hip_posture_weight、kp_swing_hip_posture、kd_swing_hip_posture。
  // Relaxation[478..480]：Q_WBC、Q_MPC、joint_acceleration_weight_ratio。
  // MotorMIT[481..484]：kp_stance、kd_stance、kp_swing、kd_swing。
  // 调用者必须提供全部实际值；不提供会在初始化时拒绝，禁止静默使用零默认值。
  std::array<double, kCount> values;
  Parameters() noexcept;
};

class OnlineTuning final {
 public:
  // 只在初始化、算法开始运行前构造。校验初值但绝不替换、归一化或写回初值。
  // 无 DISPLAY、python3/Tk 不可用时窗口退出；主算法仍继续使用原值。
  // 系统资源创建失败会抛异常，StateTrotting 初始化处捕获，不阻止原控制器启动。
  explicit OnlineTuning(const Parameters& initial);
  ~OnlineTuning();
  OnlineTuning(const OnlineTuning&) = delete;
  OnlineTuning& operator=(const OnlineTuning&) = delete;

  // 成功时复制一组固定长度的参数；直到 finishUpdate 前不会接受下一次修改。
  [[nodiscard]] bool tryReadUpdate(Parameters& output, std::uint64_t& revision) noexcept;
  // applied=true 仅用于整组参数已写入算法之后。快照/窗口按此确认值更新。
  void finishUpdate(std::uint64_t revision, bool applied) noexcept;
  // 停用回调只发停止信号，不 join、不等待窗口进程退出。
  void requestStop() noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sysu219::tuning
