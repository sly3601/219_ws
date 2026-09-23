//
// Created by tlab-uav on 24-9-11.
//

#include "controller_common/FSM/StateFixedDown.h"

#include <cmath>

StateFixedDown::StateFixedDown(CtrlInterfaces& ctrl_interfaces,
                               const std::vector<double>& target_pos,
                               const double kp,
                               const double kd)
    : FSMState(FSMStateName::FIXEDDOWN, "fixed down", ctrl_interfaces),
      kp_(kp), kd_(kd)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < 12; i++)
    {
        target_pos_[i] = target_pos[i];
    }
    // kp_start_/kd_start_ 是 kp_/kd_ 的百分比，恒为 1.0（起点即满刚度），
    // 只有 position 走 tanh 过渡。构造函数先给 1.0 兜底。
    kp_start_ = 1.0;
    kd_start_ = 1.0;
}

void StateFixedDown::enter()
{
    for (int i = 0; i < 12; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    }
    ctrl_interfaces_.control_inputs_.command = 0;

    // ========== 起点刚度固定为满值 ==========
    // 半趴只会从已处于满刚度的状态切来（FIXEDPRONE / FIXEDSTAND / BALANCETEST），
    // 不会从 PASSIVE 直接切来（PASSIVE 按 2 去 FIXEDPRONE），所以不需要软启动。
    kp_start_ = 1.0;
    kd_start_ = 1.0;

    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
        // kp_start_/kd_start_ 是"kp_/kd_ 的百分比"，这里要乘回 kp_/kd_ 写绝对值
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_ * kp_start_);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_ * kd_start_);
    }
    // ========== 新增：重置percent，确保从0开始 ==========
    percent_ = 0.0;
}

void StateFixedDown::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    // 用tanh做平滑过渡，范围[0, 1]
    phase = std::tanh(percent_);
    // 刚度按比例从 kp_start_ 升到 1.0：
    //   kp_start_=1.0 (STAND/SWING/...切来) → current_kp 始终等于 kp_，不缓冲。
    //   kp_start_=0.0 (PASSIVE 切来)         → current_kp 从 0 升到 kp_，软启动。
    double current_kp = kp_ * (kp_start_ + (1.0 - kp_start_) * phase);
    double current_kd = kd_ * (kd_start_ + (1.0 - kd_start_) * phase);
    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(current_kp);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(current_kd);
    }
}

void StateFixedDown::exit()
{
    percent_ = 0;
    // 离开半趴时清掉"待转全趴"标志，避免下次进半趴时残留误触发
    ctrl_interfaces_.go_prone_after_down = false;
}

FSMStateName StateFixedDown::checkChange()
{
    // 失能优先：1 键不受姿态过渡锁定期限制，随时可切
    if (ctrl_interfaces_.control_inputs_.command == 1)
    {
        return FSMStateName::PASSIVE;
    }
    if (percent_ < 1.5)
    {
        return FSMStateName::FIXEDDOWN;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 2:
        // 2 键：半趴 <-> 站立 往返
        return FSMStateName::FIXEDSTAND;
    case 8:
        // 8 键：从半趴继续往下到全趴
        return FSMStateName::FIXEDPRONE;
    default:
        break;
    }
    // FIXEDSTAND 按 8 的续段：半趴姿态过渡完成后自动继续到全趴
    if (ctrl_interfaces_.go_prone_after_down)
    {
        return FSMStateName::FIXEDPRONE;
    }
    return FSMStateName::FIXEDDOWN;
}
