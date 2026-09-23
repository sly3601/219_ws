//
// Created by tlab-uav on 24-9-11.
//

#include "controller_common/FSM/StateFixedProne.h"

#include <cmath>

StateFixedProne::StateFixedProne(CtrlInterfaces& ctrl_interfaces,
                                 const std::vector<double>& target_pos,
                                 const double kp,
                                 const double kd)
    : FSMState(FSMStateName::FIXEDPRONE, "fixed prone", ctrl_interfaces),
      kp_(kp), kd_(kd)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < 12; i++)
    {
        target_pos_[i] = target_pos[i];
    }
}

void StateFixedProne::enter()
{
    for (int i = 0; i < 12; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        // 以"当前实际生效的 kp/kd"作为过渡起点，保证刚度连续、不产生突变
        kp_start_[i] = ctrl_interfaces_.joint_kp_command_interface_[i].get().get_value();
        kd_start_[i] = ctrl_interfaces_.joint_kd_command_interface_[i].get().get_value();
    }
    ctrl_interfaces_.control_inputs_.command = 0;

    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_start_[i]);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_start_[i]);
    }
    percent_ = 0.0;
}

void StateFixedProne::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    // 用tanh做平滑过渡，范围[0, 1]
    const double phase = std::tanh(percent_);
    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(
            kp_start_[i] + (kp_ - kp_start_[i]) * phase);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(
            kd_start_[i] + (kd_ - kd_start_[i]) * phase);
    }
}

void StateFixedProne::exit()
{
    percent_ = 0;
}

FSMStateName StateFixedProne::checkChange()
{
    // 失能优先：1 键不受姿态过渡锁定期限制，随时可切
    if (ctrl_interfaces_.control_inputs_.command == 1)
    {
        return FSMStateName::PASSIVE;
    }
    if (percent_ < 1.5)
    {
        return FSMStateName::FIXEDPRONE;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 2:
        // 全趴是 2 键往返的底端：再按 2 回到半趴，不直接失能。
        // 要失能请按 1。
        return FSMStateName::FIXEDDOWN;
    default:
        return FSMStateName::FIXEDPRONE;
    }
}
