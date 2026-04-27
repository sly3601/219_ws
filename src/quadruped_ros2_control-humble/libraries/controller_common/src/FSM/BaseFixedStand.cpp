//
// Created by biao on 24-9-10.
//

#include "controller_common/FSM/BaseFixedStand.h"
#include <cmath>

#include <functional>
#include <std_msgs/msg/float64_multi_array.hpp>

BaseFixedStand::BaseFixedStand(CtrlInterfaces& ctrl_interfaces, const std::vector<double>& target_pos,
                               const double kp,
                               const double kd)
    : FSMState(FSMStateName::FIXEDSTAND, "fixed stand", ctrl_interfaces),
      kp_(kp), kd_(kd)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    // ========== 新增：初始化关节微调相关成员变量 ==========
    for (int i = 0; i < 12; i++)
    {
        target_pos_[i] = target_pos[i];
        offset_pos_[i] = 0.0;
        cmd_pos_[i] = target_pos[i];
    }
    if (ctrl_interfaces_.node) // 确保 node 不为空才创建订阅者
    {
        // 订阅键盘控制节点发布的微调指令，话题名为 "/fixedstand_offset_cmd"，消息类型为 std_msgs::msg::Int32MultiArray
        fixedstand_offset_sub_ =
            ctrl_interfaces_.node->create_subscription<std_msgs::msg::Int32MultiArray>(
                "/fixedstand_offset_cmd",
                10,
                std::bind(&BaseFixedStand::fixedstandOffsetCallback, this, std::placeholders::_1));
    }
}

void BaseFixedStand::enter()
{
    for (int i = 0; i < 12; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    }
    int watch_joint = 2;
    // 找刚性冲击尖峰用的log
    RCLCPP_WARN(
        ctrl_interfaces_.node->get_logger(),
        "[FIXEDSTAND enter] joint=%d q_meas=%.4f start_pos=%.4f target_pos=%.4f",
        watch_joint,
        ctrl_interfaces_.joint_position_state_interface_[watch_joint].get().get_value(),
        start_pos_[watch_joint],
        target_pos_[watch_joint]);


    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }
    ctrl_interfaces_.control_inputs_.command = 0;

    fixedstand_active_ = true; // 激活状态，允许微调
}

void BaseFixedStand::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < 12; i++)
    {
        // 关节位置指令 = 过渡阶段插值 + 键盘微调偏移
        cmd_pos_[i] = phase * target_pos_[i] + (1 - phase) * start_pos_[i] + offset_pos_[i];
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(cmd_pos_[i]);
    }

    // 找刚性冲击尖峰用的log
    int watch_joint = 2;
    if (percent_ < 0.02)
    {
        RCLCPP_WARN(
            ctrl_interfaces_.node->get_logger(),
            "[FIXEDSTAND run] joint=%d phase=%.4f q_meas=%.4f start=%.4f target=%.4f",
            watch_joint,
            phase,
            ctrl_interfaces_.joint_position_state_interface_[watch_joint].get().get_value(),
            start_pos_[watch_joint],
            target_pos_[watch_joint]);
    }

    if (ctrl_interfaces_.debug_pub)
    {
        // 发布当前的关节位置指令到ROS2话题，供调试使用
        std_msgs::msg::Float64MultiArray msg;
        for (int i = 0; i < 12; ++i)
        {
            msg.data.push_back(offset_pos_[i]); // 发布微调偏移量，方便观察
        }
        ctrl_interfaces_.debug_pub->publish(msg);
    }
}

void BaseFixedStand::exit()
{
    percent_ = 0;

    fixedstand_active_ = false; // 退出状态，禁止微调
}

FSMStateName BaseFixedStand::checkChange()
{
    if (percent_ < 1.5)
    {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDDOWN;
    default:
        return FSMStateName::FIXEDSTAND;
    }
}

// ================ 新增：键盘控制回调函数实现 =================
void BaseFixedStand::fixedstandOffsetCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (!fixedstand_active_)
    {
        return;
    }

    if (msg->data.size() < 2)
    {
        return;
    }

    const int joint_index = msg->data[0];
    const int op = msg->data[1];

    // joint_index = -1, op = 99 代表全部清零
    if (joint_index == -1 && op == 99)
    {
        for (int i = 0; i < 12; ++i)
        {
            offset_pos_[i] = 0.0;
        }
        return;
    }

    if (joint_index < 0 || joint_index >= 12)
    {
        return;
    }

    if (op > 0)
    {
        offset_pos_[joint_index] += offset_step_;
    }
    else if (op < 0)
    {
        offset_pos_[joint_index] -= offset_step_;
    }
    else
    {
        offset_pos_[joint_index] = 0.0;
    }

    if (offset_pos_[joint_index] > offset_limit_)
    {
        offset_pos_[joint_index] = offset_limit_;
    }
    else if (offset_pos_[joint_index] < -offset_limit_)
    {
        offset_pos_[joint_index] = -offset_limit_;
    }
}