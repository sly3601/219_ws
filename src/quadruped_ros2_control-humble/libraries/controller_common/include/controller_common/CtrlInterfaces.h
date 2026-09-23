//
// Created by biao on 24-9-10.
//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <vector>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <control_input_msgs/msg/inputs.hpp>

// ========== 新增：必须加的 ROS2 头文件 ==========
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

struct CtrlInterfaces
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_torque_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kp_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kd_command_interface_;


    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_effort_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    imu_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    foot_force_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    odom_state_interface_;


    control_input_msgs::msg::Inputs control_inputs_;
    int frequency_{};

    // FSM 用：FIXEDSTAND 按 8 时置位，表示"先到 FIXEDDOWN，等姿态过渡完成后再自动转到 FIXEDPRONE"。
    // 放在这里是因为切状态时各状态 enter() 会把 control_inputs_.command 清 0，按键信号留不住。
    bool go_prone_after_down{false};

    // ========== 新增：添加发布器指针 ==========
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> body_debug_pub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> debug_pub;
    // ========== 新增 添加 ROS2 节点智能指针 ==========
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node; // 注意这里是 LifecycleNode，因为 Controller 通常是生命周期节点


    CtrlInterfaces() = default;

    void clear()
    {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_kd_command_interface_.clear();
        joint_kp_command_interface_.clear();

        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        imu_state_interface_.clear();
        imu_state_interface_.clear();
        foot_force_state_interface_.clear();

        // 控制器重启（deactivate -> activate）时，FSM 的中间标志不能带到下一次运行
        go_prone_after_down = false;
    }
};

#endif //INTERFACE_H
