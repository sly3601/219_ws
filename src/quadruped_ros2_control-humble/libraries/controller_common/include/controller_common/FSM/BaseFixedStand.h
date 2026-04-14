//
// Created by biao on 24-9-10.
//
#pragma once

#include "FSMState.h"
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class BaseFixedStand : public FSMState
{
public:
    BaseFixedStand(CtrlInterfaces& ctrl_interfaces,
                   const std::vector<double>& target_pos,
                   double kp,
                   double kd);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

    /* 新的键盘控制节点微调关节使用 */
    void fixedstandOffsetCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

protected:
    double target_pos_[12] = {};
    double start_pos_[12] = {};
    rclcpp::Time start_time_;

    double kp_, kd_;

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;

    /* 新的键盘控制节点微调关节使用 */
    double offset_pos_[12] = {};
    double cmd_pos_[12] = {};

    bool fixedstand_active_ = false;

    double offset_step_ = 0.01;   // 每次按键调 0.01 rad
    double offset_limit_ = 0.60;  // 防止调飞，可改

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fixedstand_offset_sub_;
};
