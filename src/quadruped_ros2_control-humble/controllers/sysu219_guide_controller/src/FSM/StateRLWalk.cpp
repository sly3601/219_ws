#include "sysu219_guide_controller/FSM/StateRLWalk.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

StateRLWalk::StateRLWalk(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component)
    : FSMState(FSMStateName::RLWALK, "rlwalk", ctrl_interfaces),
      ctrl_component_(ctrl_component) {
    q_target_ctrl_.fill(0.0);
    last_action_isaac_.fill(0.0);
    current_action_isaac_.fill(0.0);
}

void StateRLWalk::enter() {
    std::cout << "[StateRLWalk] enter" << std::endl;

    // 进入 RLWALK 时，从当前 Gazebo / ROS2 controller 关节角开始
    for (int ctrl_i = 0; ctrl_i < 12; ++ctrl_i) {
        const double q = ctrl_interfaces_.joint_position_state_interface_[ctrl_i].get().get_value();
        q_target_ctrl_[ctrl_i] = q;
    }

    last_action_isaac_.fill(0.0);
    current_action_isaac_.fill(0.0);

    cmd_vx_ = 0.0;
    cmd_vy_ = 0.0;
    cmd_wz_ = 0.0;

    policy_counter_ = 0;
    print_counter_ = 0;

    const std::string model_path =
        "/home/yzz/219_ws/src/quadruped_ros2_control-humble/controllers/"
        "sysu219_guide_controller/model/policy.onnx";

    policy_loaded_ = policy_.load(model_path);

    if (!policy_loaded_) {
        std::cerr << "[StateRLWalk] failed to load policy.onnx" << std::endl;
    } else {
        std::cout << "[StateRLWalk] policy loaded successfully" << std::endl;
    }

    std::cout << "[StateRLWalk] Isaac action order -> Controller order mapping:" << std::endl;
    std::cout << "  Isaac 0  FL_hip   -> Ctrl " << isaac_to_ctrl_[0] << std::endl;
    std::cout << "  Isaac 1  FR_hip   -> Ctrl " << isaac_to_ctrl_[1] << std::endl;
    std::cout << "  Isaac 2  RL_hip   -> Ctrl " << isaac_to_ctrl_[2] << std::endl;
    std::cout << "  Isaac 3  RR_hip   -> Ctrl " << isaac_to_ctrl_[3] << std::endl;
    std::cout << "  Isaac 4  FL_thigh -> Ctrl " << isaac_to_ctrl_[4] << std::endl;
    std::cout << "  Isaac 5  FR_thigh -> Ctrl " << isaac_to_ctrl_[5] << std::endl;
    std::cout << "  Isaac 6  RL_thigh -> Ctrl " << isaac_to_ctrl_[6] << std::endl;
    std::cout << "  Isaac 7  RR_thigh -> Ctrl " << isaac_to_ctrl_[7] << std::endl;
    std::cout << "  Isaac 8  FL_calf  -> Ctrl " << isaac_to_ctrl_[8] << std::endl;
    std::cout << "  Isaac 9  FR_calf  -> Ctrl " << isaac_to_ctrl_[9] << std::endl;
    std::cout << "  Isaac 10 RL_calf  -> Ctrl " << isaac_to_ctrl_[10] << std::endl;
    std::cout << "  Isaac 11 RR_calf  -> Ctrl " << isaac_to_ctrl_[11] << std::endl;

    sendJointCommand();
}

void StateRLWalk::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    updateCommand();

    if (policy_loaded_) {
        if (policy_counter_ <= 0) {
            runPolicy();
            applyActionToTarget();
            policy_counter_ = policy_decimation_;
        }
        policy_counter_--;
    }

    sendJointCommand();
}

void StateRLWalk::exit() {
    std::cout << "[StateRLWalk] exit" << std::endl;
    sendJointCommand();
}

FSMStateName StateRLWalk::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::RLWALK;
    }
}

void StateRLWalk::updateCommand() {
    const double deadzone = 0.05;

    double ly = ctrl_interfaces_.control_inputs_.ly;  // 前后
    double lx = ctrl_interfaces_.control_inputs_.lx;  // 左右
    double rx = ctrl_interfaces_.control_inputs_.rx;  // 转向

    if (std::abs(ly) < deadzone) {
        ly = 0.0;
    }
    if (std::abs(lx) < deadzone) {
        lx = 0.0;
    }
    if (std::abs(rx) < deadzone) {
        rx = 0.0;
    }

    ly = std::clamp(ly, -1.0, 1.0);
    lx = std::clamp(lx, -1.0, 1.0);
    rx = std::clamp(rx, -1.0, 1.0);

    // IsaacLab 训练范围：
    // lin_vel_x = (-0.5, 0.5)
    // lin_vel_y = (-0.05, 0.05)
    // ang_vel_z = (-0.5, 0.5)
    cmd_vx_ = 0.5 * ly;
    cmd_vy_ = 0.05 * lx;
    cmd_wz_ = 0.5 * rx;
}

void StateRLWalk::buildObservation(std::vector<float> &obs) {
    obs.clear();
    obs.reserve(48);

    // IsaacLab PolicyCfg observation order:
    // 0-2    base_lin_vel
    // 3-5    base_ang_vel
    // 6-8    projected_gravity
    // 9-11   velocity_commands
    // 12-23  joint_pos_rel      Isaac joint order
    // 24-35  joint_vel_rel      Isaac joint order
    // 36-47  last_action        Isaac action order

    // 0-2: base linear velocity in body frame
    Vec3 base_lin_vel_b = ctrl_component_.estimator_->getVelocity();

    obs.push_back(static_cast<float>(base_lin_vel_b(0)));
    obs.push_back(static_cast<float>(base_lin_vel_b(1)));
    obs.push_back(static_cast<float>(base_lin_vel_b(2)));

    // 3-5: base angular velocity in body frame
    Vec3 gyro_b = ctrl_component_.estimator_->getGyro();

    obs.push_back(static_cast<float>(gyro_b(0)));
    obs.push_back(static_cast<float>(gyro_b(1)));
    obs.push_back(static_cast<float>(gyro_b(2)));

    // 6-8: projected gravity in body frame
    RotMat R_wb = ctrl_component_.estimator_->getRotation();
    Vec3 gravity_w(0.0, 0.0, -1.0);
    Vec3 gravity_b = R_wb.transpose() * gravity_w;

    obs.push_back(static_cast<float>(gravity_b(0)));
    obs.push_back(static_cast<float>(gravity_b(1)));
    obs.push_back(static_cast<float>(gravity_b(2)));

    // 9-11: command vx, vy, wz
    obs.push_back(static_cast<float>(cmd_vx_));
    obs.push_back(static_cast<float>(cmd_vy_));
    obs.push_back(static_cast<float>(cmd_wz_));

    // 12-23: joint position relative to default pose, 必须按 IsaacLab joint order
    for (int isaac_i = 0; isaac_i < 12; ++isaac_i) {
        const int ctrl_i = isaac_to_ctrl_[isaac_i];
        const double q = ctrl_interfaces_.joint_position_state_interface_[ctrl_i].get().get_value();
        obs.push_back(static_cast<float>(q - q_default_isaac_[isaac_i]));
    }

    // 24-35: joint velocity, 必须按 IsaacLab joint order
    for (int isaac_i = 0; isaac_i < 12; ++isaac_i) {
        const int ctrl_i = isaac_to_ctrl_[isaac_i];
        const double qd = ctrl_interfaces_.joint_velocity_state_interface_[ctrl_i].get().get_value();
        obs.push_back(static_cast<float>(qd));
    }

    // 36-47: last action, 本来就是 IsaacLab action order
    for (int isaac_i = 0; isaac_i < 12; ++isaac_i) {
        obs.push_back(static_cast<float>(last_action_isaac_[isaac_i]));
    }

    if (obs.size() != 48) {
        std::cerr << "[StateRLWalk] obs size error: " << obs.size() << std::endl;
    }
}

void StateRLWalk::runPolicy() {
    std::vector<float> obs;
    buildObservation(obs);

    if (obs.size() != 48) {
        current_action_isaac_.fill(0.0);
        return;
    }

    try {
        current_action_isaac_ = policy_.forward(obs);
    } catch (const std::exception &e) {
        std::cerr << "[StateRLWalk] policy forward failed: " << e.what() << std::endl;
        current_action_isaac_.fill(0.0);
        return;
    }

    print_counter_++;

    // policy 50Hz，50 次约 1 秒打印一次
    if (print_counter_ >= 50) {
        print_counter_ = 0;

        std::cout << "[StateRLWalk] cmd = "
                  << cmd_vx_ << " " << cmd_vy_ << " " << cmd_wz_ << std::endl;

        std::cout << "[StateRLWalk] base_lin_vel_b = "
                  << obs[0] << " " << obs[1] << " " << obs[2] << std::endl;

        std::cout << "[StateRLWalk] gyro_b = "
                  << obs[3] << " " << obs[4] << " " << obs[5] << std::endl;

        std::cout << "[StateRLWalk] projected_gravity = "
                  << obs[6] << " " << obs[7] << " " << obs[8] << std::endl;

        std::cout << "[StateRLWalk] q_rel_isaac_order = ";
        for (int isaac_i = 0; isaac_i < 12; ++isaac_i) {
            std::cout << obs[12 + isaac_i] << " ";
        }
        std::cout << std::endl;

        std::cout << "[StateRLWalk] qd_isaac_order = ";
        for (int isaac_i = 0; isaac_i < 12; ++isaac_i) {
            std::cout << obs[24 + isaac_i] << " ";
        }
        std::cout << std::endl;

        std::cout << "[StateRLWalk] action_isaac_order = ";
        for (int isaac_i = 0; isaac_i < 12; ++isaac_i) {
            std::cout << current_action_isaac_[isaac_i] << " ";
        }
        std::cout << std::endl;

        std::cout << "[StateRLWalk] q_target_ctrl_order = ";
        for (int ctrl_i = 0; ctrl_i < 12; ++ctrl_i) {
            std::cout << q_target_ctrl_[ctrl_i] << " ";
        }
        std::cout << std::endl;
    }
}

void StateRLWalk::applyActionToTarget() {
    // policy action 是 IsaacLab order
    // controller command 必须转成 Gazebo / ROS2 controller order
    for (int isaac_i = 0; isaac_i < 12; ++isaac_i) {
        const int ctrl_i = isaac_to_ctrl_[isaac_i];
        q_target_ctrl_[ctrl_i] =
            q_default_isaac_[isaac_i] + current_action_isaac_[isaac_i] * action_scale_;
    }

    last_action_isaac_ = current_action_isaac_;
}

void StateRLWalk::sendJointCommand() {
    for (int ctrl_i = 0; ctrl_i < 12; ++ctrl_i) {
        ctrl_interfaces_.joint_position_command_interface_[ctrl_i].get().set_value(q_target_ctrl_[ctrl_i]);
        ctrl_interfaces_.joint_velocity_command_interface_[ctrl_i].get().set_value(0.0);
        ctrl_interfaces_.joint_torque_command_interface_[ctrl_i].get().set_value(0.0);

        // controller order 是每条腿 hip/thigh/calf 排列，所以 ctrl_i % 3 == 0 是 hip
        if (ctrl_i % 3 == 0) {
            ctrl_interfaces_.joint_kp_command_interface_[ctrl_i].get().set_value(hip_kp_);
        } else {
            ctrl_interfaces_.joint_kp_command_interface_[ctrl_i].get().set_value(leg_kp_);
        }

        ctrl_interfaces_.joint_kd_command_interface_[ctrl_i].get().set_value(kd_);
    }
}