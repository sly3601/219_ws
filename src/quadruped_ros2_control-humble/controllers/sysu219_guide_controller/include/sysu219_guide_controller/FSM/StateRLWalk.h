#ifndef STATERLWALK_H
#define STATERLWALK_H

#include <array>
#include <vector>

#include "controller_common/FSM/FSMState.h"
#include "sysu219_guide_controller/control/CtrlComponent.h"
#include "sysu219_guide_controller/control/RLPolicy.h"

class StateRLWalk final : public FSMState {
public:
    explicit StateRLWalk(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component);

    void enter() override;
    void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    void updateCommand();
    void buildObservation(std::vector<float> &obs);
    void runPolicy();
    void applyActionToTarget();
    void sendJointCommand();

private:
    CtrlComponent &ctrl_component_;

    RLPolicy policy_;
    bool policy_loaded_ = false;

    // Gazebo 中 sysu219_guide_controller = 250Hz
    // IsaacLab: sim.dt = 0.005, decimation = 4 -> policy = 50Hz
    // 所以 250 / 5 = 50Hz
    int policy_counter_ = 0;
    int policy_decimation_ = 5;

    int print_counter_ = 0;

    // IsaacLab policy/action/joint order:
    // 0  FL_hip
    // 1  FR_hip
    // 2  RL_hip
    // 3  RR_hip
    // 4  FL_thigh
    // 5  FR_thigh
    // 6  RL_thigh
    // 7  RR_thigh
    // 8  FL_calf
    // 9  FR_calf
    // 10 RL_calf
    // 11 RR_calf
    //
    // ROS2 controller / Gazebo joint order:
    // 0  FR_hip
    // 1  FR_thigh
    // 2  FR_calf
    // 3  FL_hip
    // 4  FL_thigh
    // 5  FL_calf
    // 6  RR_hip
    // 7  RR_thigh
    // 8  RR_calf
    // 9  RL_hip
    // 10 RL_thigh
    // 11 RL_calf
    //
    // isaac_to_ctrl_[isaac_index] = ctrl_index
    std::array<int, 12> isaac_to_ctrl_ = {
        3,   // Isaac 0  FL_hip   -> Ctrl 3  FL_hip
        0,   // Isaac 1  FR_hip   -> Ctrl 0  FR_hip
        9,   // Isaac 2  RL_hip   -> Ctrl 9  RL_hip
        6,   // Isaac 3  RR_hip   -> Ctrl 6  RR_hip
        4,   // Isaac 4  FL_thigh -> Ctrl 4  FL_thigh
        1,   // Isaac 5  FR_thigh -> Ctrl 1  FR_thigh
        10,  // Isaac 6  RL_thigh -> Ctrl 10 RL_thigh
        7,   // Isaac 7  RR_thigh -> Ctrl 7  RR_thigh
        5,   // Isaac 8  FL_calf  -> Ctrl 5  FL_calf
        2,   // Isaac 9  FR_calf  -> Ctrl 2  FR_calf
        11,  // Isaac 10 RL_calf  -> Ctrl 11 RL_calf
        8    // Isaac 11 RR_calf  -> Ctrl 8  RR_calf
    };

    // IsaacLab default_joint_pos:
    // [0, 0, 0, 0, 0.67, 0.67, 0.67, 0.67, -1.3, -1.3, -1.3, -1.3]
    std::array<double, 12> q_default_isaac_ = {
        0.0, 0.0, 0.0, 0.0,
        0.67, 0.67, 0.67, 0.67,
        -1.30, -1.30, -1.30, -1.30
    };

    // q_target_ctrl_ 按 ROS2 controller / Gazebo 顺序保存
    std::array<double, 12> q_target_ctrl_{};

    // action 一律按 IsaacLab policy 顺序保存
    std::array<double, 12> last_action_isaac_{};
    std::array<double, 12> current_action_isaac_{};

    // SYSU219RoughEnvCfg: self.actions.joint_pos.scale = 0.25
    double action_scale_ = 0.25;

    // Gazebo / MIT PD 参数，对齐 IsaacLab actuator 配置
    double hip_kp_ = 285.0;
    double leg_kp_ = 265.0;
    double kd_ = 4.5;

    // policy command
    double cmd_vx_ = 0.0;
    double cmd_vy_ = 0.0;
    double cmd_wz_ = 0.0;
};

#endif // STATERLWALK_H