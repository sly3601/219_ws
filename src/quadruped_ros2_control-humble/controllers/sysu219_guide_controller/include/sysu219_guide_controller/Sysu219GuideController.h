//
// Created by tlab-uav on 24-9-6.
//

#ifndef QUADRUPEDCONTROLLER_H
#define QUADRUPEDCONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <controller_common/FSM/FSMState.h>
#include <controller_common/FSM/StatePassive.h>
#include <controller_common/FSM/StateFixedProne.h>
#include <controller_common/FSM/StateFixedDown.h>
#include <controller_common/common/enumClass.h>

#include "control/CtrlComponent.h"
#include "FSM/StateBalanceTest.h"
#include "FSM/StateFixedStand.h"
#include "FSM/StateFreeStand.h"
#include "FSM/StateSwingTest.h"
#include "FSM/StateTrotting.h"
#include "FSM/StateRLWalk.h"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace sysu219_guide_controller {
    struct FSMStateList {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StatePassive> passive;
        std::shared_ptr<StateFixedProne> fixedProne;
        std::shared_ptr<StateFixedDown> fixedDown;
        std::shared_ptr<StateFixedStand> fixedStand;
        std::shared_ptr<StateFreeStand> freeStand;
        std::shared_ptr<StateTrotting> trotting;

        std::shared_ptr<StateSwingTest> swingTest;
        std::shared_ptr<StateBalanceTest> balanceTest;
        std::shared_ptr<StateRLWalk> rlWalk;
    };

    class Sysu219GuideController final : public controller_interface::ControllerInterface {
    public:
        Sysu219GuideController() = default;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        CtrlComponent ctrl_component_;
        CtrlInterfaces ctrl_interfaces_;

    protected:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::string imu_name_;
        std::string base_name_;
        std::string command_prefix_;
        std::vector<std::string> imu_interface_types_;
        std::vector<std::string> feet_names_;

        // ========== 固定姿态参数 ==========
        // 实际生效值来自参数文件：
        //   仿真      descriptions/sysu219/sysu219_description/config/gazebo.yaml
        //   实机/MuJoCo 同目录下的 config/robot_control.yaml
        // 这里只是参数文件缺少对应键时的兜底默认值，调姿态请改参数文件。
        // 每个姿态 12 个关节角，顺序：FR/FL/RR/RL × (hip, thigh, calf)，单位 rad

        // 站立姿态（FIXEDSTAND）
        std::vector<double> stand_pos_ = {
            0.0, 0.9, -1.53,
            0.0, 0.9, -1.53,
            0.0, 0.9, -1.3,
            0.0, 0.9, -1.3
        };

        // 半趴姿态（FIXEDDOWN）
        // 取值来源：tools/config/joint_states/joint_snapshot_20260923_112924.yaml
        std::vector<double> down_pos_ = {
            -0.054, 1.111, -2.155,
            0.054, 1.111, -2.155,
            -0.163, 0.999, -2.094,
            0.163, 0.999, -2.094
        };

        // 全趴姿态（FIXEDPRONE，PASSIVE 按 2 进入）
        // 取值来源：tools/config/joint_states/joint_snapshot_20260923_213557.yaml
        // 已对左右腿取平均（FR↔FL、RR↔RL），使左右完全对称
        std::vector<double> prone_pos_ = {
            -0.394, 1.599, -2.519,
            0.394, 1.599, -2.519,
            -0.435, 1.587, -2.490,
            0.435, 1.587, -2.490
        };

        // 固定姿态的 MIT 增益（FIXEDDOWN / FIXEDSTAND 共用）
        double stand_kp_ = 260.0;
        double stand_kd_ = 3.8;

        // FIXEDPRONE 专用增益（默认与 stand_kp_/stand_kd_ 相同）。
        // FIXEDPRONE 的 kp/kd 是从"进入时的接口实际值"斜坡到这一对值，
        // 所以从 FIXEDDOWN(260/3.8) 切进来时会平滑过渡，不会突变。
        double prone_kp_ = 260.0;
        double prone_kd_ = 3.8;

        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // 关节目标位置镜像发布（话题：/joint_cmd_states），给 rqt 工具订阅
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr com_estimated_pub_;
        double last_com_publish_s_ = -1.0;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr estimator_debug_pub_;
        double last_estimator_debug_s_ = -1.0;



        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
        command_interface_map_ = {
            {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };

        FSMMode mode_ = FSMMode::NORMAL;
        std::string state_name_;
        FSMStateName next_state_name_ = FSMStateName::INVALID;
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;

        std::chrono::time_point<std::chrono::steady_clock> last_update_time_;
        double update_frequency_;

        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;




        
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"effort", &ctrl_interfaces_.joint_effort_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };
    };
}


#endif //QUADRUPEDCONTROLLER_H
