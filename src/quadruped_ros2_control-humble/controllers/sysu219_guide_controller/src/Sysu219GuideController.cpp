//
// Created by tlab-uav on 24-9-6.
//

#include "sysu219_guide_controller/Sysu219GuideController.h"

#include <sysu219_guide_controller/gait/WaveGenerator.h>
#include "sysu219_guide_controller/robot/QuadrupedRobot.h"
#include "sysu219_guide_controller/common/mathTools.h"
#include "sysu219_guide_controller/debug/DebugConfig.h"

#include <Eigen/Geometry>

namespace sysu219_guide_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration Sysu219GuideController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration Sysu219GuideController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type Sysu219GuideController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        const auto debug_begin = quadruped_debug::Csv_DebugMode ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        const auto debug_system_begin = quadruped_debug::Csv_DebugMode ? getSystemTime() : 0LL;
        bool trotting_ran = false;
        // auto now = std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_diff = now - last_update_time_;
        // last_update_time_ = now;
        //
        // // Calculate the frequency
        // update_frequency_ = 1.0 / time_diff.count();
        // RCLCPP_INFO(get_node()->get_logger(), "Update frequency: %f Hz", update_frequency_);

        if (ctrl_component_.robot_model_ == nullptr)
        {
            return controller_interface::return_type::OK;
        }

        ctrl_component_.robot_model_->update();
        ctrl_component_.wave_generator_->update(period.seconds());
        ctrl_component_.estimator_->update();

        // 仅 Gazebo 配置开启；所有 FSM 状态都以约 25 Hz 输出 MPC 的估计质心。
        if (com_estimated_pub_ && ctrl_component_.convex_mpc_) {
            const auto stamp = get_node()->get_clock()->now();
            const double now_s = stamp.seconds();
            if (now_s < last_com_publish_s_ || now_s - last_com_publish_s_ >= 0.04) {
                const Vec3 com_G = ctrl_component_.estimator_->getPosition()
                    + ctrl_component_.estimator_->getRotation()
                      * ctrl_component_.convex_mpc_->comOffsetBody();
                if (com_G.allFinite()) {
                    geometry_msgs::msg::PointStamped msg;
                    msg.header.frame_id = "world";
                    msg.header.stamp = stamp;
                    msg.point.x = com_G(0);
                    msg.point.y = com_G(1);
                    msg.point.z = com_G(2);
                    com_estimated_pub_->publish(msg);
                }
                last_com_publish_s_ = now_s;
            }
        }

        if (tf_broadcaster_)
        {
            geometry_msgs::msg::TransformStamped tf_msg;

            Vec3 pos_body = ctrl_component_.estimator_->getPosition();
            RotMat rotation_body = ctrl_component_.estimator_->getRotation();
            Eigen::Quaterniond q(rotation_body);

            tf_msg.header.stamp = get_node()->get_clock()->now();
            tf_msg.header.frame_id = "world";
            tf_msg.child_frame_id = base_name_;

            tf_msg.transform.translation.x = pos_body(0);
            tf_msg.transform.translation.y = pos_body(1);
            tf_msg.transform.translation.z = pos_body(2);

            tf_msg.transform.rotation.w = q.w();
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();

            tf_broadcaster_->sendTransform(tf_msg);
        }


        if (ctrl_interfaces_.body_debug_pub)
        {
            std_msgs::msg::Float64MultiArray msg;

            Vec3 pos_body = ctrl_component_.estimator_->getPosition();
            Vec3 vel_body = ctrl_component_.estimator_->getVelocity();
            RotMat rotation_body = ctrl_component_.estimator_->getRotation();
            Vec3 rpy_body = rotMatToRPY(rotation_body);

            const auto feet_2b = ctrl_component_.robot_model_->getFeet2BPositions();

            // 0-2: pos_body
            msg.data.push_back(pos_body(0));
            msg.data.push_back(pos_body(1));
            msg.data.push_back(pos_body(2));

            // 3-5: vel_body
            msg.data.push_back(vel_body(0));
            msg.data.push_back(vel_body(1));
            msg.data.push_back(vel_body(2));

            // // 6-8: roll pitch yaw
            msg.data.push_back(rpy_body(0));
            msg.data.push_back(rpy_body(1));
            msg.data.push_back(rpy_body(2));

            // 三轴加速度
            const double ax = ctrl_interfaces_.imu_state_interface_[7].get().get_value();
            const double ay = ctrl_interfaces_.imu_state_interface_[8].get().get_value();
            const double az = ctrl_interfaces_.imu_state_interface_[9].get().get_value();

            msg.data.push_back(ax);
            msg.data.push_back(ay);
            msg.data.push_back(az);

            // // 0-2: FR x y z
            // msg.data.push_back(feet_2b[0].p.x());
            // msg.data.push_back(feet_2b[0].p.y());
            // msg.data.push_back(feet_2b[0].p.z());

            // // 3-5: FL x y z
            // msg.data.push_back(feet_2b[1].p.x());
            // msg.data.push_back(feet_2b[1].p.y());
            // msg.data.push_back(feet_2b[1].p.z());

            // // 6-8: RR x y z
            // msg.data.push_back(feet_2b[2].p.x());
            // msg.data.push_back(feet_2b[2].p.y());
            // msg.data.push_back(feet_2b[2].p.z());

            // // 9-11: RL x y z
            // msg.data.push_back(feet_2b[3].p.x());
            // msg.data.push_back(feet_2b[3].p.y());
            // msg.data.push_back(feet_2b[3].p.z());


            ctrl_interfaces_.body_debug_pub->publish(msg);
        }

        // 2026.04.28关键修改：把状态机的更新放在最后，否则会漏掉一拍新状态
        if (mode_ == FSMMode::NORMAL)
        {
            next_state_name_ = current_state_->checkChange();

            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                // 兜底：状态名没有对应实现时 next_state_ 会是空指针，绝不能让下面直接解引用
                if (!next_state_)
                {
                    RCLCPP_ERROR(get_node()->get_logger(),
                                 "FSM 找不到状态实现（state id %d），强制切到 PASSIVE",
                                 static_cast<int>(next_state_name_));
                    next_state_ = state_list_.passive;
                }
                // 记下"上一状态名"，让 next_state_ 的 enter() 能据此决定过渡策略
                next_state_->previous_state_name = current_state_->state_name;
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
            else
            {
                current_state_->run(time, period);
                trotting_ran = current_state_ == state_list_.trotting;
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        // ========== 发布关节目标位置（cmd_pos）镜像到 /joint_cmd_states ==========
        if (joint_cmd_pub_) {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = time;
            msg.name = joint_names_;
            msg.position.resize(joint_names_.size());
            for (size_t i = 0; i < joint_names_.size() && i < ctrl_interfaces_.joint_position_command_interface_.size(); ++i) {
                msg.position[i] = ctrl_interfaces_.joint_position_command_interface_[i].get().get_value();
            }
            joint_cmd_pub_->publish(msg);
        }

        if (quadruped_debug::Csv_DebugMode && trotting_ran)
            state_list_.trotting->recordDebug(time, period, debug_begin, debug_system_begin);
        // 仿真诊断：所有 FSM 状态都可采集；50 Hz，无订阅者时不组装消息。
        if (quadruped_debug::Csv_DebugMode && estimator_debug_pub_ &&
            estimator_debug_pub_->get_subscription_count() > 0 &&
            (last_estimator_debug_s_ < 0.0 || time.seconds() < last_estimator_debug_s_ ||
             time.seconds() - last_estimator_debug_s_ >= 0.02 - 1e-9)) {
            const auto& est = ctrl_component_.estimator_;
            std_msgs::msg::Float64MultiArray msg;
            msg.data.reserve(28);
            msg.data = {time.seconds(), static_cast<double>(current_state_->state_name), period.seconds()};
            const auto append = [&](const auto& values) {
                for (int i = 0; i < values.size(); ++i) msg.data.push_back(values(i));
            };
            append(est->getPosition()); append(est->getVelocity());
            append(est->debugVelocityPredicted()); append(est->debugVelocityUnfiltered());
            append(est->debugAcceleration());
            append(ctrl_component_.wave_generator_->contact_);
            append(ctrl_component_.wave_generator_->phase_);
            msg.data.push_back(est->debugDt());
            msg.data.push_back(static_cast<double>(mode_));
            estimator_debug_pub_->publish(msg);
            last_estimator_debug_s_ = time.seconds();
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn Sysu219GuideController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);
            auto_declare<bool>("gazebo_com_visualization", false);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            feet_names_ =
                auto_declare<std::vector<std::string>>("feet_names", feet_names_);

            // pose parameters
            prone_pos_ = auto_declare<std::vector<double>>("prone_pos", prone_pos_);
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);
            prone_kp_ = auto_declare<double>("prone_kp", prone_kp_);
            prone_kd_ = auto_declare<double>("prone_kd", prone_kd_);

            // 姿态向量长度校验：三个姿态都必须恰好 12 个关节角，
            // 否则各固定姿态状态里的 target_pos_[i] (i < 12) 会越界读
            const auto check_pose_size = [this](const char* name, const std::vector<double>& pose)
            {
                if (pose.size() == 12)
                {
                    return true;
                }
                RCLCPP_ERROR(get_node()->get_logger(),
                             "参数 %s 需要 12 个关节角（FR/FL/RR/RL × hip/thigh/calf），当前 %zu 个",
                             name, pose.size());
                return false;
            };

            if (!check_pose_size("prone_pos", prone_pos_)
                || !check_pose_size("down_pos", down_pos_)
                || !check_pose_size("stand_pos", stand_pos_))
            {
                return controller_interface::CallbackReturn::ERROR;
            }

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_);
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Sysu219GuideController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                // Handle message
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                    ctrl_interfaces_, msg->data, feet_names_, base_name_);
                ctrl_component_.balance_ctrl_ = std::make_shared<BalanceCtrl>(ctrl_component_.robot_model_);
                ctrl_component_.convex_mpc_ = std::make_shared<ConvexMpcSolver>();
            });

        // 摆动保持 0.325 s，支撑加倍为 0.650 s；每次对角交接四足支撑 0.1625 s。
        ctrl_component_.wave_generator_ = std::make_shared<WaveGenerator>(0.975, 2.0 / 3.0, Vec4(0, 0.5, 0.5, 0));

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    Sysu219GuideController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // ========== 1. 【关键修改】最先赋值 node ==========
        ctrl_interfaces_.node = get_node(); // <-- 移到最前面！
        estimator_debug_pub_.reset();
        last_estimator_debug_s_ = -1.0;
        if (quadruped_debug::Csv_DebugMode && get_node()->get_parameter("use_sim_time").as_bool())
            estimator_debug_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/estimator_debug", rclcpp::SensorDataQoS());
        com_estimated_pub_.reset();
        last_com_publish_s_ = -1.0;
        if (get_node()->get_parameter("gazebo_com_visualization").as_bool() &&
            get_node()->get_parameter("use_sim_time").as_bool()) {
            com_estimated_pub_ = get_node()->create_publisher<geometry_msgs::msg::PointStamped>(
                "/com_estimated", rclcpp::QoS(1));
        }
        ctrl_interfaces_.body_debug_pub = ctrl_interfaces_.node->create_publisher<std_msgs::msg::Float64MultiArray>("/body_debug", 10);
        joint_cmd_pub_ = ctrl_interfaces_.node->create_publisher<sensor_msgs::msg::JointState>("/joint_cmd_states", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        if (quadruped_debug::Csv_DebugMode) {
            const auto& positions = ctrl_interfaces_.joint_position_state_interface_;
            const auto& velocities = ctrl_interfaces_.joint_velocity_state_interface_;
            const auto& efforts = ctrl_interfaces_.joint_effort_state_interface_;
            RCLCPP_INFO(get_node()->get_logger(), "[JOINT_DEBUG] position_count=%zu velocity_count=%zu effort_count=%zu",
                positions.size(), velocities.size(), efforts.size());
            for (size_t i = 0; i < std::min(positions.size(), velocities.size()); ++i) {
                const std::string effort_name = i < efforts.size() ? efforts[i].get().get_name() : "missing";
                RCLCPP_INFO(get_node()->get_logger(), "[JOINT_DEBUG] index=%zu position=%s velocity=%s effort=%s",
                    i, positions[i].get().get_name().c_str(), velocities[i].get().get_name().c_str(), effort_name.c_str());
            }
        }

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedProne = std::make_shared<StateFixedProne>(ctrl_interfaces_, prone_pos_, prone_kp_, prone_kd_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.swingTest = std::make_shared<StateSwingTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_);
        state_list_.balanceTest = std::make_shared<StateBalanceTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.trotting = std::make_shared<StateTrotting>(ctrl_interfaces_, ctrl_component_);
        state_list_.rlWalk = std::make_shared<StateRLWalk>(ctrl_interfaces_, ctrl_component_);

        // Initialize FSM
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;


        // ctrl_interfaces_.node = get_node(); // 将节点指针传递给 CtrlInterfaces 以供 FSM 状态使用
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Sysu219GuideController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        estimator_debug_pub_.reset();
        com_estimated_pub_.reset();
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    Sysu219GuideController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    Sysu219GuideController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    Sysu219GuideController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> Sysu219GuideController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDPRONE:
            return state_list_.fixedProne;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::FIXEDSTAND:
            return state_list_.fixedStand;
        case FSMStateName::FREESTAND:
            return state_list_.freeStand;
        case FSMStateName::TROTTING:
            return state_list_.trotting;
        case FSMStateName::SWINGTEST:
            return state_list_.swingTest;
        case FSMStateName::BALANCETEST:
            return state_list_.balanceTest;
        case FSMStateName::RLWALK:
            return state_list_.rlWalk;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sysu219_guide_controller::Sysu219GuideController, controller_interface::ControllerInterface);
