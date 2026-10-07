//
// Created by tlab-uav on 24-9-19.
//

#include "leg_pd_controller/LegPdController.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace leg_pd_controller {
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::CallbackReturn LegPdController::on_init() {
        try {
            joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);
            reference_interface_types_ =
                    auto_declare<std::vector<std::string> >("reference_interfaces", reference_interface_types_);
            state_interface_types_ = auto_declare<std::vector<
                std::string> >("state_interfaces", state_interface_types_);
            const size_t n = joint_names_.size();
            effort_limits_ = auto_declare<std::vector<double>>("effort_limits", std::vector<double>(n, 100.0));
            velocity_limits_ = auto_declare<std::vector<double>>("velocity_limits", std::vector<double>(n, 20.0));
            position_min_ = auto_declare<std::vector<double>>("position_min", std::vector<double>(n, -12.5));
            position_max_ = auto_declare<std::vector<double>>("position_max", std::vector<double>(n, 12.5));
            torque_response_time_ = auto_declare<double>("torque_response_time", 0.0);
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        const size_t joint_num = joint_names_.size();
        joint_effort_command_.assign(joint_num, 0);
        joint_position_command_.assign(joint_num, 0);
        joint_velocities_command_.assign(joint_num, 0);
        joint_kp_command_.assign(joint_num, 0);
        joint_kd_command_.assign(joint_num, 0);
        applied_torque_.assign(joint_num, 0);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration LegPdController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size());
        for (const auto &joint_name: joint_names_) {
            conf.names.push_back(joint_name + "/effort");
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration LegPdController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: state_interface_types_) {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }
        return conf;
    }

    controller_interface::CallbackReturn LegPdController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        const size_t n = joint_names_.size();
        if (effort_limits_.size() != n || velocity_limits_.size() != n ||
            position_min_.size() != n || position_max_.size() != n ||
            !std::isfinite(torque_response_time_) || torque_response_time_ < 0.0) {
            RCLCPP_ERROR(get_node()->get_logger(), "Invalid PD actuator parameter sizes or response time");
            return CallbackReturn::ERROR;
        }
        for (size_t i = 0; i < n; ++i) {
            if (!std::isfinite(effort_limits_[i]) || effort_limits_[i] <= 0.0 ||
                !std::isfinite(velocity_limits_[i]) || velocity_limits_[i] <= 0.0 ||
                !std::isfinite(position_min_[i]) || !std::isfinite(position_max_[i]) ||
                position_min_[i] >= position_max_[i]) {
                RCLCPP_ERROR(get_node()->get_logger(), "Invalid PD actuator limits for %s", joint_names_[i].c_str());
                return CallbackReturn::ERROR;
            }
        }
        reference_interfaces_.resize(joint_names_.size() * 5, std::numeric_limits<double>::quiet_NaN());
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LegPdController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        joint_effort_command_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();
        std::fill(applied_torque_.begin(), applied_torque_.end(), 0.0);

        // assign effort command interface
        for (auto &interface: command_interfaces_) {
            joint_effort_command_interface_.emplace_back(interface);
        }

        // assign state interfaces
        for (auto &interface: state_interfaces_) {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LegPdController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        for (auto &interface : joint_effort_command_interface_) interface.get().set_value(0.0);
        std::fill(applied_torque_.begin(), applied_torque_.end(), 0.0);
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    bool LegPdController::on_set_chained_mode(bool /*chained_mode*/) {
        return true;
    }

    controller_interface::return_type LegPdController::update_and_write_commands(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
        if (joint_names_.size() != joint_effort_command_.size() ||
            joint_names_.size() != joint_kp_command_.size() ||
            joint_names_.size() != joint_kd_command_.size() ||
            joint_names_.size() != joint_velocities_command_.size() ||
            joint_names_.size() != joint_position_command_.size() ||
            joint_names_.size() != joint_position_state_interface_.size() ||
            joint_names_.size() != joint_velocity_state_interface_.size() ||
            joint_names_.size() != joint_effort_command_interface_.size()) {
            std::cout << "joint_names_.size() = " << joint_names_.size() << std::endl;
            std::cout << "joint_effort_command_.size() = " << joint_effort_command_.size() << std::endl;
            std::cout << "joint_kp_command_.size() = " << joint_kp_command_.size() << std::endl;
            std::cout << "joint_position_command_.size() = " << joint_position_command_.size() << std::endl;
            std::cout << "joint_position_state_interface_.size() = " << joint_position_state_interface_.size() <<
                    std::endl;
            std::cout << "joint_velocity_state_interface_.size() = " << joint_velocity_state_interface_.size() <<
                    std::endl;
            std::cout << "joint_effort_command_interface_.size() = " << joint_effort_command_interface_.size() <<
                    std::endl;

            throw std::runtime_error("Mismatch in vector sizes in update_and_write_commands");
        }

        const double dt = period.seconds();
        const double alpha = torque_response_time_ > 0.0 && std::isfinite(dt) && dt > 0.0
            ? -std::expm1(-dt / torque_response_time_) : 0.0;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            const double q = joint_position_state_interface_[i].get().get_value();
            const double qd = joint_velocity_state_interface_[i].get().get_value();
            if (!std::isfinite(q) || !std::isfinite(qd) ||
                !std::isfinite(joint_position_command_[i]) || !std::isfinite(joint_velocities_command_[i]) ||
                !std::isfinite(joint_effort_command_[i]) || !std::isfinite(joint_kp_command_[i]) ||
                !std::isfinite(joint_kd_command_[i]) || !std::isfinite(dt) || dt <= 0.0) {
                applied_torque_[i] = 0.0;
                joint_effort_command_interface_[i].get().set_value(0.0);
                continue;
            }
            // MIT PD: match hardware command ranges, then limit the SUM (feedforward + P + D).
            const double q_ref = std::clamp(joint_position_command_[i], position_min_[i], position_max_[i]);
            const double qd_ref = std::clamp(joint_velocities_command_[i], -velocity_limits_[i], velocity_limits_[i]);
            const double kp = std::clamp(joint_kp_command_[i], 0.0, 500.0);
            const double kd = std::clamp(joint_kd_command_[i], 0.0, 5.0);
            const double ff = std::clamp(joint_effort_command_[i], -effort_limits_[i], effort_limits_[i]);
            const double requested = std::clamp(ff + kp * (q_ref - q) + kd * (qd_ref - qd),
                                                -effort_limits_[i], effort_limits_[i]);
            // Exact discretization of T * d(tau)/dt + tau = requested; T=0 disables actuator lag.
            applied_torque_[i] = torque_response_time_ > 0.0
                ? applied_torque_[i] + alpha * (requested - applied_torque_[i]) : requested;
            joint_effort_command_interface_[i].get().set_value(applied_torque_[i]);
        }

        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> LegPdController::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;

        int ind = 0;
        std::string controller_name = get_node()->get_name();
        for (const auto &joint_name: joint_names_) {
            std::cout << joint_name << std::endl;
            reference_interfaces.emplace_back(controller_name, joint_name + "/position", &joint_position_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/velocity",
                                              &joint_velocities_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/effort", &joint_effort_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/kp", &joint_kp_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/kd", &joint_kd_command_[ind]);
            ind++;
        }

        return reference_interfaces;
    }

#ifdef ROS2_CONTROL_VERSION_LT_3
    controller_interface::return_type LegPdController::update_reference_from_subscribers() {
        return controller_interface::return_type::OK;
    }
#else
    controller_interface::return_type LegPdController::update_reference_from_subscribers(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        return controller_interface::return_type::OK;
    }
#endif
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(leg_pd_controller::LegPdController, controller_interface::ChainableControllerInterface);
