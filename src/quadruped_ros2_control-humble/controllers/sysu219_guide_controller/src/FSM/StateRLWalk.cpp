#include "sysu219_guide_controller/FSM/StateRLWalk.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Geometry>

namespace {
namespace cfg = sysu219_trot;
constexpr double kPi = 3.14159265358979323846;

template <typename Interfaces>
std::size_t findInterface(const Interfaces& interfaces, const std::string& joint, const std::string& type) {
    const std::string suffix = joint + "/" + type;
    std::size_t result = interfaces.size();
    for (std::size_t i = 0; i < interfaces.size(); ++i) {
        const std::string name = interfaces[i].get().get_name();
        const bool match = name == suffix ||
            (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0 &&
             name[name.size() - suffix.size() - 1] == '/');
        if (!match) continue;
        if (result != interfaces.size()) throw std::runtime_error("Duplicate interface: " + suffix);
        result = i;
    }
    if (result == interfaces.size()) throw std::runtime_error("Missing interface: " + suffix);
    return result;
}

double preprocess(double value, double lower, double upper, double scale) {
    if (!std::isfinite(value)) throw std::runtime_error("Nonfinite IMU/joint observation");
    return std::clamp(value, lower, upper) * scale;  // Isaac Lab: clip, then scale
}
}  // namespace

StateRLWalk::StateRLWalk(CtrlInterfaces& interfaces, CtrlComponent& /*ctrl_component*/)
    : FSMState(FSMStateName::RLWALK, "rlwalk", interfaces) {
    observation_.resize(cfg::kObservationDim);
    kp_target_ = cfg::kKp;
    kd_target_ = cfg::kKd;
    // on_activate constructs states before the real-time update loop starts.
    // Resolve/load here, not inside enter() on a running controller thread.
    try {
        if (!ctrl_interfaces_.node) throw std::runtime_error("Missing controller node");
        transition_s_ = ctrl_interfaces_.node->get_parameter("rl_pose_transition_s").as_double();
        entry_timeout_s_ = ctrl_interfaces_.node->get_parameter("rl_entry_timeout_s").as_double();
        entry_tolerance_rad_ = ctrl_interfaces_.node->get_parameter("rl_entry_tolerance_rad").as_double();
        entry_velocity_rad_s_ = ctrl_interfaces_.node->get_parameter("rl_entry_velocity_rad_s").as_double();
        max_period_s_ = ctrl_interfaces_.node->get_parameter("rl_max_control_period_s").as_double();
        if (!std::isfinite(transition_s_) || transition_s_ <= 0 ||
            transition_s_ > cfg::kSettle ||
            !std::isfinite(entry_timeout_s_) || entry_timeout_s_ < transition_s_ ||
            !std::isfinite(entry_tolerance_rad_) || entry_tolerance_rad_ <= 0 ||
            !std::isfinite(entry_velocity_rad_s_) || entry_velocity_rad_s_ <= 0 ||
            !std::isfinite(max_period_s_) || max_period_s_ < cfg::kPolicyDt)
            throw std::runtime_error("Invalid RL entry/timing parameters");
        resolveInterfaces();
        mapped_ = true;
        const auto names = ctrl_interfaces_.node->get_parameter("joints").as_string_array();
        const auto stand = ctrl_interfaces_.node->get_parameter("stand_pos").as_double_array();
        if (names.size() != cfg::kActionDim || stand.size() != cfg::kActionDim)
            throw std::runtime_error("ROS joints/stand_pos must contain 12 entries");
        for (std::size_t i = 0; i < cfg::kActionDim; ++i) {
            if (names[i] != cfg::kJointNames[i] || !std::isfinite(stand[i]))
                throw std::runtime_error("Invalid ROS stand_pos or joint order");
            if (std::abs(stand[i] - cfg::kDefaultPosition[i]) > 1e-9)
                throw std::runtime_error("Policy default pose differs from ROS stand_pos at " + names[i] +
                    "; retrain with ROS stand_pos and export the model and TrotPolicyConfig.h together");
            default_position_[i] = stand[i];
        }
        q_target_ = default_position_;
        std::filesystem::path path(ctrl_interfaces_.node->get_parameter("rl_model_path").as_string());
        if (path.empty()) throw std::runtime_error("Empty rl_model_path");
        if (path.is_relative())
            path = std::filesystem::path(ament_index_cpp::get_package_share_directory("sysu219_guide_controller")) / "model" / path;
        model_path_ = path.string();
        if (!policy_.load(model_path_)) throw std::runtime_error("Policy model load/contract validation failed");
        configured_ = true;
    } catch (const std::exception& error) {
        configuration_error_ = error.what();
        if (ctrl_interfaces_.node)
            RCLCPP_ERROR(ctrl_interfaces_.node->get_logger(), "[RLWALK] unavailable: %s", configuration_error_.c_str());
    }
}

void StateRLWalk::resolveInterfaces() {
    for (std::size_t i = 0; i < cfg::kActionDim; ++i) {
        const std::string joint = cfg::kJointNames[i];
        q_index_[i] = findInterface(ctrl_interfaces_.joint_position_state_interface_, joint, "position");
        dq_index_[i] = findInterface(ctrl_interfaces_.joint_velocity_state_interface_, joint, "velocity");
        q_cmd_index_[i] = findInterface(ctrl_interfaces_.joint_position_command_interface_, joint, "position");
        dq_cmd_index_[i] = findInterface(ctrl_interfaces_.joint_velocity_command_interface_, joint, "velocity");
        torque_index_[i] = findInterface(ctrl_interfaces_.joint_torque_command_interface_, joint, "effort");
        kp_index_[i] = findInterface(ctrl_interfaces_.joint_kp_command_interface_, joint, "kp");
        kd_index_[i] = findInterface(ctrl_interfaces_.joint_kd_command_interface_, joint, "kd");
    }
    const std::string imu = ctrl_interfaces_.node->get_parameter("imu_name").as_string();
    const std::array<const char*, 7> fields = {
        "orientation.w", "orientation.x", "orientation.y", "orientation.z",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"};
    for (std::size_t i = 0; i < fields.size(); ++i)
        imu_index_[i] = findInterface(ctrl_interfaces_.imu_state_interface_, imu, fields[i]);
}

void StateRLWalk::enter() {
    ctrl_interfaces_.control_inputs_.command = 0;  // require a fresh command after any failure
    fault_ = false;
    policy_started_ = false;
    entry_elapsed_s_ = policy_elapsed_s_ = since_policy_s_ = 0.0;
    last_action_.fill(0.0);
    if (!configured_) {
        fail(configuration_error_);
        return;
    }
    for (std::size_t i = 0; i < cfg::kActionDim; ++i) {
        const double measured = ctrl_interfaces_.joint_position_state_interface_[q_index_[i]].get().get_value();
        const double commanded = ctrl_interfaces_.joint_position_command_interface_[q_cmd_index_[i]].get().get_value();
        if (!std::isfinite(measured) || !std::isfinite(commanded)) {
            fail("Invalid joint feedback or position command at entry");
            return;
        }
        // Preserve the load-bearing command rather than dropping PD error to zero.
        q_start_[i] = q_target_[i] = commanded;
        if (std::abs(commanded - default_position_[i]) > entry_tolerance_rad_) {
            fail("Standing command too far from trained pose; preserving current command");
            return;
        }
        last_action_[i] = std::clamp((commanded-default_position_[i])/cfg::kActionScale[i],
                                     -cfg::kActionClip, cfg::kActionClip);
        const double kp = ctrl_interfaces_.joint_kp_command_interface_[kp_index_[i]].get().get_value();
        const double kd = ctrl_interfaces_.joint_kd_command_interface_[kd_index_[i]].get().get_value();
        kp_start_[i] = kp_target_[i] = std::isfinite(kp) && kp >= 0 ? kp : cfg::kKp[i];
        kd_start_[i] = kd_target_[i] = std::isfinite(kd) && kd >= 0 ? kd : cfg::kKd[i];
    }
    RCLCPP_INFO(ctrl_interfaces_.node->get_logger(), "[RLWALK] %s; blending directly from current standing command", model_path_.c_str());
    sendJointCommand();
}

void StateRLWalk::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
    if (fault_ || !configured_) return;
    const double dt = period.seconds();
    if (!std::isfinite(dt) || dt < 0 || dt > max_period_s_) {
        fail("Invalid or excessive control period; policy target held");
        return;
    }
    if (dt == 0) return;
    if (!policy_started_) {
        entry_elapsed_s_ += dt;
        bool pose_ready = true;
        for (std::size_t i = 0; i < cfg::kActionDim; ++i) {
            const double q = ctrl_interfaces_.joint_position_state_interface_[q_index_[i]].get().get_value();
            const double dq = ctrl_interfaces_.joint_velocity_state_interface_[dq_index_[i]].get().get_value();
            if (!std::isfinite(q) || !std::isfinite(dq)) {
                fail("Invalid joint feedback before policy takeover");
                return;
            }
            pose_ready = pose_ready && std::abs(q - default_position_[i]) <= entry_tolerance_rad_ &&
                         std::abs(dq) <= entry_velocity_rad_s_;
        }
        sendJointCommand();
        if (!pose_ready) {
            if (entry_elapsed_s_ > entry_timeout_s_) fail("Policy entry feedback conditions not satisfied before timeout");
            return;
        }
        policy_started_ = true;
        policy_elapsed_s_ = since_policy_s_ = 0.0;
        runPolicy(0.0);  // clock starts immediately; first command preserves the standing target
        sendJointCommand();
        return;
    }
    policy_elapsed_s_ += dt;
    since_policy_s_ += dt;
    if (since_policy_s_ + 1e-12 >= cfg::kPolicyDt) {
        const double intervals = std::floor((since_policy_s_ + 1e-12) / cfg::kPolicyDt);
        since_policy_s_ = std::max(0.0, since_policy_s_ - intervals*cfg::kPolicyDt);
        runPolicy(policy_elapsed_s_);  // one fresh observation; never replay stale catch-up steps
    }
    sendJointCommand();
}

void StateRLWalk::buildObservation(double elapsed_s) {
    // The ROS description's imu_joint has identity rotation relative to trunk.
    // Actor uses IMU attitude/gyro directly; no estimated translation/contact prior.
    std::array<double, 7> imu{};
    for (std::size_t i = 0; i < imu.size(); ++i) {
        imu[i] = ctrl_interfaces_.imu_state_interface_[imu_index_[i]].get().get_value();
        if (!std::isfinite(imu[i])) throw std::runtime_error("Nonfinite IMU state");
    }
    Eigen::Quaterniond rotation(imu[0], imu[1], imu[2], imu[3]);
    if (!std::isfinite(rotation.norm()) || rotation.norm() < 1e-6)
        throw std::runtime_error("Invalid IMU quaternion");
    rotation.normalize();
    const Eigen::Vector3d gravity = rotation.toRotationMatrix().transpose() * Eigen::Vector3d(0, 0, -1);
    for (std::size_t i = 0; i < 3; ++i) {
        observation_[i] = static_cast<float>(preprocess(imu[4+i], cfg::kAngularVelocityClipMin, cfg::kAngularVelocityClipMax, cfg::kAngularVelocityScale));
        observation_[3+i] = static_cast<float>(preprocess(gravity(static_cast<int>(i)), cfg::kGravityClipMin, cfg::kGravityClipMax, cfg::kGravityScale));
    }
    for (std::size_t i = 0; i < cfg::kActionDim; ++i) {
        const double q = ctrl_interfaces_.joint_position_state_interface_[q_index_[i]].get().get_value();
        const double dq = ctrl_interfaces_.joint_velocity_state_interface_[dq_index_[i]].get().get_value();
        observation_[6+i] = static_cast<float>(preprocess(q - default_position_[i], cfg::kJointPositionClipMin, cfg::kJointPositionClipMax, cfg::kJointPositionScale));
        observation_[18+i] = static_cast<float>(preprocess(dq - cfg::kDefaultVelocity[i], cfg::kJointVelocityClipMin, cfg::kJointVelocityClipMax, cfg::kJointVelocityScale));
        observation_[30+i] = static_cast<float>(preprocess(last_action_[i], cfg::kLastActionClipMin, cfg::kLastActionClipMax, cfg::kLastActionScale));
    }
    const double running = std::max(elapsed_s - cfg::kSettle, 0.0);
    const double phase = std::fmod(running / cfg::kPeriod, 1.0);
    observation_[42] = static_cast<float>(std::sin(2*kPi*phase));
    observation_[43] = static_cast<float>(std::cos(2*kPi*phase));
    observation_[44] = static_cast<float>(std::clamp(running / cfg::kRamp, 0.0, 1.0));
}

bool StateRLWalk::runPolicy(double elapsed_s) {
    try {
        buildObservation(elapsed_s);
        auto action = policy_.forward(observation_);
        // Direct takeover from the previous load-bearing COMMAND. The trained
        // default remains the observation reference and policy action offset.
        const double u = std::clamp(elapsed_s / transition_s_, 0.0, 1.0);
        const double blend = u*u*u*(10.0 + u*(-15.0 + 6.0*u));
        std::array<double, 12> target{};
        for (std::size_t i = 0; i < cfg::kActionDim; ++i) {
            action[i] = std::clamp(action[i], -cfg::kActionClip, cfg::kActionClip);
            target[i] = std::clamp(default_position_[i] + cfg::kActionScale[i]*action[i], cfg::kTargetMin[i], cfg::kTargetMax[i]);
            if (u < 1.0) {
                target[i] = q_start_[i] + blend*(target[i] - q_start_[i]);
                // Equivalent applied action, bounded like the trained history.
                // Outside the policy envelope this is a clipped approximation.
                action[i] = std::clamp((target[i]-default_position_[i])/cfg::kActionScale[i],
                                       -cfg::kActionClip, cfg::kActionClip);
            }
            kp_target_[i] = kp_start_[i] + blend*(cfg::kKp[i] - kp_start_[i]);
            kd_target_[i] = kd_start_[i] + blend*(cfg::kKd[i] - kd_start_[i]);
        }
        q_target_ = target;
        // History must describe the clipped, blended action actually sent,
        // before joint scaling/default offset, rather than the raw actor output.
        last_action_ = action;
        return true;
    } catch (const std::exception& error) {
        fail(error.what());  // keep the last valid command; no zero-action jump
        return false;
    }
}

void StateRLWalk::sendJointCommand() {
    if (!mapped_) return;
    for (std::size_t i = 0; i < cfg::kActionDim; ++i) {
        ctrl_interfaces_.joint_position_command_interface_[q_cmd_index_[i]].get().set_value(q_target_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[dq_cmd_index_[i]].get().set_value(0.0);
        ctrl_interfaces_.joint_torque_command_interface_[torque_index_[i]].get().set_value(0.0);
        ctrl_interfaces_.joint_kp_command_interface_[kp_index_[i]].get().set_value(kp_target_[i]);
        ctrl_interfaces_.joint_kd_command_interface_[kd_index_[i]].get().set_value(kd_target_[i]);
    }
}

void StateRLWalk::fail(const std::string& message) {
    if (!fault_ && ctrl_interfaces_.node)
        RCLCPP_ERROR(ctrl_interfaces_.node->get_logger(), "[RLWALK] %s; leaving policy mode", message.c_str());
    fault_ = true;
}

void StateRLWalk::exit() {
    policy_started_ = false;
    last_action_.fill(0.0);
}

FSMStateName StateRLWalk::checkChange() {
    if (ctrl_interfaces_.control_inputs_.command == 1) return FSMStateName::PASSIVE;
    if (fault_) {
        // FixedStand requires valid joint feedback for its own smooth transition.
        if (mapped_) {
            for (std::size_t i = 0; i < cfg::kActionDim; ++i)
                if (!std::isfinite(ctrl_interfaces_.joint_position_state_interface_[q_index_[i]].get().get_value()) ||
                    !std::isfinite(ctrl_interfaces_.joint_velocity_state_interface_[dq_index_[i]].get().get_value()))
                    return FSMStateName::PASSIVE;
        }
        return FSMStateName::FIXEDSTAND;
    }
    if (ctrl_interfaces_.control_inputs_.command == 2) return FSMStateName::FIXEDSTAND;
    return FSMStateName::RLWALK;
}
