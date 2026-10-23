#include "StateComIdentification.h"
#include "sysu219_guide_controller/common/mathTools.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <urdf/model.h>

namespace sysu219_guide_controller {
namespace {
constexpr int pose_count = 13;
constexpr double pi = 3.14159265358979323846;
const char* pose_names[] = {"reference", "lower", "center_after_lower", "higher", "center_after_higher",
    "pitch_positive", "center_after_pitch_positive", "pitch_negative", "center_after_pitch_negative",
    "roll_positive", "center_after_roll_positive", "roll_negative", "reference_repeat"};
Eigen::Vector3d poseGoal(int pose, double height, double angle) {
    switch (pose) {
        case 1: return {0, 0, -height}; case 3: return {0, 0, height};
        case 5: return {0, angle, 0}; case 7: return {0, -angle, 0};
        case 9: return {angle, 0, 0}; case 11: return {-angle, 0, 0};
        default: return Eigen::Vector3d::Zero();
    }
}
}
double StateComIdentification::now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
StateComIdentification::StateComIdentification(CtrlInterfaces& interfaces, CtrlComponent& component,
    const std::vector<std::string>& names, const std::vector<double>& stand,
    const std::string& base, const std::vector<std::string>& feet)
    : FSMState(FSMStateName::COMIDENTIFICATION, "COM identification", interfaces),
      component_(component), stand_(stand), base_(base), feet_(feet) {
    if (names.size() != 12 || stand.size() != 12 || feet.size() != 4)
        throw std::invalid_argument("COM identification requires 12 joints and four feet");
    size_t index = 0;
    for (const auto* leg : {"FR", "FL", "RR", "RL"})
        for (const auto* kind : {"hip", "thigh", "calf"})
            if (names[index++] != std::string(leg) + '_' + kind + "_joint")
                throw std::invalid_argument("COM identification joint order must be FR/FL/RR/RL");
    const auto check = [&names](const auto& values, const char* type) {
        if (values.size() != 12) throw std::invalid_argument("COM interface count mismatch");
        for (size_t j = 0; j < 12; ++j) {
            const std::string expected = names[j] + '/' + type;
            const auto actual = values[j].get().get_name();
            if (actual != expected && (actual.size() <= expected.size() ||
                actual.compare(actual.size() - expected.size() - 1, expected.size() + 1, '/' + expected) != 0))
                throw std::invalid_argument("COM interface name/order mismatch: " + actual);
        }
    };
    check(interfaces.joint_position_state_interface_, "position");
    check(interfaces.joint_velocity_state_interface_, "velocity");
    check(interfaces.joint_effort_state_interface_, "effort");
    check(interfaces.joint_position_command_interface_, "position");
    check(interfaces.joint_velocity_command_interface_, "velocity");
    check(interfaces.joint_torque_command_interface_, "effort");
    check(interfaces.joint_kp_command_interface_, "kp");
    check(interfaces.joint_kd_command_interface_, "kd");
    auto node = interfaces.node;
    const auto parameter = [node](const char* name, double value) {
        const std::string key = std::string("com_identification.") + name;
        if (!node->has_parameter(key)) node->declare_parameter<double>(key, value);
        const auto result = node->get_parameter(key).as_double();
        if (!std::isfinite(result)) throw std::invalid_argument("Nonfinite COM parameter");
        return result;
    };
    transition_ = parameter("transition_s", transition_);
    settling_ = parameter("settling_s", settling_);
    sample_seconds_ = parameter("sample_s", sample_seconds_);
    height_ = parameter("height_m", height_);
    angle_ = parameter("tilt_deg", 2.0) * pi / 180;
    if (transition_ < 8 || transition_ > 30 || settling_ < 1 || settling_ > 30 ||
        sample_seconds_ < 10 || sample_seconds_ > 30 || height_ <= 0 || height_ > 0.008 ||
        angle_ <= 0 || angle_ > 3 * pi / 180)
        throw std::invalid_argument("COM motion exceeds conservative parameter bounds");
    if (!node->has_parameter("com_identification.output_directory")) {
        // Source workspace path, also valid when the Windows workspace is shared with Linux.
        auto project = std::filesystem::path(__FILE__);
        for (int parent = 0; parent < 5; ++parent) project = project.parent_path();
        const auto dataset = project / "Troting_Dataset";
        if (!dataset.is_absolute() || !std::filesystem::is_directory(dataset))
            throw std::invalid_argument("COM找不到源码工作区Troting_Dataset；请将com_identification.output_directory指定到共享目录");
        node->declare_parameter<std::string>("com_identification.output_directory",
                                             (dataset / "COM_identification").string());
    }
    root_ = node->get_parameter("com_identification.output_directory").as_string();
    status_pub_ = node->create_publisher<std_msgs::msg::UInt8>("/com_identification/status", 10);
    feedback_pub_ = node->create_publisher<std_msgs::msg::String>("/com_identification/feedback", 10);
    directory_pub_ = node->create_publisher<std_msgs::msg::String>(
        "/com_identification/dataset", rclcpp::QoS(1).transient_local());
    timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this] {
        std_msgs::msg::UInt8 status; status.data = active_.load() ? status_.load() : 0;
        status_pub_->publish(status);
        if (const auto* text = notice_.exchange(nullptr)) {
            std_msgs::msg::String msg;
            msg.data = std::string(text) + " [pose " + std::to_string(pose_status_.load()) +
                       "/12: " + pose_names[pose_status_.load()] + ']';
            feedback_pub_->publish(msg);
        }
        if (recorder_.done()) {
            if (recorder_.failed()) {
                RCLCPP_ERROR_ONCE(ctrl_interfaces_.node->get_logger(), "COM 数据写入失败；本次结果不可用");
            } else {
                std_msgs::msg::String msg; msg.data = recorder_.directory(); directory_pub_->publish(msg);
            }
        }
    });
    command_sub_ = node->create_subscription<std_msgs::msg::String>("/com_identification/command", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "heartbeat") { heartbeat_.store(now()); return; }
            if (msg->data == "1") {
                entry_.store(false); request_.store(0); stop_.store(true); return;
            }
            if (msg->data == "a") {
                if (!model_ready_.load()) { notice("模型尚未就绪，未开始"); return; }
                if (!writer_started_.exchange(true)) {
                    std::ostringstream meta;
                    meta.precision(17);
                    meta << "{\"schema\":1,\"base\":\"" << base_ << "\",\"trunk\":\"trunk\",\"feet\":[";
                    for (size_t i = 0; i < 4; ++i) meta << (i ? "," : "") << '"' << feet_[i] << '"';
                    meta << "],\"stand_q\":[";
                    for (size_t j = 0; j < 12; ++j) meta << (j ? "," : "") << stand_[j];
                    meta << "],\"height_m\":" << height_ << ",\"tilt_rad\":" << angle_
                         << ",\"transition_s\":" << transition_ << ",\"settling_s\":" << settling_
                         << ",\"sample_s\":" << sample_seconds_ << ",\"pose_names\":[";
                    for (int i = 0; i < pose_count; ++i) meta << (i ? "," : "") << '"' << pose_names[i] << '"';
                    meta << "],\"force_source\":\"motor feedback estimate, not measured GRF\","
                         << "\"static_gate\":{\"joint_position_error_rad\":0.1,"
                         << "\"joint_speed_rad_s\":0.015,\"gyro_norm_rad_s\":0.015}}\n";
                    recorder_.start(root_, description_, meta.str());
                }
                if (!active_.load()) { stop_.store(false); entry_.store(true); }
                request_.store('a');
            } else if (msg->data == "b" && !active_.load()) {
                stop_.store(false); entry_.store(true);
            }
            else if (msg->data == "h" && active_.load()) request_.store('h');
        });
    emergency_sub_ = node->create_subscription<control_input_msgs::msg::Inputs>("/control_input", 10,
        [this](const control_input_msgs::msg::Inputs::SharedPtr msg) {
            if (active_.load() && msg->command == 1) stop_.store(true);
        });
    description_sub_ = node->create_subscription<std_msgs::msg::String>("/robot_description",
        rclcpp::QoS(1).transient_local(), [this](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == description_) return;
            if (writer_started_.load() || active_.load()) {
                model_ready_.store(false); stop_.store(true); notice("模型发生变化，已中止"); return;
            }
            urdf::Model model;
            if (!model.initString(msg->data) || !model.getLink(base_) || !model.getLink("trunk")) return;
            description_ = msg->data; model_ready_.store(true);
        });
}

bool StateComIdentification::healthy(bool entry) const {
    if (!model_ready_.load() || !component_.robot_model_ || !component_.robot_model_->wbc_model_ ||
        ctrl_interfaces_.imu_state_interface_.size() < 10 || now() - heartbeat_.load() > 0.5) return false;
    const auto& robot = *component_.robot_model_;
    for (size_t j = 0; j < 12; ++j) {
        const double q = ctrl_interfaces_.joint_position_state_interface_[j].get().get_value();
        const double dq = ctrl_interfaces_.joint_velocity_state_interface_[j].get().get_value();
        const double tau = ctrl_interfaces_.joint_effort_state_interface_[j].get().get_value();
        const double cmd = ctrl_interfaces_.joint_position_command_interface_[j].get().get_value();
        const double kp = ctrl_interfaces_.joint_kp_command_interface_[j].get().get_value();
        const double kd = ctrl_interfaces_.joint_kd_command_interface_[j].get().get_value();
        if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(tau) || !std::isfinite(cmd) ||
            !std::isfinite(kp) || !std::isfinite(kd) || kp <= 0 || kd <= 0 || std::abs(tau) > 80 ||
            q < robot.joint_lower_(j) + 0.03 || q > robot.joint_upper_(j) - 0.03 ||
            std::abs(dq) > (entry ? 0.03 : 0.4) || std::abs(q - cmd) > 0.12) return false;
        if (entry && (std::abs(cmd - stand_[j]) > 0.015 ||
            std::abs(ctrl_interfaces_.joint_velocity_command_interface_[j].get().get_value()) > 1e-6 ||
            !std::isfinite(ctrl_interfaces_.joint_torque_command_interface_[j].get().get_value()) ||
            std::abs(ctrl_interfaces_.joint_torque_command_interface_[j].get().get_value()) > 1e-6)) return false;
    }
    double imu[10];
    for (size_t i = 0; i < 10; ++i) {
        imu[i] = ctrl_interfaces_.imu_state_interface_[i].get().get_value();
        if (!std::isfinite(imu[i])) return false;
    }
    const Eigen::Quaterniond quat(imu[0], imu[1], imu[2], imu[3]);
    if (std::abs(quat.norm() - 1) > 0.01) return false;
    const Vec3 rpy = rotMatToRPY(quat.normalized().toRotationMatrix());
    return std::abs(rpy.x()) < 8 * pi / 180 && std::abs(rpy.y()) < 8 * pi / 180 &&
        Vec3(imu[4], imu[5], imu[6]).norm() < (entry ? 0.03 : 0.35);
}
bool StateComIdentification::consumeEntryRequest(bool from_fixed_stand) {
    if (!entry_.exchange(false)) return false;
    if (used_) { request_.store(0); notice("本次激活已执行过辨识，保持原FSM，不重新进入"); return false; }
    if (!from_fixed_stand || !healthy(true) || ctrl_interfaces_.control_inputs_.command == 1) {
        request_.store(0); notice("拒绝进入：须稳定FixStand、命令接近配置站姿、模型和键盘有效"); return false;
    }
    return true;
}
void StateComIdentification::setReturnTarget(const std::array<double, 12>& q) {
    for (int j = 0; j < 12; ++j) return_goal_(j) = q[j];
    return_target_ready_ = true;
}
FSMStateName StateComIdentification::checkChange() {
    if (phase_ == 6 && !stop_.load() && healthy() && settled() && recorder_.done() && !recorder_.failed())
        return FSMStateName::FIXEDSTAND;
    return FSMStateName::COMIDENTIFICATION;
}
void StateComIdentification::enter() {
    active_.store(true); phase_ = 4; status_.store(1);
    for (int j = 0; j < 12; ++j) {
        baseline_(j) = target_(j) = ctrl_interfaces_.joint_position_command_interface_[j].get().get_value();
        kp_(j) = ctrl_interfaces_.joint_kp_command_interface_[j].get().get_value();
        kd_(j) = ctrl_interfaces_.joint_kd_command_interface_[j].get().get_value();
        ff_(j) = ctrl_interfaces_.joint_torque_command_interface_[j].get().get_value();
    }
    model_input_.R_OB.setIdentity(); model_input_.p_com_O.setZero(); model_input_.qdot.setZero();
    model_input_.q_j = baseline_;
    try {
        component_.robot_model_->wbc_model_->update(model_input_, model_M_, model_C_);
        baseline_feet_ = model_input_.x_f_O; reference_ready_ = true;
    } catch (...) { stop("参考站姿模型计算失败：保持原目标，不允许动作"); }
    previous_quaternion_ = Eigen::Quaterniond(
        ctrl_interfaces_.imu_state_interface_[0].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[1].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[2].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[3].get().get_value()).normalized();
    entry_rotation_ = previous_quaternion_.toRotationMatrix(); last_time_ = now();
    ctrl_interfaces_.control_inputs_.command = 0;
    if (!return_target_ready_ || !return_goal_.allFinite() ||
        (return_goal_ - baseline_).cwiseAbs().maxCoeff() > 0.03 ||
        (return_goal_.array() < component_.robot_model_->joint_lower_.array() + 0.05).any() ||
        (return_goal_.array() > component_.robot_model_->joint_upper_.array() - 0.05).any())
        stop("原FixStand返回目标无效或与当前指令相差过大：保持，不开始动作");
    else if (phase_ != 5)
        notice("已接管原FixStand目标和增益，没有跳变；a全流程，1中止保持，h慢速回中");
}
void StateComIdentification::holdOutputs() {
    for (int j = 0; j < 12; ++j) {
        ctrl_interfaces_.joint_position_command_interface_[j].get().set_value(target_(j));
        ctrl_interfaces_.joint_velocity_command_interface_[j].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[j].get().set_value(ff_(j));
        ctrl_interfaces_.joint_kp_command_interface_[j].get().set_value(kp_(j));
        ctrl_interfaces_.joint_kd_command_interface_[j].get().set_value(kd_(j));
    }
}
void StateComIdentification::stop(const char* reason) {
    phase_ = 5; status_.store(6); request_.store(0); returning_ = false;
    if (writer_started_.load()) recorder_.finish(false);
    notice(reason); // Preserve the LAST COMMAND, not measured q: avoids removing the load-bearing PD offset.
}
bool StateComIdentification::settled() const {
    // Allow a fixed load-bearing PD offset; arrival never requires zero position error.
    for (int j = 0; j < 12; ++j)
        if (std::abs(ctrl_interfaces_.joint_velocity_state_interface_[j].get().get_value()) > 0.015 ||
            std::abs(ctrl_interfaces_.joint_position_state_interface_[j].get().get_value() - target_(j)) > 0.10)
            return false;
    const auto& imu = ctrl_interfaces_.imu_state_interface_;
    return Vec3(imu[4].get().get_value(), imu[5].get().get_value(), imu[6].get().get_value()).norm() < 0.015;
}
bool StateComIdentification::solveTarget(const Eigen::Vector3d& offset, Vec12& q) {
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(offset.y(), Vec3::UnitY()) *
                                     Eigen::AngleAxisd(offset.x(), Vec3::UnitX())).toRotationMatrix();
    const Vec34 feet = rotation.transpose() * (baseline_feet_.colwise() - Vec3(0, 0, offset.z()));
    q = target_;
    for (int iteration = 0; iteration < 12; ++iteration) {
        model_input_.q_j = q;
        component_.robot_model_->wbc_model_->update(model_input_, model_M_, model_C_);
        const Vec34 error = feet - model_input_.x_f_O;
        if (error.cwiseAbs().maxCoeff() < 1e-6) return true;
        for (int leg = 0; leg < 4; ++leg) {
            const Eigen::Matrix3d J = model_input_.J_f_O[leg].block<3, 3>(0, 6 + 3 * leg);
            if (J.jacobiSvd().singularValues().minCoeff() < 0.015) return false;
            Vec3 step = (J.transpose() * J + 1e-8 * Eigen::Matrix3d::Identity()).ldlt().solve(
                J.transpose() * error.col(leg));
            if (!step.allFinite()) return false;
            const double scale = std::max(1.0, step.cwiseAbs().maxCoeff() / 0.005);
            q.segment<3>(3 * leg) += step / scale;
        }
        if (!q.allFinite() || (q - baseline_).cwiseAbs().maxCoeff() > 0.12 ||
            (q.array() < component_.robot_model_->joint_lower_.array() + 0.05).any() ||
            (q.array() > component_.robot_model_->joint_upper_.array() - 0.05).any()) return false;
    }
    return false;
}
void StateComIdentification::nextPose(double t) {
    from_ = offset_; goal_ = poseGoal(pose_, height_, angle_); phase_start_ = t; stable_since_ = -1;
    phase_ = (pose_ == 0 ? 2 : 1); status_.store(phase_ + 1); pose_status_.store(pose_);
    notice("开始下一姿态：至少8秒过渡，停稳后采10秒；1随时中止保持");
}
void StateComIdentification::record(const rclcpp::Time& time, double period, double t) {
    if (!used_ || returning_ || phase_ >= 4 || t < next_sample_) return;
    next_sample_ += 0.01;
    if (next_sample_ < t) next_sample_ = t + 0.01;
    com_identification::Sample s;
    s.steady_ns = static_cast<int64_t>(t * 1e9); s.ros_ns = time.nanoseconds();
    s.sequence = sequence_++; s.pose = pose_; s.phase = phase_; s.period = period;
    for (int j = 0; j < 12; ++j) {
        s.q[j] = ctrl_interfaces_.joint_position_state_interface_[j].get().get_value();
        s.dq[j] = ctrl_interfaces_.joint_velocity_state_interface_[j].get().get_value();
        s.tau[j] = ctrl_interfaces_.joint_effort_state_interface_[j].get().get_value();
        s.q_cmd[j] = target_(j); s.kp[j] = kp_(j); s.kd[j] = kd_(j);
    }
    for (int i = 0; i < 10; ++i) s.imu[i] = ctrl_interfaces_.imu_state_interface_[i].get().get_value();
    if (!recorder_.push(s)) stop("数据写入失败或队列满：已中止并保持");
}
void StateComIdentification::run(const rclcpp::Time& time, const rclcpp::Duration& period) {
    const double t = now(), dt = t - last_time_; last_time_ = t;
    const bool stop_requested = stop_.exchange(false);
    const int request = request_.exchange(0);
    if (stop_requested) stop("已中止：保持最后关节目标与增益，不失能；不自动回中");
    const bool action_allowed = !stop_requested && healthy() && reference_ready_ &&
        dt > 0 && dt <= 0.02 && !recorder_.failed();
    if (!stop_requested && (phase_ < 4 || phase_ >= 6 || request == 'a' || request == 'h') && !action_allowed)
        stop("反馈、键盘心跳、周期或写盘异常：已中止并保持");
    const auto& imu = ctrl_interfaces_.imu_state_interface_;
    if (imu.size() >= 4 && (phase_ < 4 || phase_ >= 6)) {
        Eigen::Quaterniond quaternion(imu[0].get().get_value(), imu[1].get().get_value(),
                                     imu[2].get().get_value(), imu[3].get().get_value());
        quaternion.normalize();
        if (previous_quaternion_.angularDistance(quaternion) > pi / 180 ||
            Eigen::Quaterniond(entry_rotation_).angularDistance(quaternion) > 5 * pi / 180)
            stop("IMU姿态跳变或偏离初始姿态过大：已中止并保持");
        previous_quaternion_ = quaternion;
    }
    if (action_allowed && request == 'a' && phase_ == 4 && !used_ && recorder_.ready()) {
        used_ = true; pose_ = 0; next_sample_ = t; nextPose(t);
    } else if (request == 'a' && !recorder_.ready() && !used_ && phase_ == 4 && !stop_requested) {
        request_.store('a'); // Wait for file-open acknowledgement without moving.
    }
    if (action_allowed && request == 'h' && (phase_ == 4 || phase_ == 5)) {
        returning_ = true; return_start_ = target_; phase_start_ = t; phase_ = 7; status_.store(7);
        notice("人工请求慢速回中；不恢复采集，不自动切换FSM");
    }
    if (request == 'a' && used_ && phase_ >= 4 && !stop_requested)
        notice("本次激活已经开始过采集，不自动重启整套；保持，或h慢速回中");
    if (phase_ == 1) {
        const double u = std::clamp((t - phase_start_) / transition_, 0.0, 1.0);
        const double blend = u * u * u * (10 + u * (-15 + 6 * u));
        const Vec3 candidate_offset = from_ + blend * (goal_ - from_);
        Vec12 candidate;
        try {
            if (!solveTarget(candidate_offset, candidate) ||
                (candidate - target_).cwiseAbs().maxCoeff() > 0.025 * dt + 1e-6) {
                stop("位置逆解、关节限位或目标速度检查失败：已中止并保持");
            } else { target_ = candidate; offset_ = candidate_offset; }
        } catch (...) { stop("质心辨识模型计算异常：已中止并保持"); }
        if (phase_ == 1 && u >= 1) {
            phase_ = 2; status_.store(3);
            phase_start_ = t; stable_since_ = -1;
        }
    } else if (phase_ == 2) {
        if (settled()) { if (stable_since_ < 0) stable_since_ = t; }
        else stable_since_ = -1;
        if (stable_since_ >= 0 && t - stable_since_ >= settling_) {
            phase_ = 3; status_.store(4); phase_start_ = t; notice("已停稳，开始当前姿态正式采集");
        } else if (t - phase_start_ > 30) stop("30秒内未持续停稳：已中止并保持");
    } else if (phase_ == 3) {
        if (!settled()) stop("正式静态采集中出现运动：整套已中止并保持");
        else if (t - phase_start_ >= sample_seconds_) {
            if (++pose_ >= pose_count) {
                pose_ = pose_count - 1; phase_ = 7; status_.store(7);
                returning_ = false; return_start_ = target_; phase_start_ = t;
                notice("采集完成：至少8秒缓慢返回原FixStand目标，保留承重增益");
            } else nextPose(t);
        }
    } else if (phase_ == 7) {
        const double u = std::clamp((t - phase_start_) / transition_, 0.0, 1.0);
        const double blend = u * u * u * (10 + u * (-15 + 6 * u));
        const Vec12 candidate = return_start_ + blend * (return_goal_ - return_start_);
        if (!candidate.allFinite() || (candidate - target_).cwiseAbs().maxCoeff() > 0.025 * dt + 1e-6 ||
            (candidate - baseline_).cwiseAbs().maxCoeff() > 0.12 ||
            (candidate.array() < component_.robot_model_->joint_lower_.array() + 0.05).any() ||
            (candidate.array() > component_.robot_model_->joint_upper_.array() - 0.05).any()) {
            stop("返回FixStand目标检查失败：中止并保持，不切模式");
        } else {
            target_ = candidate;
            if (u >= 1) {
                target_ = return_goal_; phase_ = 8; status_.store(8);
                phase_start_ = t; stable_since_ = -1;
                notice("已恢复原FixStand目标，确认静止后自动交还原FixStand模式");
            }
        }
    } else if (phase_ == 8) {
        if (settled()) { if (stable_since_ < 0) stable_since_ = t; }
        else stable_since_ = -1;
        if (stable_since_ >= 0 && t - stable_since_ >= settling_) {
            if (returning_) {
                phase_ = 4; status_.store(1); returning_ = false;
                notice("人工回中已完成并停稳：保留辨识模式保持，不恢复采集");
            } else {
                phase_ = 6; status_.store(5); recorder_.finish(true);
                notice("返回后已确认静止；关闭CSV并准备交还FixStand，全程未失能");
            }
        } else if (t - phase_start_ > 30) stop("返回FixStand后未能确认静止：保持，不自动切模式");
    } else if (phase_ == 6 && !settled()) {
        phase_ = 8; status_.store(8); stable_since_ = -1; phase_start_ = t;
    }
    holdOutputs(); record(time, period.seconds(), t);
}
void StateComIdentification::exit() {
    active_.store(false);
    if (used_ && phase_ != 6) recorder_.finish(false);
    if (phase_ == 6) {
        ctrl_interfaces_.control_inputs_.command = 0;
        notice("已交还原FixStand模式，保持承重；请用原键盘手动趴下后再断电");
    }
}
} // namespace sysu219_guide_controller
