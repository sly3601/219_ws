#include "sysu219_guide_controller/dynamics_identification/StateLegIdentification.h"

#include <chrono>
#include <limits>
#include <iterator>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <urdf/model.h>

namespace sysu219_guide_controller {
namespace li = leg_identification;
namespace {
constexpr const char* joints[] = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"};
}

double StateLegIdentification::now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

StateLegIdentification::StateLegIdentification(
    CtrlInterfaces& interfaces, const std::vector<std::string>& names,
    const std::vector<double>& /* stand: keep the existing FSM construction interface */, double kp, double kd)
    : FSMState(FSMStateName::LEGIDENTIFICATION, "leg identification", interfaces), kp_(kp), kd_(kd) {
    if (!std::isfinite(kp_) || !std::isfinite(kd_) || kp_ <= 0.0 || kd_ <= 0.0)
        throw std::invalid_argument("辨识增益必须是有限正数");
    const auto check_order = [&names](const auto& values, const char* interface_type) {
        if (values.size() != names.size()) throw std::invalid_argument("辨识关节接口数量不一致");
        for (size_t i = 0; i < names.size(); ++i) {
            // Direct hardware: FL_hip_joint/position.
            // Chained Gazebo command: leg_pd_controller/FL_hip_joint/position.
            // In the latter, get_prefix_name() is only "leg_pd_controller".
            const std::string actual = values[i].get().get_name();
            const std::string expected = names[i] + "/" + interface_type;
            const std::string suffix = "/" + expected;
            if (actual != expected && (actual.size() < suffix.size() ||
                actual.compare(actual.size() - suffix.size(), suffix.size(), suffix) != 0))
                throw std::invalid_argument("辨识接口不匹配：期望 " + expected + "，实际 " + actual);
        }
    };
    check_order(interfaces.joint_position_state_interface_, "position");
    check_order(interfaces.joint_velocity_state_interface_, "velocity");
    check_order(interfaces.joint_effort_state_interface_, "effort");
    check_order(interfaces.joint_position_command_interface_, "position");
    check_order(interfaces.joint_velocity_command_interface_, "velocity");
    check_order(interfaces.joint_torque_command_interface_, "effort");
    check_order(interfaces.joint_kp_command_interface_, "kp");
    check_order(interfaces.joint_kd_command_interface_, "kd");
    for (size_t j = 0; j < 3; ++j) {
        const auto it = std::find(names.begin(), names.end(), joints[j]);
        if (it == names.end()) throw std::invalid_argument("辨识缺少左前腿关节");
        index_[j] = std::distance(names.begin(), it);
        center_[j] = li::identification_center[j];
    }
    auto node = ctrl_interfaces_.node;
    const std::string output_key = "leg_identification.output_directory";
    if (!node->has_parameter(output_key))
        node->declare_parameter<std::string>(output_key, "/tmp/dynamics_identification");
    recorder_ = std::make_unique<li::CsvRecorder>(node->get_parameter(output_key).as_string(), center_, kp_, kd_);
    status_pub_ = node->create_publisher<std_msgs::msg::UInt8>("/leg_identification/status", 10);
    feedback_pub_ = node->create_publisher<std_msgs::msg::String>(
        "/leg_identification/feedback", rclcpp::QoS(64).transient_local());
    status_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this] {
        std_msgs::msg::UInt8 msg;
        msg.data = active_.load() ? status_.load() : 0;
        status_pub_->publish(msg);
        publishFeedback();
    });
    command_sub_ = node->create_subscription<std_msgs::msg::String>(
        "/leg_identification/command", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "heartbeat") { keyboard_time_.store(now()); return; }
            if (msg->data == "enter" && !active_.load()) { entry_requested_.store(true); return; }
            if (!active_.load() || msg->data.size() != 1) return;
            const char c = msg->data[0];
            if (c == '1') { requestStop(); return; }
            if (std::string("qwertyuah").find(c) != std::string::npos) request_.store(c);
        });
    emergency_sub_ = node->create_subscription<control_input_msgs::msg::Inputs>(
        "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg) {
            if (active_.load() && msg->command == 1) requestStop();
        });
    description_sub_ = node->create_subscription<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == description_) return;
            if (active_.load() && !description_.empty()) { model_ready_.store(false); return; }
            urdf::Model model;
            if (!model.initString(msg->data)) { model_ready_.store(false); return; }
            model_ready_.store(false);
            for (size_t j = 0; j < 3; ++j) {
                const auto joint = model.getJoint(joints[j]);
                if (!joint || !joint->limits || !std::isfinite(joint->limits->lower) ||
                    !std::isfinite(joint->limits->upper) || joint->limits->lower >= joint->limits->upper) return;
                urdf_lower_[j].store(joint->limits->lower);
                urdf_upper_[j].store(joint->limits->upper);
            }
            description_ = msg->data;
            recorder_->setDescription(description_);
            model_ready_.store(true);
        });
}

bool StateLegIdentification::feedbackHealthy() const {
    if (!model_ready_.load()) return false;
    for (size_t j = 0; j < 3; ++j) {
        const size_t i = index_[j];
        if (!std::isfinite(ctrl_interfaces_.joint_position_state_interface_[i].get().get_value()) ||
            !std::isfinite(ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value()) ||
            !std::isfinite(ctrl_interfaces_.joint_effort_state_interface_[i].get().get_value())) return false;
    }
    return true;
}

StateLegIdentification::Bounds StateLegIdentification::bounds(MotionRange range, bool with_margin) const {
    Bounds result;
    for (size_t j = 0; j < 3; ++j) {
        const double m = with_margin ? li::margin : 0.0;
        result.lower[j] = std::max(li::hardware_lower[j], urdf_lower_[j].load()) + m;
        result.upper[j] = std::min(li::hardware_upper[j], urdf_upper_[j].load()) - m;
        if (j == 0 && range == MotionRange::Excitation) {
            result.lower[j] = std::max(result.lower[j], li::hip_lower);
            result.upper[j] = std::min(result.upper[j], li::hip_upper);
        }
    }
    return result;
}

bool StateLegIdentification::safe(const Target& target, MotionRange range, bool with_margin) const {
    const auto limits = bounds(range, with_margin);
    for (size_t j = 0; j < 3; ++j) {
        if (!std::isfinite(target.q[j]) || !std::isfinite(target.dq[j]) || !std::isfinite(target.ddq[j]) ||
            target.q[j] < limits.lower[j] || target.q[j] > limits.upper[j] ||
            std::abs(target.dq[j]) > li::velocity_limit[j] ||
            std::abs(target.ddq[j]) > li::acceleration_limit[j]) return false;
    }
    return true;
}

li::Vec StateLegIdentification::startPose() const {
    return action_ == Action::Static ? li::staticPose(center_, trial_) : center_;
}

bool StateLegIdentification::arrived() const {
    for (size_t j = 0; j < 3; ++j) {
        const size_t i = index_[j];
        const double q = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        const double dq = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        if (std::abs(target_.q[j] - q) > li::ready_position_error[j] ||
            std::abs(dq) > li::ready_velocity[j]) return false;
    }
    return true;
}

void StateLegIdentification::zeroOutputs(bool include_left_front) {
    for (size_t i = 0; i < ctrl_interfaces_.joint_kp_command_interface_.size(); ++i) {
        if (!include_left_front && std::find(index_.begin(), index_.end(), i) != index_.end()) continue;
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(0.0);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(0.0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
        const double q = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        if (std::isfinite(q)) ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q);
    }
}

void StateLegIdentification::pushFeedback(const Feedback& feedback) {
    const size_t head = feedback_head_.load(std::memory_order_relaxed);
    const size_t next = (head + 1) % feedback_capacity;
    if (next == feedback_tail_.load(std::memory_order_acquire)) {
        feedback_overflow_.store(true);
        return;  // Dropping a terminal notice never changes control or data acquisition.
    }
    feedback_queue_[head] = feedback;
    feedback_head_.store(next, std::memory_order_release);
}

void StateLegIdentification::notify(const char* text, bool warning, const Target* invalid, bool with_margin,
                                    MotionRange range) {
    Feedback feedback;
    feedback.text = text; feedback.warning = warning;
    feedback.action = action_; feedback.phase = phase_; feedback.trial = trial_;
    if (invalid) {
        feedback.has_bounds = true; feedback.q = invalid->q;
        const auto limits = bounds(range, with_margin);
        feedback.lower = limits.lower; feedback.upper = limits.upper;
    }
    pushFeedback(feedback);
}

void StateLegIdentification::progress(Notice kind) {
    Feedback feedback;
    feedback.kind = kind; feedback.action = action_; feedback.phase = phase_;
    feedback.trial = trial_; feedback.suite = suite_; feedback.q = startPose();
    feedback.first = experiment_first_;
    feedback.has_samples = sample_sequence_ > experiment_first_;
    feedback.last = feedback.has_samples ? sample_sequence_ - 1 : experiment_first_;
    feedback.warning = kind == Notice::Interrupted || kind == Notice::Waiting;
    if (kind == Notice::Waiting) {
        for (size_t j = 0; j < 3; ++j) {
            const size_t i = index_[j];
            feedback.q[j] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value() - target_.q[j];
            feedback.velocity[j] = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        }
    }
    pushFeedback(feedback);
}

void StateLegIdentification::publishFeedback() {
    const auto publish = [this](const std::string& text, bool warning) {
        std_msgs::msg::String msg;
        msg.data = std::string(warning ? "W\n" : "I\n") + text;
        feedback_pub_->publish(msg);
    };
    const auto notice = recorder_->takeNotice();
    if (!notice.empty()) publish(notice, recorder_->failed());
    for (;;) {
        const size_t tail = feedback_tail_.load(std::memory_order_relaxed);
        if (tail == feedback_head_.load(std::memory_order_acquire)) break;
        const Feedback feedback = feedback_queue_[tail];
        feedback_tail_.store((tail + 1) % feedback_capacity, std::memory_order_release);
        std::ostringstream out;
        if (feedback.kind == Notice::Text) out << feedback.text;
        else {
            constexpr const char* names[] = {"左前腿归位", "静态姿态", "髋关节", "大腿关节", "小腿关节", "联合多频", "独立验证"};
            std::ostringstream label;
            label << names[static_cast<int>(feedback.action)];
            if (feedback.action != Action::Hold)
                label << "测试（第" << feedback.trial + 1 << '/' << li::repetitions(feedback.action) << "次）";
            out << label.str();
            switch (feedback.kind) {
                case Notice::Preparing:
                    out << "：开始准备，从当前位置缓慢移动到测试起点 ["
                        << feedback.q[0] << ", " << feedback.q[1] << ", " << feedback.q[2] << "] rad。";
                    break;
                case Notice::Prepared:
                    out << "：准备完成，实测到位和低速条件已连续满足 " << li::ready_hold_seconds << " 秒。";
                    if (feedback.action == Action::Hold) out << "保持中心姿态；选择 w/e/r/t/y/u/a 开始测试。";
                    break;
                case Notice::Started:
                    out << "：正式开始，动作段 " << li::duration(feedback.action)
                        << " 秒，正在记录250 Hz源数据。";
                    break;
                case Notice::Completed: case Notice::Interrupted:
                    out << (feedback.kind == Notice::Completed ? "：本次测试完成。" : "：测试已中止，本段不标记为完成。");
                    if (feedback.kind == Notice::Completed && feedback.trial + 1 == li::repetitions(feedback.action)) {
                        out << names[static_cast<int>(feedback.action)] << "测试全部完成。";
                        if (feedback.suite && feedback.action == Action::Validation) out << "全套测试完成，缓慢返回中心保持。";
                    }
                    if (feedback.has_samples) {
                        out << "正式段样本序号 [" << feedback.first << ", " << feedback.last << "]，等待CSV写入确认。";
                        recorder_->checkpoint(label.str() + (feedback.kind == Notice::Interrupted ? "，中止" : "，完成"),
                                              feedback.first, feedback.last);
                    } else out << "本段尚无正式样本。";
                    break;
                case Notice::Waiting:
                    out << "：正在等待实测到位，尚未开始正式测试。位置误差 ["
                        << feedback.q[0] << ", " << feedback.q[1] << ", " << feedback.q[2]
                        << "] rad；速度 [" << feedback.velocity[0] << ", " << feedback.velocity[1]
                        << ", " << feedback.velocity[2] << "] rad/s。1 可急停。";
                    break;
                default: break;
            }
        }
        if (feedback.has_bounds) {
            constexpr const char* names[] = {"髋", "大腿", "小腿"};
            out << std::fixed << std::setprecision(8);
            for (size_t j = 0; j < 3; ++j) {
                const bool outside = !std::isfinite(feedback.q[j]) || feedback.q[j] < feedback.lower[j] || feedback.q[j] > feedback.upper[j];
                out << '\n' << "  左前" << names[j] << "：" << feedback.q[j]
                    << " rad；允许 [" << feedback.lower[j] << ", " << feedback.upper[j] << "] rad"
                    << (outside ? "，越界" : "，范围内");
            }
            out << "\n未开始归位；上述范围为本次检查所用范围。";
        }
        if (feedback.warning && feedback.phase == Phase::Holding)
            out << "\n当前测试/全套流程已中止；左前腿保持触发时的位置和增益，不失能、不自动恢复。按 q 归位或重新选择测试。";
        else if (feedback.warning && feedback.phase == Phase::Stopped)
            out << "\n仍在辨识状态，尚未开始驱动；处理原因后按 q 或重新选择测试。";
        publish(out.str(), feedback.warning);
    }
    if (feedback_overflow_.exchange(false))
        publish("终端提示积压，部分提示未显示；控制状态和源数据采集不受提示队列影响。", true);
}

void StateLegIdentification::stop(const char* reason, const Target* invalid, bool with_margin, MotionRange range) {
    if (phase_ == Phase::Experiment) progress(Notice::Interrupted);
    if (phase_ != Phase::Holding && gain_ > 0.0) {
        target_ = Target{};
        for (size_t j = 0; j < 3; ++j) {
            const size_t i = index_[j];
            const double q = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            const double previous = ctrl_interfaces_.joint_position_command_interface_[i].get().get_value();
            target_.q[j] = std::isfinite(q) ? q : (std::isfinite(previous) ? previous : center_[j]);
        }
    }
    phase_ = gain_ > 0.0 ? Phase::Holding : Phase::Stopped;
    action_ = Action::Hold; suite_ = false; motion_range_ = MotionRange::Preparation;
    phase_start_ = now(); request_.store(0); status_.store(phase_ == Phase::Holding ? 5 : 1);
    notify(reason, true, invalid, with_margin, range);
}

void StateLegIdentification::enter() {
    zeroOutputs();
    phase_ = Phase::Stopped; action_ = Action::Hold; suite_ = false;
    request_.store(0); stop_requested_.store(false); status_.store(1); active_.store(true);
    gain_ = gain_from_ = 0.0;
    phase_start_ = now(); next_sample_ = 0.0; last_sample_ = 0.0;
    sample_sequence_ = 0;
    recorder_->begin();
    ctrl_interfaces_.control_inputs_.command = 0;
    notify("已进入腿部参数辨识，仅左前腿；当前输出清零。选择 w/e/r/t/y/u/a 后自动先准备再测试；q 仅归位，1 急停。髋测试只在0～37°耸肩侧往复，0为准备/回中位置。");
}

void StateLegIdentification::exit() {
    if (gain_ > 0.0) {
        stop("辨识控制器退出，保留左前腿位置保持指令");
        for (size_t j = 0; j < 3; ++j) {
            const size_t i = index_[j];
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(target_.q[j]);
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
            ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
        }
    }
    zeroOutputs(false); active_.store(false); status_.store(0);
    recorder_->finish();
}

void StateLegIdentification::transferTo(const li::Vec& goal, double t) {
    phase_ = Phase::Transition;
    // A long polynomial can overshoot when interrupting a moving sinusoid.
    // Prefer the slowest safe C2 transfer; preserve the same bounds for every candidate.
    for (double seconds : {li::transition_seconds, 2.0, 1.0}) {
        transfer_.reset(target_, goal, seconds);
        bool valid = true;
        for (double s = 0.0; s <= seconds + 1e-9; s += li::sample_period) {
            if (!safe(transfer_.at(s), motion_range_)) { valid = false; break; }
        }
        if (!valid) continue;
        phase_ = Phase::Transition; phase_start_ = t; ready_since_ = -1.0; status_.store(3);
        return;
    }
    stop("没有满足范围/速度/加速度保护的连续过渡，拒绝执行");
}

void StateLegIdentification::startAction(Action action, double t) {
    if (phase_ == Phase::Experiment) progress(Notice::Interrupted);
    action_ = action; trial_ = 0;
    transferTo(startPose(), t);
    if (phase_ == Phase::Transition) progress(Notice::Preparing);
}

void StateLegIdentification::beginPreparation(Action action, bool suite, double t) {
    if (!feedbackHealthy() || !recorder_->ready() || t - keyboard_time_.load() > li::keyboard_timeout) {
        if (!model_ready_.load()) notify("准备未执行：尚无有效 URDF，请等待机器人描述就绪。", true);
        else if (!feedbackHealthy()) notify("准备未执行：左前腿位置、速度或力矩反馈不是有限数值。", true);
        else if (!recorder_->ready()) notify("准备未执行：CSV记录器尚未就绪或已故障。", true);
        else notify("准备未执行：专用键盘心跳已超过2秒。", true);
        return;
    }
    Target initial;
    const auto limits = bounds(MotionRange::Preparation);
    for (size_t j = 0; j < 3; ++j)
        initial.q[j] = ctrl_interfaces_.joint_position_state_interface_[index_[j]].get().get_value();
    for (size_t j = 0; j < 3; ++j) {
        if (limits.lower[j] >= limits.upper[j] ||
            initial.q[j] < limits.lower[j] - li::position_fault_margin ||
            initial.q[j] > limits.upper[j] + li::position_fault_margin) {
            stop("实测角度明显超出既有硬件/URDF限位（超过约20度），请检查反馈或机械状态。",
                 &initial, false, MotionRange::Preparation);
            return;
        }
    }
    // Project preparation commands into the existing limits; keep raw feedback unchanged.
    bool projected = false;
    for (size_t j = 0; j < 3; ++j) {
        const double q = std::clamp(initial.q[j], limits.lower[j], limits.upper[j]);
        projected = projected || q != initial.q[j];
        initial.q[j] = q;
    }
    for (int sign : {-1, 1}) {
        Target edge; edge.q = center_;
        for (size_t j = 0; j < 3; ++j) {
            if (j == 0) edge.q[j] = sign < 0 ? li::hip_lower : li::hip_upper;
            else edge.q[j] += sign * li::amplitudes[j];
        }
        if (!safe(edge, MotionRange::Excitation, true)) {
            stop("计划激励边界超出允许范围；这项检查针对测试目标，不要求初始姿态已在起点。", &edge, true);
            return;
        }
    }
    target_ = initial; gain_from_ = gain_;
    action_ = action; trial_ = 0; suite_ = suite;
    motion_range_ = MotionRange::Preparation;
    phase_ = Phase::GainRamp; phase_start_ = t; ready_since_ = -1.0; status_.store(3);
    progress(Notice::Preparing);
    notify("先在当前位置渐入增益1.5秒，再缓慢归位；初次归位使用既有关节限位。到位后开始测试；正式髋轨迹不进入负角度溜肩侧。");
    if (projected) notify("初始反馈超出指令限位，归位起始目标已投影到既有限位内；原始反馈仍原样记录。");
}

void StateLegIdentification::nextExperiment(double t) {
    progress(Notice::Completed);
    ++trial_;
    if (trial_ >= li::repetitions(action_)) {
        trial_ = 0;
        if (suite_ && action_ != Action::Validation) action_ = static_cast<Action>(static_cast<int>(action_) + 1);
        else { action_ = Action::Hold; suite_ = false; }
    }
    transferTo(startPose(), t);
    if (phase_ == Phase::Transition) progress(Notice::Preparing);
}

void StateLegIdentification::run(const rclcpp::Time& time, const rclcpp::Duration& period) {
    const double t = now();
    const bool emergency = stop_requested_.exchange(false) || ctrl_interfaces_.control_inputs_.command == 1;
    ctrl_interfaces_.control_inputs_.command = 0;  // Ordinary FSM commands cannot leave this state.
    if (emergency) stop("按键1急停");
    if (phase_ != Phase::Stopped && phase_ != Phase::Holding &&
        (!feedbackHealthy() || !recorder_->ready() || t - keyboard_time_.load() > li::keyboard_timeout))
        stop("反馈无效、记录器故障或专用键盘离线超过2秒，测试中止");

    const int command = emergency ? 0 : request_.exchange(0);
    if (command && phase_ == Phase::GainRamp) {
        notify("当前正在渐入增益，请等待准备继续；1 仍可急停。", true);
    } else if (command) {
        Action selected = Action::Hold;
        switch (command) {
            case 'w': case 'a': selected = Action::Static; break;
            case 'e': selected = Action::Hip; break;
            case 'r': selected = Action::Thigh; break;
            case 't': selected = Action::Calf; break;
            case 'y': selected = Action::Combined; break;
            case 'u': selected = Action::Validation; break;
            default: break;  // q/h both return smoothly to the center.
        }
        if (phase_ == Phase::Stopped || phase_ == Phase::Holding) {
            if (phase_ == Phase::Stopped && command == 'h') notify("当前尚未驱动；按 q 单独归位，或选测试键自动准备并开始测试。");
            else beginPreparation(selected, command == 'a', t);
        } else {
            startAction(selected, t);
            suite_ = (phase_ == Phase::Stopped || phase_ == Phase::Holding) ? false : command == 'a';
        }
    }

    zeroOutputs(false);  // Never clear left-front gains as an intermediate/default action.
    bool experiment_complete = false;
    if (phase_ != Phase::Stopped) {
        const double elapsed = t - phase_start_;
        double gain = gain_;
        if (phase_ == Phase::GainRamp) {
            gain = gain_from_ + (1.0 - gain_from_) * li::smooth(elapsed, li::gain_seconds)[0];
            if (elapsed >= li::gain_seconds) transferTo(startPose(), t);
        } else if (phase_ == Phase::Transition) {
            target_ = transfer_.at(elapsed);
            if (elapsed >= transfer_.duration()) {
                target_ = Target{}; target_.q = startPose();
                motion_range_ = MotionRange::Excitation;
                phase_ = Phase::Settling; phase_start_ = t;
                ready_since_ = -1.0; next_wait_notice_ = t + 2.0;
            }
        } else if (phase_ == Phase::Settling) {
            if (arrived()) {
                if (ready_since_ < 0.0) ready_since_ = t;
                if (t - ready_since_ >= li::ready_hold_seconds) {
                    progress(Notice::Prepared);
                    phase_ = action_ == Action::Hold ? Phase::Ready : Phase::Experiment;
                    phase_start_ = t; status_.store(phase_ == Phase::Ready ? 2 : 4);
                    if (phase_ == Phase::Experiment) {
                        experiment_first_ = sample_sequence_;
                        progress(Notice::Started);
                    }
                }
            } else ready_since_ = -1.0;
            if (phase_ == Phase::Settling && t >= next_wait_notice_) {
                progress(Notice::Waiting); next_wait_notice_ = t + 2.0;
            }
        } else if (phase_ == Phase::Experiment) {
            target_ = li::excitation(center_, action_, trial_, std::min(elapsed, li::duration(action_)));
            experiment_complete = elapsed >= li::duration(action_);
        }
        if (phase_ != Phase::Holding) {
            const auto limits = bounds(MotionRange::Preparation);
            bool valid = true;
            for (size_t j = 0; j < 3; ++j) {
                const size_t i = index_[j];
                const double q = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
                const double dq = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
                valid = valid && std::isfinite(target_.q[j]) && std::isfinite(target_.dq[j]) &&
                        std::isfinite(target_.ddq[j]) && std::isfinite(q) && std::isfinite(dq) &&
                        q >= limits.lower[j] - li::position_fault_margin &&
                        q <= limits.upper[j] + li::position_fault_margin &&
                        std::abs(dq) <= li::measured_velocity_limit;
            }
            if (!valid) stop("角度明显越界、角速度异常或反馈/目标无效，测试中止并保持当前位置");
        }
        if (phase_ != Phase::Stopped) {
            if (phase_ != Phase::Holding) gain_ = gain;
            for (size_t j = 0; j < 3; ++j) {
                const size_t i = index_[j];
                ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(target_.q[j]);
                ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(target_.dq[j]);
                ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_*gain_);
                ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_*gain_);
                ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
            }
        }
    }
    recordSample(time, period, t);
    // The last formal sample keeps the completed action/trial before advancing the FSM.
    if (experiment_complete && phase_ == Phase::Experiment) nextExperiment(t);
}

void StateLegIdentification::recordSample(const rclcpp::Time& time, const rclcpp::Duration& period, double t) {
    if (t + 1e-9 < next_sample_) return;
    if (next_sample_ == 0.0) next_sample_ = t;
    next_sample_ += (1.0 + std::floor(std::max(0.0, t - next_sample_) / li::sample_period)) * li::sample_period;
    li::Sample sample;
    sample.sequence = sample_sequence_++;
    sample.steady_ns = static_cast<int64_t>(t * 1e9);
    sample.system_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    sample.ros_ns = time.nanoseconds(); sample.period_s = period.seconds();
    sample.action = static_cast<int>(action_); sample.phase = static_cast<int>(phase_);
    sample.trial = trial_; sample.phase_time_s = std::max(0.0, t - phase_start_);
    if (phase_ == Phase::Stopped || phase_ == Phase::Holding) sample.flags |= 1;
    if (last_sample_ > 0.0 && (t - last_sample_ < 0.002 || t - last_sample_ > 0.006)) sample.flags |= 8;
    for (size_t j = 0; j < 3; ++j) {
        const size_t i = index_[j];
        sample.q[j] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        sample.dq[j] = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        sample.tau[j] = ctrl_interfaces_.joint_effort_state_interface_[i].get().get_value();
        if (!std::isfinite(sample.q[j]) || !std::isfinite(sample.dq[j]) || !std::isfinite(sample.tau[j])) sample.flags |= 2;
        sample.q_cmd[j] = ctrl_interfaces_.joint_position_command_interface_[i].get().get_value();
        sample.dq_cmd[j] = ctrl_interfaces_.joint_velocity_command_interface_[i].get().get_value();
        sample.tau_ff[j] = ctrl_interfaces_.joint_torque_command_interface_[i].get().get_value();
        sample.kp[j] = ctrl_interfaces_.joint_kp_command_interface_[i].get().get_value();
        sample.kd[j] = ctrl_interfaces_.joint_kd_command_interface_[i].get().get_value();
    }
    for (size_t i = 0; i < sample.imu.size(); ++i) {
        sample.imu[i] = i < ctrl_interfaces_.imu_state_interface_.size()
            ? ctrl_interfaces_.imu_state_interface_[i].get().get_value() : std::numeric_limits<double>::quiet_NaN();
        if (!std::isfinite(sample.imu[i])) sample.flags |= 4;
    }
    last_sample_ = t;
    if (!recorder_->push(sample) && recorder_->failed() && phase_ != Phase::Stopped && phase_ != Phase::Holding)
        stop("CSV队列满或文件写入失败");
}
}  // namespace sysu219_guide_controller
