#pragma once

#include <atomic>
#include <memory>
#include <controller_common/FSM/FSMState.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "Trajectory.h"
#include "CsvRecorder.h"

namespace sysu219_guide_controller {
class StateLegIdentification final : public FSMState {
public:
    StateLegIdentification(CtrlInterfaces& interfaces, const std::vector<std::string>& joints,
                           const std::vector<double>& stand, double kp, double kd);
    void enter() override;
    void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void exit() override;
    FSMStateName checkChange() override { return FSMStateName::LEGIDENTIFICATION; }
    void requestStop() { stop_requested_.store(true); }
    bool consumeEntryRequest() { return entry_requested_.exchange(false); }

private:
    using Target = leg_identification::Target;
    using Action = leg_identification::Action;
    using Phase = leg_identification::Phase;
    using MotionRange = leg_identification::MotionRange;
    enum class Notice { Text, Preparing, Prepared, Started, Completed, Interrupted, Waiting };
    struct Bounds { leg_identification::Vec lower{}, upper{}; };
    static double now();
    void zeroOutputs(bool include_left_front = true);
    void stop(const char* reason, const Target* invalid = nullptr, bool margin = false,
              MotionRange range = MotionRange::Excitation);
    void notify(const char* text, bool warning = false, const Target* invalid = nullptr, bool margin = false,
                MotionRange range = MotionRange::Excitation);
    void progress(Notice kind);
    struct Feedback;
    void pushFeedback(const Feedback& feedback);
    void publishFeedback();
    bool feedbackHealthy() const;
    Bounds bounds(MotionRange range, bool margin = false) const;
    bool safe(const Target& target, MotionRange range, bool margin = false) const;
    bool arrived() const;
    leg_identification::Vec startPose() const;
    void beginPreparation(Action action, bool suite, double t);
    void transferTo(const leg_identification::Vec& goal, double t);
    void startAction(Action action, double t);
    void nextExperiment(double t);
    void recordSample(const rclcpp::Time& time, const rclcpp::Duration& period, double t);

    std::array<size_t, 3> index_{};
    leg_identification::Vec center_{};
    double kp_, kd_;
    std::array<std::atomic<double>, 3> urdf_lower_{}, urdf_upper_{};
    std::atomic<bool> model_ready_{false}, active_{false}, entry_requested_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<int> request_{0}, status_{0};
    std::atomic<double> keyboard_time_{0.0};
    Phase phase_ = Phase::Stopped;
    Action action_ = Action::Hold;
    Target target_{};
    leg_identification::Transfer transfer_;
    double gain_ = 0.0, gain_from_ = 0.0;
    double phase_start_ = 0.0, next_sample_ = 0.0, last_sample_ = 0.0;
    uint64_t sample_sequence_ = 0;
    std::string description_;
    std::unique_ptr<leg_identification::CsvRecorder> recorder_;
    int trial_ = 0;
    bool suite_ = false;
    MotionRange motion_range_ = MotionRange::Excitation;
    double ready_since_ = -1.0, next_wait_notice_ = 0.0;
    uint64_t experiment_first_ = 0;

    // FSM producer -> non-real-time timer consumer; no formatting or ROS logging in run().
    struct Feedback {
        const char* text = nullptr;  // Static string literals, formatted only by the timer.
        Notice kind = Notice::Text;
        bool warning = false, has_bounds = false, has_samples = false;
        leg_identification::Vec q{}, lower{}, upper{}, velocity{};
        Action action = Action::Hold;
        Phase phase = Phase::Stopped;
        int trial = 0;
        uint64_t first = 0, last = 0;
        bool suite = false;
    };
    static constexpr size_t feedback_capacity = 64;
    std::array<Feedback, feedback_capacity> feedback_queue_{};
    std::atomic<size_t> feedback_head_{0}, feedback_tail_{0};
    std::atomic<bool> feedback_overflow_{false};

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_, description_sub_;
    rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr emergency_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace sysu219_guide_controller
