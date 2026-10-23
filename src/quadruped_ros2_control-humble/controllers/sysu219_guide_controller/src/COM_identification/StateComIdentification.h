#pragma once

#include <controller_common/FSM/FSMState.h>
#include <Eigen/Geometry>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "sysu219_guide_controller/control/CtrlComponent.h"
#include "ComRecorder.h"

namespace sysu219_guide_controller {
class StateComIdentification final : public FSMState {
public:
    StateComIdentification(CtrlInterfaces& interfaces, CtrlComponent& component,
                           const std::vector<std::string>& joints, const std::vector<double>& stand,
                           const std::string& base, const std::vector<std::string>& feet);
    bool consumeEntryRequest(bool from_fixed_stand);
    void setReturnTarget(const std::array<double, 12>& q);
    void enter() override;
    void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void exit() override;
    FSMStateName checkChange() override;
private:
    static double now();
    bool healthy(bool entry = false) const;
    bool settled() const;
    bool solveTarget(const Eigen::Vector3d& offset, Vec12& q);
    void holdOutputs();
    void stop(const char* reason);
    void notice(const char* text) { notice_.store(text); }
    void record(const rclcpp::Time& time, double period, double t);
    void nextPose(double t);

    CtrlComponent& component_;
    const std::vector<double> stand_;
    const std::string base_;
    const std::vector<std::string> feet_;
    double transition_ = 8.0, settling_ = 1.0, sample_seconds_ = 10.0;
    double height_ = 0.005, angle_ = 0.03490658503988659;
    std::string root_, description_;
    std::atomic<bool> model_ready_{false}, entry_{false}, active_{false}, stop_{false};
    std::atomic<bool> writer_started_{false};
    std::atomic<int> request_{0}, status_{0}, pose_status_{0};
    std::atomic<double> heartbeat_{0};
    std::atomic<const char*> notice_{nullptr};
    bool used_ = false, returning_ = false, reference_ready_ = false;
    bool return_target_ready_ = false;
    int pose_ = 0, phase_ = 0; // 1 move, 2 settle, 3 sample, 4 hold, 5 stopped, 6 ready to exit, 7 return, 8 return settling
    double phase_start_ = 0, stable_since_ = -1, next_sample_ = 0, last_time_ = 0;
    uint64_t sequence_ = 0;
    Vec12 baseline_, target_, kp_, kd_, ff_;
    Vec12 return_goal_, return_start_;
    Vec34 baseline_feet_;
    Eigen::Vector3d offset_ = Eigen::Vector3d::Zero(), from_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_ = Eigen::Vector3d::Zero(); // roll, pitch, body height delta
    Eigen::Matrix3d entry_rotation_ = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond previous_quaternion_ = Eigen::Quaterniond::Identity();
    sysu219::wbc::WbcInput model_input_;
    sysu219::wbc::Mat18 model_M_;
    sysu219::wbc::Vec18 model_C_;
    com_identification::Recorder recorder_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_, description_sub_;
    rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr emergency_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_, directory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace sysu219_guide_controller
