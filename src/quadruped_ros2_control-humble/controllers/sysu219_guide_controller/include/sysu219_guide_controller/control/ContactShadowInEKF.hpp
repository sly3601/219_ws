#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include "ContactObservation.hpp"

struct CtrlInterfaces;
class WaveGenerator;
class Estimator;

namespace sysu219_guide_controller {

// 黄色接触 InEKF。状态/模型/线程/日志独立；默认影子运行，配置明确选中后才允许控制读取。
class ContactShadowInEKF final {
public:
    struct Settings {
        // 仅用于旁路采集的待标定起点，绝非已验证的接触阈值。
        double force_on_N = 25.0;
        double force_off_N = 12.0;
        double confirm_on_s = 0.012;
        double confirm_off_s = 0.020;
        bool write_csv = true;
        bool serve_odometry = false; // 只有启动时选中本实例才允许输出控制状态。
    };
    ContactShadowInEKF(const std::vector<std::string>& joint_names,
                      const std::vector<std::string>& foot_names,
                      const std::string& base_name, const std::string& imu_name,
                      const Settings& settings);
    ~ContactShadowInEKF();
    ContactShadowInEKF(const ContactShadowInEKF&) = delete;
    ContactShadowInEKF& operator=(const ContactShadowInEKF&) = delete;
    void setRobotDescription(const std::string& xml);
    void startSession(const CtrlInterfaces& io);
    void stopSession() noexcept;
    void submit(const rclcpp::Time& time, const rclcpp::Duration& period,
                const CtrlInterfaces& io, const WaveGenerator& wave, const Estimator& reference,
                bool fixed_stance_prior, bool reference_ready) noexcept;
    bool selectForControl(const rclcpp::Time& time, Estimator& estimator) noexcept;
    bool controlContact(const rclcpp::Time& time, ContactObservation& observation) const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sysu219_guide_controller
