#pragma once

#include <memory>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include "ContactObservation.hpp"

struct CtrlInterfaces;
class WaveGenerator;
class Estimator;

namespace sysu219_guide_controller {

// 独立运行的 InEKF：默认仅用于对照，可通过主控制器源码开关选择为主算法的状态来源。
// 实现、调参与 RViz 使用说明位于 src/control/ShadowInEKF.cpp。
class ShadowInEKF final {
public:
    // 可选的只读观测入口，只在该实例自己的后台线程调用。
    // 不提供 Estimator、控制接口或主回路模型的可写引用。
    struct ContactSample {
        std::int64_t stamp_ns = 0;
        std::uint64_t session = 0;
        double dt = 0.0, period = 0.0, worker_age = 0.0;
        bool initialized = false, fixed_stance_prior = false, reference_ready = false;
        Eigen::Matrix<double, 12, 1> q, qd, tau;
        Eigen::Matrix<double, 3, 4> feet_B, feet_velocity_B;
        Eigen::Matrix3d R_OB;
        Eigen::Vector3d position, velocity, gyro;
        Eigen::Vector3d reference_position, reference_velocity;
        std::array<int, 4> planned_contact{};
        std::array<double, 4> phase{};
        // 黄色实例本拍足端校正前的运动学一致性及候选位置观测噪声，仅用于诊断。
        std::array<double, 4> stationary_quality{}, foot_speed_xy{}, position_R_scale{};
        double stationary_speed_scale = 0.0, contact_noise_limit = 0.0;
    };
    class ContactObserver {
    public:
        virtual ~ContactObserver() = default;
        virtual void setRobotDescription(const std::string& xml) = 0;
        virtual void reset() = 0;
        virtual void update(const ContactSample& sample, std::array<int, 4>& contact,
                            std::array<double, 4>& phase) = 0;
        virtual void recordEstimate(const ContactSample& sample, const Eigen::Vector3d& p,
                                    const Eigen::Vector3d& v, bool ready) = 0;
        virtual void readObservation(ContactObservation& observation) const = 0;
    };
    ShadowInEKF(const std::vector<std::string>& joint_names,
                const std::vector<std::string>& foot_names,
                const std::string& base_name, const std::string& imu_name,
                std::shared_ptr<ContactObserver> contact_observer = nullptr,
                bool allow_contact_control = false);
    ~ShadowInEKF();
    ShadowInEKF(const ShadowInEKF&) = delete;
    ShadowInEKF& operator=(const ShadowInEKF&) = delete;

    void setRobotDescription(const std::string& xml);
    void startSession(const CtrlInterfaces& io);
    void stopSession() noexcept;
    void submit(const rclcpp::Time& time, const rclcpp::Duration& period,
                const CtrlInterfaces& io, const WaveGenerator& wave, const Estimator& reference,
                bool fixed_stance_prior = false, bool reference_ready = true) noexcept;
    // 实时线程调用；仅传递新系统结果，返回 false 表示新系统未就绪或自身失效。
    bool selectForControl(const rclcpp::Time& time, Estimator& estimator) noexcept;
    // 只读已接受的控制结果；不另取后台新帧，避免状态与接触来自不同时间。
    bool controlContact(const rclcpp::Time& time, ContactObservation& observation) const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // 命名空间 sysu219_guide_controller
