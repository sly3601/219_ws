#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

struct CtrlInterfaces;
class WaveGenerator;
class Estimator;

namespace sysu219_guide_controller {

// 独立运行的 InEKF：默认仅用于对照，可通过主控制器源码开关选择为主算法的状态来源。
// 实现、调参与 RViz 使用说明位于 src/control/ShadowInEKF.cpp。
class ShadowInEKF final {
public:
    ShadowInEKF(const std::vector<std::string>& joint_names,
                const std::vector<std::string>& foot_names,
                const std::string& base_name, const std::string& imu_name);
    ~ShadowInEKF();
    ShadowInEKF(const ShadowInEKF&) = delete;
    ShadowInEKF& operator=(const ShadowInEKF&) = delete;

    void setRobotDescription(const std::string& xml);
    void startSession(const CtrlInterfaces& io);
    void stopSession() noexcept;
    void submit(const rclcpp::Time& time, const rclcpp::Duration& period,
                const CtrlInterfaces& io, const WaveGenerator& wave, Estimator& legacy,
                bool fixed_stance_prior = false) noexcept;
    // 实时线程调用；仅传递新系统结果，返回 false 表示新系统未就绪或自身失效。
    bool selectForControl(const rclcpp::Time& time, Estimator& estimator) noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // 命名空间 sysu219_guide_controller
