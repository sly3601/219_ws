#pragma once

#include <memory>
#include <array>
#include <cstdint>
#include <limits>
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
    // 只读控制线程本次实际选择的来源与外推结果；valid=false时数值状态不可用。
    // source_stamp_s是后台输入采样的控制时钟，source_age_s使用稳态时钟计算。
    struct ControlDebug {
        bool valid = false, new_source_sample = false, reused_trusted_result = false;
        double source_stamp_s = std::numeric_limits<double>::quiet_NaN();
        double source_age_s = std::numeric_limits<double>::quiet_NaN();
        double projection_dt_s = std::numeric_limits<double>::quiet_NaN();
        std::uint64_t source_epoch = 0, source_session = 0, output_rejections = 0;
        // selection_reason：本次选择的拒绝/不可用位；成功沿用旧结果时也保留本次拒绝原因。
        // 1时序/会话，2年龄/数值，4位置，8速度，16姿态，32角速度，64后台不可用，
        // 128来源超时，256未启用控制，512保护已锁存，1024来源epoch改变。
        // last_rejection_reason保留既有输出保护的最近拒绝位，不冒充本拍新异常。
        unsigned last_rejection_reason = 0, selection_reason = 0;
        // position/velocity在世界系，rotation为B->世界，gyro/bias在机身系。
        Eigen::Vector3d source_position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Vector3d source_velocity = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Matrix3d source_rotation = Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Vector3d source_gyro = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Vector3d output_position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Vector3d output_velocity = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Matrix3d output_rotation = Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Vector3d output_gyro = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Vector3d gyro_bias = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Vector3d accel_bias = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        // 后台本来源帧的IMU数值未变化时长，不是硬件采样年龄或独立的新鲜度证明。
        double source_imu_unchanged_s = std::numeric_limits<double>::quiet_NaN();
        bool source_imu_stale = false;
        // 最近一次姿态检查的残差与累计计数；本来源帧未检查时残差不会刷新。
        double source_attitude_residual_rad = std::numeric_limits<double>::quiet_NaN();
        std::uint64_t source_attitude_accepted = 0, source_attitude_rejected = 0;
        // 滤波器最近姿态检查（初始化也设置）的控制时钟，不是AHRS硬件接收时间。
        double source_attitude_update_stamp_s = std::numeric_limits<double>::quiet_NaN();
        bool source_attitude_update_attempted = false, source_attitude_update_accepted = false;
        bool source_attitude_reacquiring = false;
    };
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
    // 仅控制线程读取；不消费后台结果队列，不改变来源选择或外推状态。
    ControlDebug debugControl() const noexcept;
    // 只读已接受的控制结果；不另取后台新帧，避免状态与接触来自不同时间。
    bool controlContact(const rclcpp::Time& time, ContactObservation& observation) const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // 命名空间 sysu219_guide_controller
