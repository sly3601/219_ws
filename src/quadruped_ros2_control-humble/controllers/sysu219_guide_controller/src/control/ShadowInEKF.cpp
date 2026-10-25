/*
 * SYSU219 独立影子里程计：算法实现及调参集中在本文件，不增加第三方依赖。
 * 公开接口位于 include/sysu219_guide_controller/control/ShadowInEKF.hpp。
 *
 * 运行与切换：
 * submit() 只向有界单生产者单消费者队列复制定长传感器快照。
 * 后台线程独占运动学求解器、滤波状态、发布器和显示历史，不读取主算法的
 * 活动机器人模型，不调用状态机，不写控制命令，也不修改原 KF 的内部状态。
 * 队列满时丢弃当前快照，控制线程不等待后台线程。
 * selectForControl() 通过另一条有界队列读取完整结果，由 Estimator 统一切换输出。
 * 主控制器源码中的 kUseInEKFOdometry=false 为默认值，改为 true 并重新编译后接管。
 * 两套状态完全独立；初始化前不进入运动控制，新系统失效时报告错误，不回退旧 KF。
 * 检查控制时钟及稳态时钟年龄；100 ms 内复用新系统自身的缓存结果。
 * 在这段短延迟内用恒定线速度和角速度推到控制时刻；不是重新执行滤波校正。
 * 开关只在重新编译后生效，不提供运行中切换。
 * 跳变保护：异常校正不提交状态或协方差；控制入口另查位姿、速度和角速度连续性。
 * 孤立异常仅使用本系统可信状态预测；持续 50 ms 异常锁存故障，重新激活后才清除。
 * 两线程仍共享处理器与内存带宽；这是功能隔离，不能据此宣称严格实时隔离。
 *
 * 状态与坐标约定：
 * X=(R_GB,v_GI,p_GI,d_G0,...,d_G3)，零偏为 (bg_B,ba_B)。
 * G 为世界系，I 为真实 IMU 安装位置，B 为 URDF 定义的机身参考系。
 * 在 I 点传播惯性状态，避免对角速度求导来补偿 IMU 安装杆臂。
 * 输出再换算回 B 点，包括角速度叉乘杆臂带来的速度差。
 * 采用右不变误差、左侧修正：X_true=Exp(dx)*X_hat。
 * 27 维误差顺序为 [theta_G,dv_G,dp_G,dd_G0..3,dbg_B,dba_B]。
 * 四个足端状态槽固定保留；触地时重置并维护交叉协方差，摆动时解除约束。
 * 当前针对平地：可靠支撑足加入地面高度软约束，只约束竖直位置，水平仍允许滑移。
 * 地面约束是已知平地先验，不是接触传感器测量；不固定机身高度或机身速度。
 *
 * N100 姿态处理：
 * 使用驱动已经转换过坐标的四元数、角速度和比力，以高信任度融合 SO(3) 姿态。
 * 四元数处理不受符号翻转、欧拉角跨越正负 pi 的影响；陀螺仪一致性门限抑制
 * 短时跳变，持续不一致时保守地恢复融合。
 * AHRS 与原始惯性数据存在相关性，但现有接口不提供交叉协方差，因此这是近似的
 * AHRS 辅助 InEKF，不能宣称观测独立或严格统计一致。
 * 当前十个状态接口不含设备时间戳、帧序号；数值变化检测只是新鲜度启发式检查，
 * 不能替代时间同步，也不能区分完全相同的新帧与旧帧。
 *
 * 滑移处理：
 * 步态接触仅作为先验。根据运动学创新，以及足间/惯性速度不一致程度，连续增大
 * 水平与竖直方向的接触随机游走；支撑期噪声始终非零，允许小幅滑动。
 * 严重异常观测降权。没有外部参考时，一般不能识别所有足端同向滑移。
 *
 * RViz 使用：固定坐标系设为 world，按话题添加 /shadow_inekf/markers，类型 MarkerArray。
 * 红色为新估计，淡蓝色为旧估计；两者均直接使用原始世界系位置，不额外平移显示。
 * 默认显示机身、腿部轮廓和足端可信度球；其他辅助图形默认关闭。
 * 不向场景发布文字，等待或异常原因写入终端日志；数值诊断保留在独立话题中。
 * /shadow_inekf/pose 发布机身在世界系的位姿及协方差。
 * /shadow_inekf/twist 发布机身坐标系下的速度及协方差。
 * /shadow_inekf/diagnostics 发布带列名的数值诊断，记录两套估计的初始原点差。
 * 新增 world -> shadow_inekf_base；原 world/base 变换跟随主算法选择的状态来源。
 * 以下参数为工程起始值，不是 N100 厂家标定参数。
 * 控制器参数 shadow_inekf_enabled 默认为 true，在配置阶段读取。
 *
 * 参考：Hartley 等人的带零偏扩展 InEKF：https://arxiv.org/abs/1904.09251。
 * 自适应接触噪声的思路参考 https://arxiv.org/abs/2510.16755，并非该论文的完整复现。
 */

#include "sysu219_guide_controller/control/ShadowInEKF.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <controller_common/CtrlInterfaces.h>
#include <sysu219_guide_controller/control/Estimator.h>
#include <sysu219_guide_controller/gait/WaveGenerator.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace sysu219_guide_controller {

class ShadowInEKF::Impl final {
    using V3 = Eigen::Vector3d;
    using M3 = Eigen::Matrix3d;
    using V27 = Eigen::Matrix<double, 27, 1>;
    using P27 = Eigen::Matrix<double, 27, 27>;
    using H3 = Eigen::Matrix<double, 3, 27>;
    using Marker = visualization_msgs::msg::Marker;
    static constexpr int kTheta = 0, kVel = 3, kPos = 6, kGyroBias = 21, kAccelBias = 24;
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr std::size_t kQueueSize = 64;
    static_assert(std::atomic<std::size_t>::is_always_lock_free,
                  "Shadow snapshots require lock-free queue indices");
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
                  "Shadow counters require lock-free atomics");

public:
    struct Tuning {
        // 连续时间噪声幅值，进入协方差前取平方，再按实际时间步长离散化。
        double gyro_noise = 0.012;             // 陀螺仪噪声密度，rad/s/sqrt(Hz)
        double accel_noise = 0.30;             // 加速度噪声密度，包含振动，m/s^2/sqrt(Hz)
        double gyro_bias_walk = 0.00015;       // 陀螺仪零偏随机游走，rad/s/sqrt(s)
        double accel_bias_walk = 0.003;        // 加速度计零偏随机游走，m/s^2/sqrt(s)
        double contact_xy_walk = 0.015;        // 水平接触随机游走，支撑期也非零，m/sqrt(s)
        double contact_z_walk = 0.008;         // 竖直接触随机游走，m/sqrt(s)
        double ground_height_sigma = 0.010;    // 平地支撑足高度先验标准差，m，容纳柔性与几何误差
        double encoder_sigma = 0.002;          // 编码器标准差，经运动学雅可比传播，rad
        double fk_xy_sigma = 0.004;            // 水平柔性及模型误差标准差，m
        double fk_z_sigma = 0.006;             // 竖直柔性及模型误差标准差，m
        double attitude_rp_sigma = 0.20 * kPi / 180.0;
        double attitude_yaw_sigma = 0.35 * kPi / 180.0;
        double attitude_jump_gate = 0.60 * kPi / 180.0;
        double attitude_update_period = 0.020; // 限制姿态更新率，避免把重复值当成高频独立观测
        double imu_stale_time = 0.20;          // 数值持续不变的告警时间，不代表设备数据年龄
        double slip_speed = 0.05;              // 滑移一致性的软尺度，非实测滑移速度，m/s
        double max_contact_noise_scale = 100.0; // 接触噪声方差放大上限
        double position_nis_soft = 11.345;     // 三自由度卡方分布的 99% 分位，用于软降权
        double position_nis_hard = 100.0;
        double touchdown_delay = 0.020;
        double static_init_time = 0.35;
        double max_gap = 0.10;
        // 跳变保护的工程起始门限，比较机身物理状态，不比较依赖世界原点的不变误差分量。
        double correction_position_limit = 0.020; // 单次校正及输出预测残差，m
        double correction_velocity_limit = 0.50; // m/s
        double correction_angle_limit = 2.0 * kPi / 180.0;
        double continuity_acceleration = 30.0; // 输出间隔内的加速度容差，m/s^2
        double continuity_angular_acceleration = 100.0; // rad/s^2
        double jump_fault_time = 0.050; // 连续异常最长容忍时间，s；只复用新系统自身预测
        double publish_period = 0.04;
        double history_seconds = 60.0;
    };

    Impl(const std::vector<std::string>& joint_names,
                const std::vector<std::string>& foot_names,
                const std::string& base_name, const std::string& imu_name)
        : joint_names_(joint_names), foot_names_(foot_names),
          base_name_(base_name), imu_name_(imu_name) {
        if (joint_names_.size() != 12 || foot_names_.size() != 4)
            throw std::runtime_error("shadow InEKF requires 12 named joints and four named feet");
        rclcpp::NodeOptions options;
        options.use_global_arguments(false).start_parameter_services(false)
               .start_parameter_event_publisher(false);
        node_ = std::make_shared<rclcpp::Node>("sysu219_shadow_inekf", options);
        // 发布导致的等待仅发生在后台线程，不进入控制器的更新函数。
        const auto qos = rclcpp::QoS(1).reliable().durability_volatile();
        markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/shadow_inekf/markers", qos);
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/shadow_inekf/pose", qos);
        twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/shadow_inekf/twist", qos);
        diagnostics_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/shadow_inekf/diagnostics", qos);
        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
        worker_ = std::thread([this] { run(); });
    }

    ~Impl() {
        accepting_.store(false, std::memory_order_release);
        running_.store(false, std::memory_order_release);
        if (worker_.joinable()) worker_.join();
    }
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;

    // 仅在非实时的机器人描述回调中调用，不共享运动学求解器或模型对象。
    void setRobotDescription(const std::string& xml) {
        std::lock_guard<std::mutex> lock(description_mutex_);
        if (xml != description_) {
            description_ = xml;
            description_pending_.store(true, std::memory_order_release);
        }
    }

    // 仅在生命周期激活阶段调用：按接口名称绑定，不依赖借用接口的排列顺序。
    void startSession(const CtrlInterfaces& io) {
        accepting_.store(false, std::memory_order_release);
        for (int j = 0; j < 12; ++j) {
            q_indices_[j] = findInterface(io.joint_position_state_interface_, joint_names_[j] + "/position");
            qd_indices_[j] = findInterface(io.joint_velocity_state_interface_, joint_names_[j] + "/velocity");
        }
        const std::array<std::string, 10> suffix{{
            "orientation.w", "orientation.x", "orientation.y", "orientation.z",
            "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
            "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}};
        for (int j = 0; j < 10; ++j)
            imu_indices_[j] = findInterface(io.imu_state_interface_, imu_name_ + "/" + suffix[j]);
        session_.fetch_add(1, std::memory_order_relaxed);
        accepting_.store(true, std::memory_order_release);
    }

    void stopSession() noexcept { accepting_.store(false, std::memory_order_release); }

    // 唯一实时入口：不分配内存、不加互斥锁、不发布消息、不执行滤波矩阵运算。
    void submit(const rclcpp::Time& time, const rclcpp::Duration& period,
                const CtrlInterfaces& io, const WaveGenerator& wave, Estimator& legacy,
                bool fixed_stance_prior) noexcept {
        // 模型未就绪时也保留快照，让后台能够使用正确的控制时钟显示失败原因。
        if (!accepting_.load(std::memory_order_acquire)) return;
        try {
            const std::size_t write = write_.load(std::memory_order_relaxed);
            const std::size_t next = (write + 1) % kQueueSize;
            if (next == read_.load(std::memory_order_acquire)) {
                dropped_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            Snapshot& s = queue_[write];
            s.stamp_ns = time.nanoseconds();
            s.steady_ns = steadyNanoseconds();
            s.period = period.seconds();
            s.session = session_.load(std::memory_order_relaxed);
            for (int j = 0; j < 12; ++j) {
                s.q[j] = io.joint_position_state_interface_[q_indices_[j]].get().get_value();
                s.qd[j] = io.joint_velocity_state_interface_[qd_indices_[j]].get().get_value();
            }
            for (int j = 0; j < 10; ++j)
                s.imu[j] = io.imu_state_interface_[imu_indices_[j]].get().get_value();
            for (int leg = 0; leg < 4; ++leg) {
                // 固定站立状态也提供支撑先验，不改变主步态发生器，更不冒充实测触地。
                s.contact[leg] = fixed_stance_prior ? 1 : wave.contact_(leg);
                s.phase[leg] = fixed_stance_prior ? 0.5 : wave.phase_(leg);
            }
            s.stance = wave.get_t_stance();
            s.swing = wave.get_t_swing();
            V3 p, v; M3 rotation;
            legacy.legacyState(p, v, rotation); // 仅供显示与诊断，不参与新滤波器的状态传播或校正。
            const Eigen::Quaterniond q(rotation);
            for (int j = 0; j < 3; ++j) { s.legacy_p[j] = p(j); s.legacy_v[j] = v(j); }
            s.legacy_quat = {{q.w(), q.x(), q.y(), q.z()}};
            write_.store(next, std::memory_order_release);
        } catch (...) {
            // 可选观测模块的异常不能改变主控制器的返回值。
            dropped_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    bool selectForControl(const rclcpp::Time& time, Estimator& estimator) noexcept {
        const auto session = session_.load(std::memory_order_acquire);
        if (control_guard_session_ != session) {
            control_guard_session_ = session;
            have_control_result_ = have_control_output_ = false;
            output_bad_since_ns_ = 0;
            control_result_ = ControlResult{};
        }
        if (guardFaulted(session)) return false;
        // 先在局部检查候选结果，坏结果绝不能覆盖上一次通过检查的参照。
        ControlResult candidate = control_result_;
        for (std::size_t count = 0; count < kQueueSize; ++count) {
            const std::size_t read = result_read_.load(std::memory_order_relaxed);
            if (read == result_write_.load(std::memory_order_acquire)) break;
            if (results_[read].session == session) candidate = results_[read];
            result_read_.store((read + 1) % kQueueSize, std::memory_order_release);
        }
        if (!accepting_.load(std::memory_order_acquire) ||
            !model_ready_.load(std::memory_order_acquire) ||
            !control_ready_.load(std::memory_order_acquire)) return false;
        const auto epoch = control_epoch_.load(std::memory_order_acquire);
        if (have_control_result_ && control_result_.epoch != epoch) {
            latchGuardFault(session, 3); // 已接管后发生内部重置，禁止自动换原点再接回控制。
            return false;
        }
        const auto now_ns = time.nanoseconds(), steady_ns = steadyNanoseconds();
        ControlResult output;
        bool valid = candidate.session == session && candidate.epoch == epoch &&
            (!have_control_result_ || candidate.stamp_ns >= control_result_.stamp_ns) &&
            projectControlResult(candidate, now_ns, steady_ns, output);
        if (valid && have_control_output_) valid = continuousControlResult(control_output_, output);
        if (!valid) {
            output_rejections_.fetch_add(1, std::memory_order_relaxed);
            if (output_bad_since_ns_ == 0) output_bad_since_ns_ = steady_ns;
            if ((steady_ns - output_bad_since_ns_) * 1e-9 >= tune_.jump_fault_time) {
                latchGuardFault(session, 2);
                return false;
            }
            // 孤立异常只沿用最后可信的新状态预测；不抬高时间戳来伪装数据新鲜度。
            if (!have_control_result_ || !projectControlResult(control_result_, now_ns, steady_ns, output)) return false;
        } else {
            if (!have_control_result_ || candidate.stamp_ns > control_result_.stamp_ns)
                output_bad_since_ns_ = 0;
            control_result_ = candidate;
            have_control_result_ = true;
        }
        if (output_bad_since_ns_ != 0 &&
            (steady_ns - output_bad_since_ns_) * 1e-9 >= tune_.jump_fault_time) {
            latchGuardFault(session, 2);
            return false;
        }
        if (guardFaulted(session)) return false;
        estimator.useInEKFState(output.position, output.velocity, output.rotation, output.gyro);
        control_output_ = output;
        have_control_output_ = true;
        return true;
    }

private:
    struct ControlResult {
        std::int64_t stamp_ns = 0, steady_ns = 0;
        std::uint64_t session = 0, epoch = 0;
        V3 position = V3::Zero(), velocity = V3::Zero(), gyro = V3::Zero();
        M3 rotation = M3::Identity();
    };

    bool guardFaulted(std::uint64_t session) const noexcept {
        return session != 0 && guard_fault_session_.load(std::memory_order_acquire) == session;
    }
    void latchGuardFault(std::uint64_t session, int code) noexcept {
        if (session == 0 || session != session_.load(std::memory_order_acquire) || guardFaulted(session)) return;
        guard_fault_code_.store(code, std::memory_order_relaxed);
        guard_fault_session_.store(session, std::memory_order_release);
    }
    bool projectControlResult(const ControlResult& source, std::int64_t now_ns,
                              std::int64_t steady_ns, ControlResult& output) const {
        const double dt = (now_ns - source.stamp_ns) * 1e-9;
        const double age = (steady_ns - source.steady_ns) * 1e-9;
        if (dt < 0.0 || dt > tune_.max_gap || age < 0.0 || age > tune_.max_gap ||
            !source.position.allFinite() || !source.velocity.allFinite() ||
            !source.rotation.allFinite() || !source.gyro.allFinite() ||
            (source.rotation.transpose() * source.rotation - M3::Identity()).norm() > 1e-6 ||
            std::abs(source.rotation.determinant() - 1.0) > 1e-6) return false;
        M3 dr, unused1, unused2;
        so3(source.gyro * dt, dr, unused1, unused2);
        output = source;
        output.position += source.velocity * dt;
        output.rotation = source.rotation * dr;
        output.stamp_ns = now_ns;
        return output.position.allFinite() && output.rotation.allFinite();
    }
    bool continuousControlResult(const ControlResult& previous, const ControlResult& next) const {
        const double dt = (next.stamp_ns - previous.stamp_ns) * 1e-9;
        if (dt < 0.0 || dt > tune_.max_gap) return false;
        M3 dr, unused1, unused2;
        so3(previous.gyro * dt, dr, unused1, unused2);
        return (next.position - previous.position - previous.velocity * dt).norm() <=
                tune_.correction_position_limit + 0.5 * tune_.continuity_acceleration * dt * dt &&
            (next.velocity - previous.velocity).norm() <=
                tune_.correction_velocity_limit + tune_.continuity_acceleration * dt &&
            logarithm((previous.rotation * dr).transpose() * next.rotation).norm() <=
                tune_.correction_angle_limit + 0.5 * tune_.continuity_angular_acceleration * dt * dt &&
            (next.gyro - previous.gyro).norm() <= 0.5 + tune_.continuity_angular_acceleration * dt;
    }

    struct Snapshot {
        std::int64_t stamp_ns = 0, steady_ns = 0;
        std::uint64_t session = 0;
        double period = 0.0, stance = 0.65, swing = 0.325;
        std::array<double, 12> q{}, qd{};
        std::array<double, 10> imu{};
        std::array<int, 4> contact{};
        std::array<double, 4> phase{}, legacy_quat{};
        std::array<double, 3> legacy_p{}, legacy_v{};
    };
    void writeControlResult(const Snapshot& s) {
        const std::size_t write = result_write_.load(std::memory_order_relaxed);
        const std::size_t next = (write + 1) % kQueueSize;
        if (next == result_read_.load(std::memory_order_acquire)) return;
        auto& result = results_[write];
        result.stamp_ns = s.stamp_ns; result.steady_ns = s.steady_ns;
        result.session = s.session; result.epoch = control_epoch_.load(std::memory_order_relaxed);
        result.position = basePosition(); result.velocity = baseVelocity();
        result.rotation = R_; result.gyro = gyro_ - bg_;
        result_write_.store(next, std::memory_order_release);
        control_ready_.store(true, std::memory_order_release);
    }
    struct LegKinematics {
        KDL::Chain chain;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
        std::unique_ptr<KDL::ChainJntToJacSolver> jac;
        KDL::JntArray q{3}, qd{3};
        KDL::Jacobian J{3};
        std::array<int, 3> input_index{};
        V3 r = V3::Zero(), dr = V3::Zero(); // 相对机身的位置及导数，均用机身坐标表达
        M3 covariance = M3::Identity();
    };

    template<class Interfaces>
    static std::size_t findInterface(const Interfaces& interfaces, const std::string& name) {
        for (std::size_t j = 0; j < interfaces.size(); ++j)
            if (interfaces[j].get().get_name() == name) return j;
        throw std::runtime_error("shadow InEKF missing state interface: " + name);
    }
    static std::int64_t steadyNanoseconds() noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    static M3 cross(const V3& a) {
        M3 m = M3::Zero();
        m(0, 1) = -a.z(); m(0, 2) = a.y();
        m(1, 0) = a.z(); m(1, 2) = -a.x();
        m(2, 0) = -a.y(); m(2, 1) = a.x();
        return m;
    }
    static void so3(const V3& phi, M3& rotation, M3& gamma1, M3& gamma2) {
        const double t2 = phi.squaredNorm();
        double a, b, c, d;
        if (t2 < 1e-6) {
            const double t4 = t2 * t2;
            a = 1.0 - t2 / 6.0 + t4 / 120.0;
            b = 0.5 - t2 / 24.0 + t4 / 720.0;
            c = 1.0 / 6.0 - t2 / 120.0 + t4 / 5040.0;
            d = 1.0 / 24.0 - t2 / 720.0 + t4 / 40320.0;
        } else {
            const double t = std::sqrt(t2);
            a = std::sin(t) / t;
            b = (1.0 - std::cos(t)) / t2;
            c = (1.0 - a) / t2;
            d = (0.5 - b) / t2;
        }
        const M3 k = cross(phi), k2 = k * k;
        rotation = M3::Identity() + a * k + b * k2;
        gamma1 = M3::Identity() + b * k + c * k2;
        gamma2 = 0.5 * M3::Identity() + c * k + d * k2;
    }
    static V3 logarithm(const M3& r) {
        Eigen::Quaterniond q(r);
        q.normalize();
        if (q.w() < 0.0) q.coeffs() *= -1.0;
        const double n = q.vec().norm();
        return n < 1e-10 ? V3(2.0 * q.vec())
                         : V3((2.0 * std::atan2(n, q.w()) / n) * q.vec());
    }
    static double median(std::vector<double> values) {
        std::sort(values.begin(), values.end());
        const std::size_t m = values.size() / 2;
        return values.size() % 2 ? values[m] : 0.5 * (values[m - 1] + values[m]);
    }
    static V3 arrayVector(const std::array<double, 3>& a) { return V3(a[0], a[1], a[2]); }
    static geometry_msgs::msg::Point point(const V3& p) {
        geometry_msgs::msg::Point out;
        out.x = p.x(); out.y = p.y(); out.z = p.z();
        return out;
    }
    static geometry_msgs::msg::Quaternion quaternion(const M3& r) {
        Eigen::Quaterniond q(r); q.normalize();
        geometry_msgs::msg::Quaternion out;
        out.w = q.w(); out.x = q.x(); out.y = q.y(); out.z = q.z();
        return out;
    }

    void buildModel(const std::string& xml) {
        // 继续使用与原机器人模型相同的 URDF/KDL 几何定义，但直接解析标准 URDF。
        // 此处不再触发 ROS 的 URDF 解析插件加载，避免后台建模受插件加载失败影响。
        // 工程现有 urdf 依赖已经导出 URDFDOM 的头文件和链接库。
        const auto urdf_model = urdf::parseURDF(xml);
        if (!urdf_model) throw std::runtime_error("URDFDOM could not parse robot description");
        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(*urdf_model, tree))
            throw std::runtime_error("could not convert parsed URDF into a KDL tree");
        for (int leg = 0; leg < 4; ++leg) {
            auto& k = legs_[leg];
            if (!tree.getChain(base_name_, foot_names_[leg], k.chain) || k.chain.getNrOfJoints() != 3)
                throw std::runtime_error("expected three joints in chain to " + foot_names_[leg]);
            int j = 0;
            for (const auto& segment : k.chain.segments) {
                if (segment.getJoint().getType() == KDL::Joint::Fixed) continue;
                const auto it = std::find(joint_names_.begin(), joint_names_.end(), segment.getJoint().getName());
                if (it == joint_names_.end()) throw std::runtime_error("unmapped URDF joint");
                k.input_index[j++] = static_cast<int>(it - joint_names_.begin());
            }
            k.fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(k.chain);
            k.jac = std::make_unique<KDL::ChainJntToJacSolver>(k.chain);
        }
        KDL::Chain imu_chain;
        if (!tree.getChain(base_name_, "imu_link", imu_chain) || imu_chain.getNrOfJoints() != 0)
            throw std::runtime_error("a fixed base -> imu_link URDF transform is required");
        KDL::ChainFkSolverPos_recursive imu_fk(imu_chain);
        KDL::Frame transform;
        if (imu_fk.JntToCart(KDL::JntArray(0), transform) < 0)
            throw std::runtime_error("cannot evaluate IMU extrinsics");
        lever_ = V3(transform.p.x(), transform.p.y(), transform.p.z());
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col) imu_to_base_(row, col) = transform.M(row, col);
        RCLCPP_INFO(node_->get_logger(),
            "Shadow InEKF ready: independent KDL, IMU lever [%.5f %.5f %.5f] m. "
            "RViz world + /shadow_inekf/markers; red=new, light blue=legacy; raw world positions, neither is truth.",
            lever_.x(), lever_.y(), lever_.z());
    }

    void updateModel() {
        const bool new_description = description_pending_.exchange(false, std::memory_order_acq_rel);
        if (!new_description && model_ready_.load(std::memory_order_acquire)) return;
        const std::int64_t now = steadyNanoseconds();
        if (!new_description && now < next_model_retry_ns_) return;
        // 重试使用稳态时钟，仿真暂停或复位不会造成重试风暴。
        next_model_retry_ns_ = now + 1000000000LL;
        model_ready_.store(false, std::memory_order_release);
        std::string xml;
        { std::lock_guard<std::mutex> lock(description_mutex_); xml = description_; }
        if (xml.empty()) {
            model_error_ = "WAITING FOR ROBOT DESCRIPTION";
            return;
        }
        resetFilter();
        ++model_attempts_;
        try {
            buildModel(xml);
            model_error_.clear();
            model_ready_.store(true, std::memory_order_release);
        } catch (const std::exception& e) {
            model_error_ = std::string("MODEL LOAD FAILED: ") + e.what() + "\nAutomatic retry every 1 s";
            RCLCPP_ERROR(node_->get_logger(),
                "[SHADOW_INEKF] 模型加载失败，第 %llu 次；1 秒后自动重试，主控制链继续运行：%s",
                static_cast<unsigned long long>(model_attempts_), e.what());
        } catch (...) {
            model_error_ = "MODEL LOAD FAILED: unknown exception\nAutomatic retry every 1 s";
            RCLCPP_ERROR(node_->get_logger(), "[SHADOW_INEKF] 模型加载异常，1 秒后自动重试。");
        }
    }

    void run() noexcept {
#ifdef __linux__
        // 后台线程使用普通调度策略，避免继承控制线程的实时优先级。
        sched_param scheduling{};
        (void)pthread_setschedparam(pthread_self(), SCHED_OTHER, &scheduling);
        (void)setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 10);
        (void)pthread_setname_np(pthread_self(), "shadow_inekf");
#endif
        while (running_.load(std::memory_order_acquire)) {
            try {
                updateModel();
                bool consumed = false;
                for (int batch = 0; batch < 8; ++batch) {
                    const std::size_t read = read_.load(std::memory_order_relaxed);
                    if (read == write_.load(std::memory_order_acquire)) break;
                    const Snapshot s = queue_[read];
                    read_.store((read + 1) % kQueueSize, std::memory_order_release);
                    consumed = true;
                    if (!accepting_.load(std::memory_order_acquire) ||
                        s.session != session_.load(std::memory_order_relaxed)) continue;
                    const double age = (steadyNanoseconds() - s.steady_ns) * 1e-9;
                    if (age > 0.15) { ++late_; continue; }
                    processing_age_ = age;
                    if (worker_session_ != s.session) {
                        ever_initialized_ = false;
                        resetFilter(); worker_session_ = s.session;
                    }
                    if (!model_ready_.load(std::memory_order_acquire)) {
                        // 无需运动学模型即可报告诊断并清理旧图形，加载失败仍保留终端反馈。
                        publishStatusIfDue(s, s.stamp_ns * 1e-9, model_error_);
                        continue;
                    }
                    process(s);
                }
                if (!consumed) std::this_thread::sleep_for(std::chrono::milliseconds(1));
                else std::this_thread::yield();
            } catch (const std::exception& e) {
                ++faults_;
                if (ever_initialized_) latchGuardFault(worker_session_, 4);
                resetFilter();
                RCLCPP_ERROR(node_->get_logger(), "[INEKF_GUARD] 滤波异常：%s；已接管的系统须重新激活。", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            } catch (...) {
                ++faults_;
                if (ever_initialized_) latchGuardFault(worker_session_, 4);
                resetFilter();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    }

    void resetFilter() {
        if (ever_initialized_) latchGuardFault(worker_session_, 3);
        control_ready_.store(false, std::memory_order_release);
        control_epoch_.fetch_add(1, std::memory_order_acq_rel);
        initialized_ = false;
        last_stamp_ = -1.0; static_since_ = -1.0;
        last_publish_ = -1.0; last_attitude_ = -1.0; last_imu_change_ = -1.0;
        attitude_mismatch_since_ = -1.0;
        correction_bad_since_ = -1.0;
        correction_rejected_this_sample_ = correction_accepted_this_sample_ = false;
        have_imu_ = false; static_samples_ = 0;
        imu_stale_ = false; attitude_reacquiring_ = false; attitude_residual_ = 0.0;
        static_gyro_.setZero(); static_accel_.setZero();
        active_.fill(false); contact_age_.fill(0.0); trust_.fill(0.0);
        slip_score_.fill(0.0); noise_scale_.fill(1.0); nis_.fill(0.0);
        shadow_history_.clear(); legacy_history_.clear(); history_time_.clear();
        status_ = "WAITING FOR VALID IMU / SUPPORT PRIOR";
    }

    bool readKinematics(const Snapshot& s) {
        for (auto& k : legs_) {
            for (int j = 0; j < 3; ++j) {
                k.q(j) = s.q[k.input_index[j]]; k.qd(j) = s.qd[k.input_index[j]];
                if (!std::isfinite(k.q(j)) || !std::isfinite(k.qd(j)) || std::abs(k.qd(j)) > 100.0) return false;
            }
            KDL::Frame pose;
            if (k.fk->JntToCart(k.q, pose) < 0 || k.jac->JntToJac(k.q, k.J) < 0) return false;
            k.r = V3(pose.p.x(), pose.p.y(), pose.p.z());
            const M3 j = k.J.data.topRows(3);
            k.dr = j * k.qd.data;
            k.covariance = tune_.encoder_sigma * tune_.encoder_sigma * j * j.transpose();
            k.covariance.diagonal() += V3(tune_.fk_xy_sigma * tune_.fk_xy_sigma,
                tune_.fk_xy_sigma * tune_.fk_xy_sigma, tune_.fk_z_sigma * tune_.fk_z_sigma);
            if (!k.r.allFinite() || !k.dr.allFinite() || !k.covariance.allFinite()) return false;
        }
        return true;
    }

    void initialize(const Snapshot& s, double now) {
        R_ = measured_R_;
        bg_ = static_gyro_ / static_samples_;
        ba_ = static_accel_ / static_samples_ + R_.transpose() * gravity_;
        // 静止候选条件只是初始化假设，不是外部运动测量。
        if (bg_.norm() > 0.15 || ba_.norm() > 1.5) {
            status_ = "WAITING / STATIC IMU BIAS OUT OF RANGE";
            return;
        }
        std::vector<double> ground;
        for (int leg = 0; leg < 4; ++leg)
            if (s.contact[leg]) ground.push_back((R_ * legs_[leg].r).z());
        const V3 initial_base(0.0, 0.0, -median(ground));
        p_ = initial_base + R_ * lever_;
        // 静止候选期间机身线速度以零初始化，再补偿 IMU 安装杆臂。
        v_ = R_ * ((gyro_ - bg_).cross(lever_));
        P_.setZero();
        P_.block<3, 3>(kTheta, kTheta).diagonal() = V3(
            tune_.attitude_rp_sigma * tune_.attitude_rp_sigma,
            tune_.attitude_rp_sigma * tune_.attitude_rp_sigma,
            tune_.attitude_yaw_sigma * tune_.attitude_yaw_sigma);
        P_.block<3, 3>(kVel, kVel).diagonal().setConstant(0.04);
        P_.block<3, 3>(kPos, kPos).diagonal().setConstant(0.01);
        P_.block<3, 3>(kGyroBias, kGyroBias).diagonal().setConstant(0.000025);
        P_.block<3, 3>(kAccelBias, kAccelBias).diagonal().setConstant(0.01);
        for (int leg = 0; leg < 4; ++leg) {
            anchors_[leg] = p_ + R_ * (legs_[leg].r - lever_);
            P_.block<3, 3>(9 + 3 * leg, 9 + 3 * leg).diagonal().setConstant(1.0);
            active_[leg] = false;
        }
        initial_origin_delta_ = initial_base - arrayVector(s.legacy_p); // 仅记录初始原点差，不移动显示
        if (!initial_origin_delta_.allFinite()) initial_origin_delta_.setZero();
        initialized_ = true; ever_initialized_ = true; last_attitude_ = now;
        attitude_seen_ = Eigen::Quaterniond(measured_R_);
        status_ = "RUNNING / STATIC BIAS BOOTSTRAP";
        RCLCPP_INFO(node_->get_logger(), "Shadow initialized: %s; red=new, light blue=legacy; no display translation.", status_.c_str());
    }

    void process(const Snapshot& s) {
        const double now = s.stamp_ns * 1e-9;
        if (!std::isfinite(now) || s.stamp_ns < 0) return;
        if (guardFaulted(s.session)) {
            control_ready_.store(false, std::memory_order_release);
            publishStatusIfDue(s, now, "FAULT / JUMP GUARD code=" +
                std::to_string(guard_fault_code_.load(std::memory_order_relaxed)) +
                " / REACTIVATE REQUIRED (1=correction,2=output,3=reset,4=numerical)");
            return;
        }
        Eigen::Quaterniond attitude(s.imu[0], s.imu[1], s.imu[2], s.imu[3]);
        if (!attitude.coeffs().allFinite() || attitude.norm() < 0.8 || attitude.norm() > 1.2) {
            control_ready_.store(false, std::memory_order_release);
            ++invalid_;
            publishStatusIfDue(s, now, "WAITING FOR VALID IMU QUATERNION");
            return;
        }
        attitude.normalize();
        measured_R_ = attitude.toRotationMatrix() * imu_to_base_.transpose();
        gyro_ = imu_to_base_ * V3(s.imu[4], s.imu[5], s.imu[6]);
        accel_ = imu_to_base_ * V3(s.imu[7], s.imu[8], s.imu[9]);
        if (!gyro_.allFinite() || !accel_.allFinite() || gyro_.norm() > 30.0 || accel_.norm() > 200.0 ||
            !readKinematics(s)) {
            control_ready_.store(false, std::memory_order_release);
            ++invalid_;
            publishStatusIfDue(s, now, "INVALID IMU OR JOINT STATE: check sensor inputs");
            return;
        }
        if (last_stamp_ >= 0.0 && (now < last_stamp_ || now - last_stamp_ > tune_.max_gap)) {
            ++gaps_; resetFilter();
        }
        if (last_stamp_ >= 0.0 && now == last_stamp_) return; // 仿真暂停时不重复积分
        const double dt = last_stamp_ >= 0.0 ? now - last_stamp_ : 0.0;
        last_stamp_ = now;
        sample_dt_ = dt;
        correction_rejected_this_sample_ = correction_accepted_this_sample_ = false;
        const bool imu_changed = !have_imu_ || s.imu != previous_imu_;
        if (imu_changed) last_imu_change_ = now;
        imu_stale_ = now - last_imu_change_ > tune_.imu_stale_time;

        if (!initialized_) {
            int support = 0;
            for (int leg = 0; leg < 4; ++leg) support += s.contact[leg] != 0;
            double max_qd = 0.0;
            for (double qd : s.qd) max_qd = std::max(max_qd, std::abs(qd));
            const bool static_candidate = support >= 2 && gyro_.norm() < 0.03 && max_qd < 0.12 &&
                std::abs(accel_.norm() - gravity_.norm()) < 0.6 &&
                (static_since_ < 0.0 || logarithm(measured_R_ * static_orientation_.transpose()).norm() < 0.005);
            if (static_candidate) {
                if (static_since_ < 0.0) { static_since_ = now; static_orientation_ = measured_R_; }
                // 取时间序列均值，不假设重复采样值彼此独立。
                // 完全静止的仿真或量化传感器可能合法地连续输出相同数值。
                static_gyro_ += gyro_; static_accel_ += accel_; ++static_samples_;
            } else {
                static_since_ = -1.0; static_samples_ = 0; static_gyro_.setZero(); static_accel_.setZero();
            }
            const bool calibrated = static_since_ >= 0.0 && now - static_since_ >= tune_.static_init_time && static_samples_ >= 10;
            if (support > 0 && calibrated)
                initialize(s, now);
            else status_ = support ? "INITIALIZING / WAITING FOR QUIET SUPPORT" : "WAITING / NO PLANNED SUPPORT";
        } else if (dt > 0.0) {
            const V3 wm = have_imu_ ? V3(0.5 * (previous_gyro_ + gyro_)) : gyro_;
            const V3 am = have_imu_ ? V3(0.5 * (previous_accel_ + accel_)) : accel_;
            const int steps = std::max(1, static_cast<int>(std::ceil(dt / 0.008)));
            for (int j = 0; j < steps; ++j) propagate(wm, am, dt / steps);
            updateAttitude(now, imu_changed);
        }

        if (initialized_) {
            updateContacts(s, dt);
            if (!P_.allFinite() || P_.diagonal().minCoeff() < -1e-9 || P_.diagonal().maxCoeff() > 1e6 ||
                !p_.allFinite() || !v_.allFinite() || !R_.allFinite() || bg_.norm() > 0.5 || ba_.norm() > 5.0)
                throw std::runtime_error("nonfinite/unbounded shadow state or covariance");
            if (correction_rejected_this_sample_) {
                if (correction_bad_since_ < 0.0) correction_bad_since_ = now;
            } else if (correction_accepted_this_sample_) correction_bad_since_ = -1.0;
            if (correction_bad_since_ >= 0.0 && now - correction_bad_since_ >= tune_.jump_fault_time) {
                latchGuardFault(s.session, 1);
                control_ready_.store(false, std::memory_order_release);
                return;
            }
            // 以传感器处理频率回传状态，放在可能等待 DDS 的低频可视化发布之前。
            // 数值不变只作诊断，不据此拒绝完全静止的合法仿真或量化传感器数据。
            writeControlResult(s);
        }
        previous_imu_ = s.imu; previous_gyro_ = gyro_; previous_accel_ = accel_; have_imu_ = true;
        if (last_publish_ < 0.0 || now - last_publish_ >= tune_.publish_period) {
            publish(s, now); last_publish_ = now;
        }
    }

    void propagate(const V3& wm, const V3& am, double dt) {
        // 带零偏扩展的右不变误差动力学，参照 Hartley 2019 年论文公式 28。
        P27 a = P27::Zero(), g = P27::Zero();
        a.block<3, 3>(kVel, kTheta) = cross(gravity_);
        a.block<3, 3>(kPos, kVel).setIdentity();
        a.block<3, 3>(kTheta, kGyroBias) = -R_;
        a.block<3, 3>(kVel, kGyroBias) = -cross(v_) * R_;
        a.block<3, 3>(kPos, kGyroBias) = -cross(p_) * R_;
        a.block<3, 3>(kVel, kAccelBias) = -R_;
        // 噪声列与状态采用相同分块顺序；同一噪声列整体反号不改变协方差。
        g.block<3, 3>(kTheta, kTheta) = R_;
        g.block<3, 3>(kVel, kTheta) = cross(v_) * R_;
        g.block<3, 3>(kPos, kTheta) = cross(p_) * R_;
        g.block<3, 3>(kVel, kVel) = R_;
        V27 variances = V27::Zero();
        variances.segment<3>(kTheta).setConstant(tune_.gyro_noise * tune_.gyro_noise);
        variances.segment<3>(kVel).setConstant(tune_.accel_noise * tune_.accel_noise * (imu_stale_ ? 25.0 : 1.0));
        for (int leg = 0; leg < 4; ++leg) {
            const int index = 9 + 3 * leg;
            a.block<3, 3>(index, kGyroBias) = -cross(anchors_[leg]) * R_;
            g.block<3, 3>(index, kTheta) = cross(anchors_[leg]) * R_;
            g.block<3, 3>(index, index).setIdentity();
            const double scale = active_[leg] ? noise_scale_[leg] : tune_.max_contact_noise_scale;
            variances.segment<3>(index) = scale * V3(
                tune_.contact_xy_walk * tune_.contact_xy_walk,
                tune_.contact_xy_walk * tune_.contact_xy_walk,
                tune_.contact_z_walk * tune_.contact_z_walk);
        }
        g.block<3, 3>(kGyroBias, kGyroBias).setIdentity();
        g.block<3, 3>(kAccelBias, kAccelBias).setIdentity();
        variances.segment<3>(kGyroBias).setConstant(tune_.gyro_bias_walk * tune_.gyro_bias_walk);
        variances.segment<3>(kAccelBias).setConstant(tune_.accel_bias_walk * tune_.accel_bias_walk);
        const P27 adt = a * dt, a2 = adt * adt;
        const P27 phi = P27::Identity() + adt + 0.5 * a2;
        const P27 half = P27::Identity() + 0.5 * adt + 0.125 * a2;
        const P27 noise = g * variances.asDiagonal() * g.transpose();
        // 使用中点状态转移传播噪声，使离散过程噪声协方差保持半正定。
        P_ = (phi * P_ * phi.transpose() + dt * half * noise * half.transpose()).eval();
        if (!imu_stale_) {
            M3 increment, gamma1, gamma2;
            so3((wm - bg_) * dt, increment, gamma1, gamma2);
            const V3 f = am - ba_;
            p_ += v_ * dt + (R_ * gamma2 * f + 0.5 * gravity_) * dt * dt;
            v_ += (R_ * gamma1 * f + gravity_) * dt;
            R_ = R_ * increment;
        } else {
            p_ += v_ * dt; // 数值长期不更新时，不持续积分同一比力值
        }
        symmetrize();
    }

    void symmetrize() {
        P_ = (0.5 * (P_ + P_.transpose())).eval();
        P_.diagonal().array() += 1e-12;
    }

    bool rejectCorrection(const char* reason, double position_step = 0.0,
                          double velocity_step = 0.0, double angle_step = 0.0) {
        ++correction_rejections_;
        correction_rejected_this_sample_ = true;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[INEKF_GUARD] 整次拒绝校正：%s，位置 %.4f m，速度 %.3f m/s，姿态 %.2f deg；状态及协方差未提交。",
            reason, position_step, velocity_step, angle_step * 180.0 / kPi);
        return false;
    }

    bool correct(const H3& h, const V3& innovation, const M3& covariance) {
        const Eigen::Matrix<double, 27, 3> pht = P_ * h.transpose();
        const M3 s = h * pht + covariance;
        const Eigen::LDLT<M3> ldlt(s);
        if (ldlt.info() != Eigen::Success || !ldlt.isPositive() || ldlt.vectorD().minCoeff() < 1e-12)
            return rejectCorrection("创新协方差异常");
        const Eigen::Matrix<double, 27, 3> k = ldlt.solve(pht.transpose()).transpose();
        const V27 delta = k * innovation;
        if (!delta.allFinite()) return rejectCorrection("非有限校正量");
        // 先生成完整候选，所有检查通过后再一并提交，避免只回滚位姿却留下错误协方差。
        M3 dr, jacobian, unused;
        so3(delta.segment<3>(kTheta), dr, jacobian, unused);
        const M3 next_r = Eigen::Quaterniond(M3(dr * R_)).normalized().toRotationMatrix();
        const V3 next_v = dr * v_ + jacobian * delta.segment<3>(kVel);
        const V3 next_p = dr * p_ + jacobian * delta.segment<3>(kPos);
        const V3 next_bg = bg_ + delta.segment<3>(kGyroBias);
        const V3 next_ba = ba_ + delta.segment<3>(kAccelBias);
        const double dp = (next_p - next_r * lever_ - basePosition()).norm();
        const double dv = (next_v - next_r * ((gyro_ - next_bg).cross(lever_)) - baseVelocity()).norm();
        const double da = logarithm(R_.transpose() * next_r).norm();
        if (!next_r.allFinite() || !next_p.allFinite() || !next_v.allFinite() ||
            !next_bg.allFinite() || !next_ba.allFinite() || next_bg.norm() > 0.5 || next_ba.norm() > 5.0 ||
            !std::isfinite(dp) || !std::isfinite(dv) || !std::isfinite(da) ||
            dp > tune_.correction_position_limit || dv > tune_.correction_velocity_limit ||
            da > tune_.correction_angle_limit) return rejectCorrection("机身状态校正越界", dp, dv, da);
        std::array<V3, 4> next_anchors;
        for (int leg = 0; leg < 4; ++leg) {
            next_anchors[leg] = dr * anchors_[leg] + jacobian * delta.segment<3>(9 + 3 * leg);
            if (!next_anchors[leg].allFinite()) return rejectCorrection("足端状态异常");
        }
        const P27 ikh = P27::Identity() - k * h;
        const P27 next_covariance = ikh * P_ * ikh.transpose() + k * covariance * k.transpose();
        if (!next_covariance.allFinite() || next_covariance.diagonal().minCoeff() < -1e-9 ||
            next_covariance.diagonal().maxCoeff() > 1e6) return rejectCorrection("候选协方差异常");
        R_ = next_r; v_ = next_v; p_ = next_p; bg_ = next_bg; ba_ = next_ba;
        anchors_ = next_anchors; P_ = next_covariance;
        symmetrize();
        correction_accepted_this_sample_ = true;
        return true;
    }

    void updateAttitude(double now, bool imu_changed) {
        const Eigen::Quaterniond observed(measured_R_);
        // 四元数 q 与 -q 表示相同姿态，不能把符号变化视为跳变。
        const bool changed = std::abs(observed.dot(attitude_seen_)) < 1.0 - 1e-14;
        if (!imu_changed || imu_stale_ || now - last_attitude_ < tune_.attitude_update_period) return;
        // 惯性数值仍在变化时，保持不变的 AHRS 四元数最多按 10 Hz 保守复用。
        // 持续偏差不能只因四元数数值保持不变而永久被拒绝，否则无法恢复融合。
        if (!changed && now - last_attitude_ < 0.10) return;
        attitude_seen_ = observed; last_attitude_ = now;
        const V3 residual = logarithm(measured_R_ * R_.transpose());
        attitude_residual_ = residual.norm();
        // 真实转动时，为 AHRS 与陀螺仪的异步延迟保留有限容差。
        const double gate = tune_.attitude_jump_gate + 0.015 * (gyro_ - bg_).norm();
        M3 covariance = M3::Zero();
        covariance.diagonal() = V3(tune_.attitude_rp_sigma * tune_.attitude_rp_sigma,
            tune_.attitude_rp_sigma * tune_.attitude_rp_sigma,
            tune_.attitude_yaw_sigma * tune_.attitude_yaw_sigma);
        attitude_reacquiring_ = false;
        if (residual.norm() > gate) {
            if (attitude_mismatch_since_ < 0.0) attitude_mismatch_since_ = now;
            if (now - attitude_mismatch_since_ < 0.20) { ++attitude_rejected_; return; }
            // 偏差持续存在时增大观测噪声并逐步恢复融合，避免永久拒绝姿态观测。
            const double scale = std::clamp(residual.squaredNorm() / (gate * gate), 4.0, 400.0);
            covariance *= scale;
            attitude_reacquiring_ = true;
        } else attitude_mismatch_since_ = -1.0;
        H3 h = H3::Zero(); h.block<3, 3>(0, kTheta).setIdentity();
        if (correct(h, residual, covariance)) ++attitude_accepted_;
    }

    void resetContact(int leg, bool attach) {
        const int index = 9 + 3 * leg;
        anchors_[leg] = p_ + R_ * (legs_[leg].r - lever_);
        P27 reset = P27::Identity();
        reset.block<3, 27>(index, 0).setZero();
        if (attach) reset.block<3, 3>(index, kPos).setIdentity();
        P_ = (reset * P_ * reset.transpose()).eval();
        P_.block<3, 3>(index, index) += attach ? M3(R_ * legs_[leg].covariance * R_.transpose()) : M3::Identity();
        active_[leg] = attach;
        symmetrize();
    }

    void updateContacts(const Snapshot& s, double dt) {
        const V3 omega = gyro_ - bg_;
        std::array<V3, 4> implied_velocity;
        std::array<std::vector<double>, 3> support_velocities;
        for (int leg = 0; leg < 4; ++leg) {
            implied_velocity[leg] = -R_ * (legs_[leg].dr + omega.cross(legs_[leg].r - lever_));
            if (s.contact[leg])
                for (int axis = 0; axis < 3; ++axis) support_velocities[axis].push_back(implied_velocity[leg](axis));
        }
        V3 consensus = v_;
        if (!support_velocities[0].empty())
            for (int axis = 0; axis < 3; ++axis) consensus(axis) = median(support_velocities[axis]);
        const double window = 0.2 * std::min(1.0, s.swing / std::max(s.stance, 1e-3));
        for (int leg = 0; leg < 4; ++leg) {
            nis_[leg] = 0.0;
            if (!s.contact[leg]) {
                if (active_[leg]) resetContact(leg, false);
                contact_age_[leg] = 0.0; trust_[leg] = 0.0; slip_score_[leg] = 0.0;
                noise_scale_[leg] = tune_.max_contact_noise_scale;
                continue;
            }
            contact_age_[leg] += dt;
            const double phase = std::isfinite(s.phase[leg]) ? std::clamp(s.phase[leg], 0.0, 1.0) : 0.0;
            const double phase_trust = std::clamp(std::min(phase, 1.0 - phase) / std::max(window, 0.01), 0.0, 1.0);
            const double contact_trust = phase_trust * std::clamp(contact_age_[leg] / 0.065, 0.0, 1.0);
            // 将每条腿推算的速度同时与惯性预测、其他支撑腿比较。
            // 关节速度仅参与可靠性判断，不作为额外的相关卡尔曼观测重复融合。
            // 所有足端同向滑移时，这项检查可能无法发现异常。
            const double disagreement = std::min((implied_velocity[leg] - v_).norm(),
                                                 (implied_velocity[leg] - consensus).norm() + 0.02);
            const double target = std::clamp(disagreement / tune_.slip_speed, 0.0, 10.0);
            const double alpha = 1.0 - std::exp(-dt / 0.04);
            slip_score_[leg] += alpha * (target - slip_score_[leg]);
            noise_scale_[leg] = std::clamp(1.0 + slip_score_[leg] * slip_score_[leg] +
                30.0 * (1.0 - contact_trust), 1.0, tune_.max_contact_noise_scale);
            trust_[leg] = contact_trust / std::sqrt(noise_scale_[leg]);
            if (contact_age_[leg] < tune_.touchdown_delay || contact_trust < 0.05) continue;
            if (!active_[leg]) { resetContact(leg, true); continue; }
            H3 h = H3::Zero();
            h.block<3, 3>(0, kPos) = -M3::Identity();
            h.block<3, 3>(0, 9 + 3 * leg).setIdentity();
            const V3 residual = R_ * (legs_[leg].r - lever_) + p_ - anchors_[leg];
            M3 covariance = R_ * legs_[leg].covariance * R_.transpose();
            covariance *= (1.0 + slip_score_[leg]) / std::max(contact_trust, 0.05);
            const M3 innovation_covariance = h * P_ * h.transpose() + covariance;
            const Eigen::LDLT<M3> ldlt(innovation_covariance);
            if (ldlt.info() != Eigen::Success || !ldlt.isPositive() || ldlt.vectorD().minCoeff() < 1e-12) continue;
            nis_[leg] = residual.dot(ldlt.solve(residual));
            if (!std::isfinite(nis_[leg])) continue;
            if (nis_[leg] > tune_.position_nis_soft) {
                const double scale = std::clamp(nis_[leg] / tune_.position_nis_soft, 1.0, tune_.max_contact_noise_scale);
                noise_scale_[leg] = std::max(noise_scale_[leg], scale);
                trust_[leg] /= std::sqrt(scale);
                covariance *= scale;
            }
            if (nis_[leg] > tune_.position_nis_hard) {
                // 异常触地点保留较大的移动不确定度，避免强行固定错误的足端锚点。
                noise_scale_[leg] = tune_.max_contact_noise_scale; trust_[leg] = 0.0;
                continue;
            }
            (void)correct(h, residual, covariance);
            // 平地高度观测：锚点 z 接近初始地面 z=0，水平滑移不需要被固定。
            // 右不变误差下 δd = -[d]×δθ + δd_G，姿态项必须保留。
            // 复用三维校正接口，后两行 H 与创新均为零，等价于一次标量观测。
            H3 ground_h = H3::Zero();
            ground_h.block<1, 3>(0, kTheta) = -cross(anchors_[leg]).row(2);
            ground_h(0, 9 + 3 * leg + 2) = 1.0;
            M3 ground_covariance = M3::Identity();
            ground_covariance(0, 0) = tune_.ground_height_sigma * tune_.ground_height_sigma /
                std::max(contact_trust, 0.05);
            (void)correct(ground_h, V3(-anchors_[leg].z(), 0.0, 0.0), ground_covariance);
        }
    }

    V3 basePosition() const { return p_ - R_ * lever_; }
    V3 baseVelocity() const { return v_ - R_ * ((gyro_ - bg_).cross(lever_)); }
    Eigen::Matrix<double, 6, 6> poseCovariance() const {
        Eigen::Matrix<double, 6, 27> j = Eigen::Matrix<double, 6, 27>::Zero();
        j.block<3, 3>(0, kTheta) = -cross(basePosition());
        j.block<3, 3>(0, kPos).setIdentity();
        j.block<3, 3>(3, kTheta).setIdentity();
        return j * P_ * j.transpose();
    }

    Marker marker(const Snapshot& s, const std::string& name, int id, int type,
                  double red, double green, double blue, double alpha = 1.0) const {
        Marker m;
        m.header.frame_id = "world"; m.header.stamp = rclcpp::Time(s.stamp_ns);
        m.ns = name; m.id = id; m.type = type; m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.color.r = static_cast<float>(red); m.color.g = static_cast<float>(green);
        m.color.b = static_cast<float>(blue); m.color.a = static_cast<float>(alpha);
        m.lifetime = rclcpp::Duration::from_seconds(0.30);
        return m;
    }

    void publishStatusIfDue(const Snapshot& s, double now, const std::string& reason) {
        if (s.stamp_ns < 0) return;
        if (last_publish_ >= 0.0 && now >= last_publish_ && now - last_publish_ < tune_.publish_period) return;
        publishStatus(s, reason);
        last_publish_ = now;
    }

    void publishStatus(const Snapshot& s, const std::string& reason) {
        // 状态只写终端，不在三维场景叠加文字，也不为未初始化的估计器编造位姿。
        if (reason != last_status_reason_) {
            RCLCPP_INFO(node_->get_logger(), "[SHADOW_INEKF] %s", reason.c_str());
            last_status_reason_ = reason;
        }
        visualization_msgs::msg::MarkerArray scene;
        Marker clear;
        clear.header.frame_id = "world";
        clear.header.stamp = rclcpp::Time(s.stamp_ns);
        clear.action = Marker::DELETEALL;
        scene.markers.push_back(clear);
        markers_pub_->publish(scene);
    }

    void publish(const Snapshot& s, double now) {
        if (!initialized_) {
            publishStatus(s, status_);
            return;
        }
        visualization_msgs::msg::MarkerArray scene;
        last_status_reason_.clear();
        const V3 pb = basePosition(), vb = baseVelocity();
        const V3 lp_raw = arrayVector(s.legacy_p), lv = arrayVector(s.legacy_v);
        // 旧骨架采用旧估计器的原始位置，与主控制器发布的基座 TF 使用同一坐标定义。
        const V3 lp = lp_raw;
        const bool legacy_valid = lp.allFinite() && lv.allFinite();
        Eigen::Quaterniond lq(s.legacy_quat[0], s.legacy_quat[1], s.legacy_quat[2], s.legacy_quat[3]);
        const M3 lr = lq.coeffs().allFinite() && lq.norm() > 0.5 ? M3(lq.normalized().toRotationMatrix()) : R_;
        const auto pose_cov = poseCovariance();
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.frame_id = "world"; pose.header.stamp = rclcpp::Time(s.stamp_ns);
        pose.pose.pose.position = point(pb); pose.pose.pose.orientation = quaternion(R_);
        for (int row = 0; row < 6; ++row)
            for (int col = 0; col < 6; ++col) pose.pose.covariance[6 * row + col] = pose_cov(row, col);
        pose_pub_->publish(pose);
        geometry_msgs::msg::TwistWithCovarianceStamped twist;
        twist.header = pose.header; twist.header.frame_id = "shadow_inekf_base";
        const V3 body_v = R_.transpose() * vb, omega = gyro_ - bg_;
        twist.twist.twist.linear.x = body_v.x(); twist.twist.twist.linear.y = body_v.y(); twist.twist.twist.linear.z = body_v.z();
        twist.twist.twist.angular.x = omega.x(); twist.twist.twist.angular.y = omega.y(); twist.twist.twist.angular.z = omega.z();
        Eigen::Matrix<double, 6, 27> jt = Eigen::Matrix<double, 6, 27>::Zero();
        jt.block<3, 3>(0, kVel) = R_.transpose();
        jt.block<3, 3>(0, kGyroBias) = -cross(lever_);
        jt.block<3, 3>(3, kGyroBias) = -M3::Identity();
        Eigen::Matrix<double, 6, 6> twist_cov = jt * P_ * jt.transpose();
        // 陀螺仪瞬时测量噪声也会通过杆臂影响输出线速度。
        Eigen::Matrix<double, 6, 3> gyro_map;
        gyro_map.topRows<3>() = cross(lever_); gyro_map.bottomRows<3>().setIdentity();
        twist_cov += 0.0004 * gyro_map * gyro_map.transpose();
        for (int row = 0; row < 6; ++row)
            for (int col = 0; col < 6; ++col) twist.twist.covariance[6 * row + col] = twist_cov(row, col);
        twist_pub_->publish(twist);
        geometry_msgs::msg::TransformStamped tf;
        tf.header = pose.header; tf.child_frame_id = "shadow_inekf_base";
        tf.transform.translation.x = pb.x(); tf.transform.translation.y = pb.y(); tf.transform.translation.z = pb.z();
        tf.transform.rotation = pose.pose.pose.orientation; tf_->sendTransform(tf);

        shadow_history_.push_back(point(pb)); legacy_history_.push_back(point(legacy_valid ? lp : pb)); history_time_.push_back(now);
        while (history_time_.size() > 1600 || (!history_time_.empty() && now - history_time_.front() > tune_.history_seconds)) {
            history_time_.pop_front(); shadow_history_.pop_front(); legacy_history_.pop_front();
        }
        for (int which = 0; which < (legacy_valid ? 2 : 1); ++which) {
            const V3 position = which ? lp : pb, velocity = which ? lv : vb;
            const M3 rotation = which ? lr : R_;
            const double red = which ? 0.45 : 1.0, green = which ? 0.78 : 0.08, blue = which ? 1.0 : 0.08;
            Marker body = marker(s, "body", which, Marker::CUBE, red, green, blue, 0.22);
            body.pose.position = point(position); body.pose.orientation = quaternion(rotation);
            body.scale.x = 0.35; body.scale.y = 0.18; body.scale.z = 0.09;
            scene.markers.push_back(body);
            Marker skeleton = marker(s, "legs", which, Marker::LINE_LIST, red, green, blue, 0.95);
            skeleton.scale.x = 0.009;
            for (auto& k : legs_) {
                V3 prior = position;
                for (unsigned int segment = 1; segment <= k.chain.getNrOfSegments(); ++segment) {
                    KDL::Frame frame;
                    if (k.fk->JntToCart(k.q, frame, static_cast<int>(segment)) < 0) continue;
                    const V3 next = position + rotation * V3(frame.p.x(), frame.p.y(), frame.p.z());
                    skeleton.points.push_back(point(prior)); skeleton.points.push_back(point(next)); prior = next;
                }
            }
            scene.markers.push_back(skeleton);
            Marker arrow = marker(s, "velocity_0.4s", which, Marker::ARROW, red, green, blue);
            arrow.scale.x = 0.014; arrow.scale.y = 0.030; arrow.scale.z = 0.045;
            arrow.points = {point(position), point(position + 0.4 * velocity)};
            scene.markers.push_back(arrow);
            Marker trail = marker(s, "trajectory_60s", which, Marker::LINE_STRIP, red, green, blue);
            trail.scale.x = 0.008;
            const auto& history = which ? legacy_history_ : shadow_history_;
            trail.points.assign(history.begin(), history.end()); scene.markers.push_back(trail);
        }
        for (int axis = 0; axis < 3; ++axis) {
            Marker line = marker(s, "shadow_axes", axis, Marker::ARROW, axis == 0, axis == 1, axis == 2);
            line.scale.x = 0.006; line.scale.y = 0.014; line.scale.z = 0.018;
            line.points = {point(pb), point(pb + 0.14 * R_.col(axis))}; scene.markers.push_back(line);
        }
        Marker raw_heading = marker(s, "raw_n100_heading", 0, Marker::ARROW, 1.0, 0.1, 0.8, 0.7);
        raw_heading.scale.x = 0.005; raw_heading.scale.y = 0.012; raw_heading.scale.z = 0.016;
        raw_heading.points = {point(pb), point(pb + 0.20 * measured_R_.col(0))}; scene.markers.push_back(raw_heading);
        if (legacy_valid) {
            Marker difference = marker(s, "estimate_difference", 0, Marker::LINE_LIST, 1.0, 1.0, 0.2);
            difference.scale.x = 0.004; difference.points = {point(pb), point(lp)}; scene.markers.push_back(difference);
        }
        Eigen::SelfAdjointEigenSolver<M3> eig(pose_cov.topLeftCorner<3, 3>());
        if (eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() >= -1e-9) {
            Marker ellipsoid = marker(s, "position_2sigma", 0, Marker::SPHERE, 1.0, 0.08, 0.08, 0.10);
            M3 axes = eig.eigenvectors(); if (axes.determinant() < 0.0) axes.col(0) *= -1.0;
            ellipsoid.pose.position = point(pb); ellipsoid.pose.orientation = quaternion(axes);
            const V3 diameter = 4.0 * eig.eigenvalues().cwiseMax(1e-8).cwiseSqrt();
            ellipsoid.scale.x = diameter.x(); ellipsoid.scale.y = diameter.y(); ellipsoid.scale.z = diameter.z();
            scene.markers.push_back(ellipsoid);
        }
        for (int leg = 0; leg < 4; ++leg) {
            const V3 foot = pb + R_ * legs_[leg].r;
            const double suspect = std::clamp(slip_score_[leg] / 2.0, 0.0, 1.0);
            Marker contact = marker(s, "contact_candidates", leg, Marker::SPHERE,
                s.contact[leg] ? suspect : 0.4, s.contact[leg] ? 1.0 - suspect : 0.4, 0.15);
            contact.pose.position = point(foot); contact.scale.x = contact.scale.y = contact.scale.z = 0.025;
            scene.markers.push_back(contact);
            if (active_[leg]) {
                Marker anchor = marker(s, "contact_model_residual", leg, Marker::LINE_LIST, 1.0, 0.15, 0.25);
                anchor.scale.x = 0.004; anchor.points = {point(foot), point(anchors_[leg])}; scene.markers.push_back(anchor);
            }
        }
        markers_pub_->publish(scene);
        publishDiagnostics(s, now, pb, vb, lp_raw, lv);
    }

    void publishDiagnostics(const Snapshot& s, double now, const V3& pb, const V3& vb,
                            const V3& legacy_p, const V3& legacy_v) {
        std_msgs::msg::Float64MultiArray m;
        const auto append = [&m](const V3& value) { for (int j = 0; j < 3; ++j) m.data.push_back(value(j)); };
        m.data = {now, sample_dt_, s.period, processing_age_};
        append(pb); append(vb); append(legacy_p); append(legacy_v); append(initial_origin_delta_); append(bg_); append(ba_);
        m.data.insert(m.data.end(), {attitude_residual_, imu_stale_ ? 1.0 : 0.0,
            static_cast<double>(attitude_accepted_), static_cast<double>(attitude_rejected_),
            static_cast<double>(dropped_.load(std::memory_order_relaxed)), static_cast<double>(late_),
            static_cast<double>(gaps_), static_cast<double>(faults_), static_cast<double>(invalid_)});
        for (int leg = 0; leg < 4; ++leg)
            m.data.insert(m.data.end(), {static_cast<double>(s.contact[leg]), trust_[leg], slip_score_[leg], nis_[leg], noise_scale_[leg]});
        m.data.insert(m.data.end(), {static_cast<double>(correction_rejections_),
            static_cast<double>(output_rejections_.load(std::memory_order_relaxed)),
            guardFaulted(s.session) ? static_cast<double>(guard_fault_code_.load(std::memory_order_relaxed)) : 0.0});
        m.layout.dim.resize(1);
        m.layout.dim[0].label = "t_control,dt_stamp,dt_period,worker_age,shadow_px,py,pz,vx,vy,vz,"
            "legacy_raw_px,py,pz,vx,vy,vz,initial_origin_dx,dy,dz,bgx,bgy,bgz,bax,bay,baz,"
            "att_residual_rad,imu_values_unchanged,att_accept,att_reject,queue_drop,late_drop,gaps,faults,invalid,"
            "FR_prior,trust,slip_score,nis,Qscale,FL_prior,trust,slip_score,nis,Qscale,"
            "RR_prior,trust,slip_score,nis,Qscale,RL_prior,trust,slip_score,nis,Qscale,"
            "guard_correction_reject,guard_output_reject,guard_fault_code";
        m.layout.dim[0].size = static_cast<std::uint32_t>(m.data.size());
        m.layout.dim[0].stride = m.layout.dim[0].size;
        diagnostics_pub_->publish(m);
    }

    Tuning tune_;
    std::vector<std::string> joint_names_, foot_names_;
    std::string base_name_, imu_name_;
    std::array<std::size_t, 12> q_indices_{}, qd_indices_{};
    std::array<std::size_t, 10> imu_indices_{};
    std::array<Snapshot, kQueueSize> queue_{};
    alignas(64) std::atomic<std::size_t> write_{0};
    alignas(64) std::atomic<std::size_t> read_{0};
    // 后台生产、控制线程消费；缓存只由控制线程访问，不共享滤波器矩阵。
    std::array<ControlResult, kQueueSize> results_{};
    ControlResult control_result_;
    // 以下输出参照及计时只由控制线程访问，不由后台重置或写入。
    ControlResult control_output_;
    bool have_control_result_ = false, have_control_output_ = false;
    std::uint64_t control_guard_session_ = 0;
    std::int64_t output_bad_since_ns_ = 0;
    std::atomic<std::uint64_t> output_rejections_{0}, guard_fault_session_{0};
    std::atomic<int> guard_fault_code_{0};
    alignas(64) std::atomic<std::size_t> result_write_{0}, result_read_{0};
    std::atomic<std::uint64_t> control_epoch_{0};
    std::atomic<bool> control_ready_{false};
    std::atomic<std::uint64_t> session_{0}, dropped_{0};
    std::atomic<bool> accepting_{false}, running_{true}, model_ready_{false}, description_pending_{false};
    std::mutex description_mutex_;
    std::string description_;
    // 以下重试状态只由后台线程访问，不引入控制线程的等待或共享模型读写。
    std::string model_error_ = "WAITING FOR ROBOT DESCRIPTION";
    std::int64_t next_model_retry_ns_ = 0;
    std::uint64_t model_attempts_ = 0;
    std::thread worker_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    std::array<LegKinematics, 4> legs_;
    M3 imu_to_base_ = M3::Identity(), R_ = M3::Identity(), measured_R_ = M3::Identity();
    M3 static_orientation_ = M3::Identity();
    V3 lever_ = V3::Zero(), v_ = V3::Zero(), p_ = V3::Zero(), bg_ = V3::Zero(), ba_ = V3::Zero();
    const V3 gravity_{0.0, 0.0, -9.81};
    P27 P_ = P27::Identity();
    std::array<V3, 4> anchors_{{V3::Zero(), V3::Zero(), V3::Zero(), V3::Zero()}};
    std::array<bool, 4> active_{};
    std::array<double, 4> contact_age_{}, trust_{}, slip_score_{}, noise_scale_{}, nis_{};
    bool initialized_ = false, have_imu_ = false, imu_stale_ = false, attitude_reacquiring_ = false;
    // 以下校正拒绝状态只由后台线程访问；故障按会话锁存，不自动重置原点恢复控制。
    bool ever_initialized_ = false, correction_rejected_this_sample_ = false, correction_accepted_this_sample_ = false;
    double correction_bad_since_ = -1.0;
    std::uint64_t correction_rejections_ = 0;
    double last_stamp_ = -1.0, static_since_ = -1.0, last_publish_ = -1.0;
    double last_attitude_ = -1.0, last_imu_change_ = -1.0, attitude_mismatch_since_ = -1.0;
    double sample_dt_ = 0.0, processing_age_ = 0.0, attitude_residual_ = 0.0;
    std::uint64_t worker_session_ = 0, late_ = 0, gaps_ = 0, faults_ = 0, invalid_ = 0;
    std::uint64_t attitude_accepted_ = 0, attitude_rejected_ = 0;
    int static_samples_ = 0;
    std::array<double, 10> previous_imu_{};
    V3 gyro_ = V3::Zero(), accel_ = V3::Zero(), previous_gyro_ = V3::Zero(), previous_accel_ = V3::Zero();
    V3 static_gyro_ = V3::Zero(), static_accel_ = V3::Zero(), initial_origin_delta_ = V3::Zero();
    Eigen::Quaterniond attitude_seen_ = Eigen::Quaterniond::Identity();
    std::string status_;
    std::string last_status_reason_;
    std::deque<geometry_msgs::msg::Point> shadow_history_, legacy_history_;
    std::deque<double> history_time_;
};

ShadowInEKF::ShadowInEKF(const std::vector<std::string>& joint_names,
                         const std::vector<std::string>& foot_names,
                         const std::string& base_name, const std::string& imu_name)
    : impl_(std::make_unique<Impl>(joint_names, foot_names, base_name, imu_name)) {}

ShadowInEKF::~ShadowInEKF() = default;

void ShadowInEKF::setRobotDescription(const std::string& xml) { impl_->setRobotDescription(xml); }
void ShadowInEKF::startSession(const CtrlInterfaces& io) { impl_->startSession(io); }
void ShadowInEKF::stopSession() noexcept { impl_->stopSession(); }
void ShadowInEKF::submit(const rclcpp::Time& time, const rclcpp::Duration& period,
                         const CtrlInterfaces& io, const WaveGenerator& wave, Estimator& legacy,
                         bool fixed_stance_prior) noexcept {
    impl_->submit(time, period, io, wave, legacy, fixed_stance_prior);
}
bool ShadowInEKF::selectForControl(const rclcpp::Time& time, Estimator& estimator) noexcept {
    return impl_->selectForControl(time, estimator);
}

} // 命名空间 sysu219_guide_controller
