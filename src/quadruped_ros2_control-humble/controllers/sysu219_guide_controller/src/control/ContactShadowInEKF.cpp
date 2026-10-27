/*
 * 黄色接触影子：力观测、接触状态机和独立诊断集中在本文件。
 * 不连接任何命令接口，不修改 WaveGenerator，不向在役 Estimator 写回。
 * InEKF 使用 ShadowInEKF 的另一实例；滤波参数和已有滑移处理不另行调参。
 *
 * 力源是电机反馈力矩，不是足底传感器。采用广义动量残差：
 * p_j = M_j(q)*v，r_dot = K*(p_dot_j - tau_j + C_j - Mdot_j*v - r)。
 * 由 M*qddot+C=S^T*tau+J^T*f，残差 r 对应 J_j^T*f。
 * 离散式用后向欧拉反馈，避免对关节速度直接求二阶差分；Mdot 用相邻模型差。
 * M/C 包括已有 URDF 的重力、科氏项、输出轴 B/Fc/Ja，不能重复补偿。
 * 思路参考 Bledt 等 Contact Model Fusion (ICRA 2018)，并非其完整复现：
 * https://dspace.mit.edu/handle/1721.1/120350
 *
 * 当前硬件接口没有设备时间戳/序号；数值变化仅是辅助证据，不宣称帧级同步。
 * 阈值、几何门限及观测器带宽均为旁路标定起点，输出不授权机器人承重。
 */
#include "sysu219_guide_controller/control/ContactShadowInEKF.hpp"
#include "sysu219_guide_controller/control/ShadowInEKF.hpp"
#include "sysu219_guide_controller/robot/FloatingBaseModel.h"

#include <Eigen/LU>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>
#include <unistd.h>

namespace sysu219_guide_controller {
namespace {
using V3 = Eigen::Vector3d;
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Vec18 = Eigen::Matrix<double, 18, 1>;
using Mat18 = Eigen::Matrix<double, 18, 18>;
using Sample = ShadowInEKF::ContactSample;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Unloaded 表示没有可靠载荷证据，不能据此断言几何上已经离地。
enum State { Unknown = 0, Air = 1, Touchdown = 2, Contact = 3, Releasing = 4, Unloaded = 5 };
enum Reason { Reset = 0, Candidate = 1, Confirmed = 2, Cancelled = 3,
              PlannedUnload = 4, LoadLost = 5, AirMotion = 6, InvalidForce = 7 };

class ForceContactObserver final : public ShadowInEKF::ContactObserver {
public:
    ForceContactObserver(std::string base, std::vector<std::string> feet,
                         ContactShadowInEKF::Settings settings)
        : base_(std::move(base)), feet_(std::move(feet)), settings_(settings) {
        if (!std::isfinite(settings_.force_on_N) || !std::isfinite(settings_.force_off_N) ||
            settings_.force_off_N < 0.0 || settings_.force_on_N <= settings_.force_off_N ||
            !std::isfinite(settings_.confirm_on_s) || settings_.confirm_on_s <= 0.0 ||
            !std::isfinite(settings_.confirm_off_s) || settings_.confirm_off_s <= 0.0)
            throw std::invalid_argument("Invalid contact-shadow hysteresis/debounce settings");
        rclcpp::NodeOptions options;
        options.use_global_arguments(false).start_parameter_services(false).start_parameter_event_publisher(false);
        node_ = std::make_shared<rclcpp::Node>("sysu219_contact_observer", options);
        diagnostics_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/contact_shadow_inekf/contact_diagnostics", rclcpp::SensorDataQoS());
    }

    void setRobotDescription(const std::string& xml) override {
        // 与控制器使用不同对象、不同缓存，只在黄色后台线程解析。
        model_ = std::make_unique<sysu219::wbc::FloatingBaseModel>(xml, base_, feet_);
    }

    void reset() override {
        have_momentum_ = false; observer_age_ = 0.0;
        state_.fill(Unknown); reason_.fill(Reset); flags_.fill(0);
        pending_s_.fill(0.0); changes_.fill(0); transitions_.fill(0);
        contact_.fill(0); raw_fz_.fill(kNaN); fz_.fill(kNaN); rcond_.fill(kNaN);
        relative_z_.fill(kNaN); relative_vz_.fill(kNaN);
        residual_.setZero(); last_publish_s_ = last_flush_s_ = -1.0;
        if (csv_.is_open()) csv_.close();
        csv_attempted_ = false;
    }

    void update(const Sample& s, std::array<int, 4>& contact, std::array<double, 4>& phase) override {
        contact.fill(0); phase.fill(0.5); flags_.fill(0); contact_.fill(0);
        raw_fz_.fill(kNaN); fz_.fill(kNaN); rcond_.fill(kNaN);
        relative_z_.fill(kNaN); relative_vz_.fill(kNaN);
        if (!model_ || !s.q.allFinite() || !s.qd.allFinite() || !s.tau.allFinite() ||
            !s.R_OB.allFinite() || !s.gyro.allFinite() || !s.velocity.allFinite() ||
            !s.position.allFinite() || !s.feet_B.allFinite() || !s.feet_velocity_B.allFinite()) {
            have_momentum_ = false;
            invalidate(); return;
        }
        sysu219::wbc::WbcInput input;
        input.R_OB = s.R_OB; input.p_com_O = s.position; input.q_j = s.q;
        input.qdot.head<3>() = s.gyro;
        input.qdot.segment<3>(3) = s.R_OB.transpose() * s.velocity;
        input.qdot.tail<12>() = s.qd;
        Mat18 M;
        Vec18 C;
        model_->update(input, M, C);
        const Vec12 momentum = M.bottomRows<12>() * input.qdot;
        if (!M.allFinite() || !C.allFinite() || !momentum.allFinite()) {
            have_momentum_ = false; invalidate(); return;
        }
        const double dt = s.dt;
        const bool time_valid = std::isfinite(dt) && dt > 0.0 && dt <= kMaxGap;
        if (!have_momentum_ || !time_valid) {
            previous_M_ = M; previous_v_ = input.qdot; previous_C_ = C;
            previous_momentum_ = momentum; previous_tau_ = s.tau;
            previous_q_ = s.q; previous_qd_ = s.qd;
            have_momentum_ = true; observer_age_ = 0.0; residual_.setZero();
            invalidate(); return;
        }
        const Vec18 average_v = 0.5 * (input.qdot + previous_v_);
        const Vec12 model_term = 0.5 * (C.tail<12>() + previous_C_.tail<12>()) -
            ((M.bottomRows<12>() - previous_M_.bottomRows<12>()) / dt) * average_v;
        const Vec12 average_tau = 0.5 * (s.tau + previous_tau_);
        residual_ = ((residual_ + kObserverGain * (momentum - previous_momentum_) +
            kObserverGain * dt * (model_term - average_tau)) / (1.0 + kObserverGain * dt)).eval();
        if (!residual_.allFinite()) {
            have_momentum_ = false; invalidate(); return;
        }
        observer_age_ += dt;
        const double mass = M.block<3, 3>(3, 3).trace() / 3.0;
        std::array<bool, 4> changed{};
        for (int leg = 0; leg < 4; ++leg) {
            const int j = 3 * leg;
            const Eigen::Matrix3d J = input.J_f_O[leg].block<3, 3>(0, 6 + j);
            const Eigen::FullPivLU<Eigen::Matrix3d> lu(J.transpose());
            rcond_[leg] = lu.rcond();
            changed[leg] = (s.q.segment<3>(j).array() != previous_q_.segment<3>(j).array()).any() ||
                (s.qd.segment<3>(j).array() != previous_qd_.segment<3>(j).array()).any() ||
                (s.tau.segment<3>(j).array() != previous_tau_.segment<3>(j).array()).any();
            // valid=1, near-touchdown intent=2, near-floor=4, not rising=8,
            // numerical feedback change=16, observer settled=32. No device freshness claim.
            if (changed[leg]) flags_[leg] |= 16;
            if (observer_age_ >= kSettle) flags_[leg] |= 32;
            if (lu.isInvertible() && std::isfinite(rcond_[leg]) && rcond_[leg] > 1e-3) {
                const V3 raw = lu.solve(-s.tau.segment<3>(j));
                const V3 force = lu.solve(residual_.segment<3>(j));
                raw_fz_[leg] = raw.z(); fz_[leg] = force.z();
                if (force.allFinite() && residual_.segment<3>(j).allFinite() &&
                    std::isfinite(mass) && mass > 0.0 && force.norm() < 3.0 * mass * 9.81 &&
                    observer_age_ >= kSettle) flags_[leg] |= 1;
            }
            geometryGates(s, leg);
        }
        // 四条腿先使用同一帧的旧状态计算门控，再统一推进，避免腿顺序影响判断。
        for (int leg = 0; leg < 4; ++leg) {
            advance(s, leg, changed[leg]);
            contact_[leg] = (flags_[leg] & 1) && (state_[leg] == Contact || state_[leg] == Releasing);
        }
        contact = contact_;
        // 以确认后的接触年龄建立滤波信任，避免混用尚未落地时的计划 stance phase。
        // 原 InEKF 仍自行执行原有 65 ms 年龄渐入，不更改 Q/R 或滑移处理。
        previous_M_ = M; previous_v_ = input.qdot; previous_C_ = C;
        previous_momentum_ = momentum; previous_tau_ = s.tau;
        previous_q_ = s.q; previous_qd_ = s.qd;
    }

    void recordEstimate(const Sample& s, const V3& p, const V3& v, bool ready) override {
        // CSV、发布和刷新均在黄色后台线程；I/O 失败只停用日志，不影响滤波或主回路。
        const double now = s.stamp_ns * 1e-9;
        if (settings_.write_csv && !csv_attempted_) {
            csv_attempted_ = true;
            const std::string path = "/tmp/contact_shadow_inekf_" + std::to_string(getpid()) + "_" +
                std::to_string(s.session) + "_" + std::to_string(s.stamp_ns) + ".csv";
            csv_.open(path);
            if (csv_) {
                csv_ << "ros_s,stamp_ns,session,dt_s,worker_age_s,filter_ready,reference_ready,"
                     << "shadow_px,shadow_py,shadow_pz,shadow_vx,shadow_vy,shadow_vz,"
                     << "serving_px,serving_py,serving_pz,serving_vx,serving_vy,serving_vz,"
                     << "force_on_N,force_off_N,confirm_on_s,confirm_off_s,observer_gain_per_s,observer_age_s";
                for (const char* leg : {"FR", "FL", "RR", "RL"})
                    for (const char* key : {"planned", "phase", "raw_proxy_fz_N", "estimated_fz_N", "force_valid",
                         "state", "reason", "gates", "pending_s", "feedback_changes", "rcond", "relative_z_m",
                         "relative_vz_m_s", "estimator_contact", "support_authorized", "transitions"})
                        csv_ << ',' << leg << '_' << key;
                for (const char* key : {"q", "qd", "tau_feedback", "external_tau_residual"})
                    for (int j = 0; j < 12; ++j) csv_ << ',' << key << '_' << j;
                csv_ << '\n' << std::setprecision(17);
                RCLCPP_INFO(node_->get_logger(), "[CONTACT_SHADOW] yellow only; force estimates (not force sensors), CSV=%s", path.c_str());
            } else RCLCPP_ERROR(node_->get_logger(), "[CONTACT_SHADOW] CSV unavailable: %s", path.c_str());
        }
        const V3 estimate_p = ready ? p : V3(kNaN, kNaN, kNaN);
        const V3 estimate_v = ready ? v : V3(kNaN, kNaN, kNaN);
        if (csv_.is_open()) {
            csv_ << now << ',' << s.stamp_ns << ',' << s.session << ',' << s.dt << ',' << s.worker_age << ','
                 << ready << ',' << s.reference_ready;
            for (const V3* x : {&estimate_p, &estimate_v, &s.reference_position, &s.reference_velocity})
                for (int axis = 0; axis < 3; ++axis) csv_ << ',' << (*x)(axis);
            csv_ << ',' << settings_.force_on_N << ',' << settings_.force_off_N << ',' << settings_.confirm_on_s
                 << ',' << settings_.confirm_off_s << ',' << kObserverGain << ',' << observer_age_;
            for (int leg = 0; leg < 4; ++leg) {
                csv_ << ',' << s.planned_contact[leg] << ',' << s.phase[leg] << ',' << raw_fz_[leg] << ',' << fz_[leg]
                     << ',' << ((flags_[leg] & 1) != 0) << ',' << state_[leg] << ',' << reason_[leg] << ',' << flags_[leg]
                     << ',' << pending_s_[leg] << ',' << changes_[leg] << ',' << rcond_[leg] << ',' << relative_z_[leg]
                     << ',' << relative_vz_[leg] << ',' << contact_[leg] << ",0," << transitions_[leg];
            }
            for (const Vec12* x : std::array<const Vec12*, 4>{{&s.q, &s.qd, &s.tau, &residual_}})
                for (int j = 0; j < 12; ++j) csv_ << ',' << (*x)(j);
            csv_ << '\n';
            if (last_flush_s_ < 0.0 || now - last_flush_s_ >= 0.5) {
                csv_.flush(); last_flush_s_ = now;
                if (!csv_) { csv_.close(); RCLCPP_ERROR(node_->get_logger(), "[CONTACT_SHADOW] CSV write failed; logging disabled"); }
            }
        }
        if (last_publish_s_ >= 0.0 && now - last_publish_s_ < 0.04) return;
        last_publish_s_ = now;
        try {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = {now, ready ? 1.0 : 0.0, s.worker_age, observer_age_};
            for (int leg = 0; leg < 4; ++leg)
                msg.data.insert(msg.data.end(), {static_cast<double>(s.planned_contact[leg]), raw_fz_[leg], fz_[leg],
                    static_cast<double>((flags_[leg] & 1) != 0), static_cast<double>(state_[leg]),
                    static_cast<double>(reason_[leg]), static_cast<double>(contact_[leg]), 0.0});
            msg.layout.dim.resize(1);
            msg.layout.dim[0].label = "ros_s,filter_ready,worker_age_s,observer_age_s;"
                "FR/FL/RR/RL:[planned,raw_proxy_fz_N,estimated_fz_N,valid,state,reason,estimator_contact,support_authorized=0]";
            msg.layout.dim[0].size = msg.layout.dim[0].stride = static_cast<std::uint32_t>(msg.data.size());
            diagnostics_->publish(msg);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[CONTACT_SHADOW] diagnostic publish failed: %s", e.what());
        }
    }

private:
    void invalidate() {
        for (int leg = 0; leg < 4; ++leg) {
            transition(leg, Unknown, InvalidForce);
            pending_s_[leg] = 0.0; changes_[leg] = 0; contact_[leg] = 0;
        }
    }
    void transition(int leg, int next, int why) {
        if (state_[leg] != next) { state_[leg] = next; reason_[leg] = why; ++transitions_[leg]; }
    }
    static double median(std::array<double, 4> values, int count) {
        std::sort(values.begin(), values.begin() + count);
        return count % 2 ? values[count / 2] : 0.5 * (values[count / 2 - 1] + values[count / 2]);
    }
    void geometryGates(const Sample& s, int leg) {
        const V3 r = s.R_OB * s.feet_B.col(leg);
        const V3 v = s.R_OB * (s.feet_velocity_B.col(leg) + s.gyro.cross(s.feet_B.col(leg)));
        std::array<double, 4> z{}, vz{};
        int count = 0;
        for (int other = 0; other < 4; ++other) {
            if (other == leg || !(state_[other] == Contact || s.planned_contact[other])) continue;
            z[count] = (s.R_OB * s.feet_B.col(other)).z();
            vz[count++] = (s.R_OB * (s.feet_velocity_B.col(other) + s.gyro.cross(s.feet_B.col(other)))).z();
        }
        relative_z_[leg] = count ? r.z() - median(z, count) : kNaN;
        relative_vz_[leg] = count ? v.z() - median(vz, count) : kNaN;
        if (s.fixed_stance_prior || s.planned_contact[leg] || s.phase[leg] >= 0.65) flags_[leg] |= 2;
        if (count && std::isfinite(relative_z_[leg]) && std::abs(relative_z_[leg]) <= 0.05) flags_[leg] |= 4;
        if (count && std::isfinite(relative_vz_[leg]) && relative_vz_[leg] <= 0.12) flags_[leg] |= 8;
    }
    void advance(const Sample& s, int leg, bool changed) {
        if (!(flags_[leg] & 1)) {
            transition(leg, Unknown, InvalidForce); pending_s_[leg] = 0.0; changes_[leg] = 0; return;
        }
        const bool on = fz_[leg] > settings_.force_on_N && (flags_[leg] & 14) == 14;
        const bool off = fz_[leg] < settings_.force_off_N;
        if (state_[leg] == Contact || state_[leg] == Releasing) {
            if (!off) {
                transition(leg, Contact, Cancelled); pending_s_[leg] = 0.0; changes_[leg] = 0;
            } else {
                transition(leg, Releasing, Candidate);
                pending_s_[leg] += s.dt; changes_[leg] += changed;
                if (pending_s_[leg] >= settings_.confirm_off_s && changes_[leg] >= 2) {
                    const bool unloading = !s.planned_contact[leg] || s.phase[leg] > 0.85;
                    transition(leg, Unloaded, unloading ? PlannedUnload : LoadLost);
                    pending_s_[leg] = 0.0; changes_[leg] = 0;
                }
            }
        } else if (on) {
            transition(leg, Touchdown, Candidate);
            pending_s_[leg] += s.dt; changes_[leg] += changed;
            if (pending_s_[leg] >= settings_.confirm_on_s && changes_[leg] >= 2) {
                transition(leg, Contact, Confirmed); pending_s_[leg] = 0.0; changes_[leg] = 0;
            }
        } else {
            pending_s_[leg] = 0.0; changes_[leg] = 0;
            if (!s.planned_contact[leg] && relative_z_[leg] > 0.01)
                transition(leg, Air, AirMotion);
            else transition(leg, Unloaded, Cancelled);
        }
    }

    static constexpr double kObserverGain = 80.0; // s^-1，残差观测器带宽起点，不是控制增益。
    static constexpr double kSettle = 0.12, kMaxGap = 0.04;
    std::string base_;
    std::vector<std::string> feet_;
    const ContactShadowInEKF::Settings settings_;
    std::unique_ptr<sysu219::wbc::FloatingBaseModel> model_;
    Mat18 previous_M_ = Mat18::Zero();
    Vec18 previous_v_ = Vec18::Zero(), previous_C_ = Vec18::Zero();
    Vec12 previous_momentum_ = Vec12::Zero(), previous_tau_ = Vec12::Zero();
    Vec12 previous_q_ = Vec12::Zero(), previous_qd_ = Vec12::Zero(), residual_ = Vec12::Zero();
    bool have_momentum_ = false, csv_attempted_ = false;
    double observer_age_ = 0.0, last_publish_s_ = -1.0, last_flush_s_ = -1.0;
    std::array<int, 4> state_{}, reason_{}, flags_{}, changes_{}, transitions_{}, contact_{};
    std::array<double, 4> pending_s_{}, raw_fz_{}, fz_{}, rcond_{}, relative_z_{}, relative_vz_{};
    std::ofstream csv_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_;
};
} // namespace

class ContactShadowInEKF::Impl {
public:
    Impl(const std::vector<std::string>& joints, const std::vector<std::string>& feet,
         const std::string& base, const std::string& imu, const Settings& settings)
        : observer(std::make_shared<ForceContactObserver>(base, feet, settings)),
          filter(joints, feet, base, imu, observer) {}
    std::shared_ptr<ForceContactObserver> observer;
    ShadowInEKF filter;
};

ContactShadowInEKF::ContactShadowInEKF(const std::vector<std::string>& joints,
    const std::vector<std::string>& feet, const std::string& base, const std::string& imu,
    const Settings& settings) : impl_(std::make_unique<Impl>(joints, feet, base, imu, settings)) {}
ContactShadowInEKF::~ContactShadowInEKF() = default;
void ContactShadowInEKF::setRobotDescription(const std::string& xml) { impl_->filter.setRobotDescription(xml); }
void ContactShadowInEKF::startSession(const CtrlInterfaces& io) { impl_->filter.startSession(io); }
void ContactShadowInEKF::stopSession() noexcept { impl_->filter.stopSession(); }
void ContactShadowInEKF::submit(const rclcpp::Time& time, const rclcpp::Duration& period,
    const CtrlInterfaces& io, const WaveGenerator& wave, const Estimator& reference,
    bool fixed_stance_prior, bool reference_ready) noexcept {
    impl_->filter.submit(time, period, io, wave, reference, fixed_stance_prior, reference_ready);
}
} // namespace sysu219_guide_controller
