#pragma once

#include <sysu219_guide_controller/common/mathTypes.h>
#include <sysu219_guide_controller/common/mathTools.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <thread>
#include <string>
#include <unistd.h>
#include <vector>

// 仅诊断：控制线程写预分配缓冲，后台线程导出；不改变控制命令。
class TrottingDebug {
public:
    enum Meta { CYCLE, ROS_S, STEADY_S, SYSTEM_S, PHASE_CONTROL_S, PERIOD_S,
                WALL_DT_MS, UPDATE_MS, MODE, WAVE, FLAGS, EVENT,
                SOLVER_ID, SOLVER_STATUS, SOLVER_ITER, SOLVER_RESULT, MPC_TOTAL_MS, META_COUNT };
    // wbc_flags为位标志，可同时成立；只按成功位读取对应数据，未执行项记NaN。
    enum WbcFlags { WBC_ATTEMPT = 1, WBC_OK = 2, RELAX_ATTEMPT = 4, RELAX_OK = 8,
                    WBC_APPLIED = 16, TORQUE_REJECTED = 32, COMMAND_CLAMPED = 64,
                    WBC_ENABLED = 128, MPC_FRAME_VALID = 256, MODEL_READY = 512,
                    BLEND_VALID = 1024 };
    template<int N> using Samples = Eigen::Matrix<float, N, 1>;
    // 只保存诊断用float，不回写控制；固定容量，不缓存M或雅可比等大矩阵。
    struct WbcSample {
        unsigned flags = 0;
        float blend = 0.0f, model_wbc_ms = 0.0f, relaxation_ms = 0.0f;
        float equality_residual = std::nanf(""), inequality_margin = std::nanf("");
        Eigen::Matrix<int, 4, 1> rank = Eigen::Matrix<int, 4, 1>::Constant(-1);
        // 四级顺序：支撑足、机身转动、机身平动、摆动足；误差为各任务向量的范数。
        Samples<4> position_residual = Samples<4>::Constant(std::nanf(""));
        Samples<4> velocity_residual = Samples<4>::Constant(std::nanf(""));
        Samples<4> acceleration_residual = Samples<4>::Constant(std::nanf(""));
        Samples<18> qddot = Samples<18>::Constant(std::nanf(""));
        Samples<6> delta_qddot = Samples<6>::Constant(std::nanf(""));
        Samples<12> delta_force = Samples<12>::Constant(std::nanf(""));
        Samples<12> stance_acceleration = Samples<12>::Constant(std::nanf(""));
        // 原控制已限幅的参考，用于与实际发送的q_cmd/qd_cmd/tau_ff_cmd逐关节比较。
        Samples<12> legacy_q = Samples<12>::Constant(std::nanf(""));
        Samples<12> legacy_qd = Samples<12>::Constant(std::nanf(""));
        Samples<12> legacy_tau = Samples<12>::Constant(std::nanf(""));
        // 完整逆动力学前馈相对原前馈的差值；被回退时也保留，不代表实际发送。
        Samples<12> candidate_tau_delta = Samples<12>::Constant(std::nanf(""));
        Samples<3> omega_d = Samples<3>::Constant(std::nanf(""));
    };
    // event 位：1时间、2估计、4足端目标、8关节目标、16运动学、32姿态、64非法值/异常、128相位、256退出、512起步、1024接触切换。
    struct Frame {
        std::array<double, META_COUNT> meta{};
        Eigen::Matrix<double, 10, 1> imu = Eigen::Matrix<double, 10, 1>::Zero();
        Vec3 rpy = Vec3::Zero(), p = Vec3::Zero(), v = Vec3::Zero();
        Vec3 p_ref = Vec3::Zero(), v_ref = Vec3::Zero();
        Vec3 rpy_ref = Vec3::Zero(), gyro_G = Vec3::Zero();
        Vec3 a_ref = Vec3::Zero(), alpha_ref = Vec3::Zero();
        Vec3 v_predicted = Vec3::Zero(), v_unfiltered = Vec3::Zero(), acc_G = Vec3::Zero();
        VecInt4 contact = VecInt4::Zero(), transition = VecInt4::Zero();
        Vec4 phase = Vec4::Zero();
        Vec34 feet_G = Vec34::Zero(), feet_B = Vec34::Zero();
        Vec34 feet_v_G = Vec34::Zero();
        Vec34 ground_force_P = Vec34::Zero(); // 求解器规划的地面对机器人接触力（N），不是实测力。
        Vec34 goal_G = Vec34::Zero(), vgoal_G = Vec34::Zero();
        Vec34 goal_B = Vec34::Zero(), vgoal_B = Vec34::Zero();
        Vec34 start_G = Vec34::Zero(), end_G = Vec34::Zero(), hold_G = Vec34::Zero();
        Vec12 q = Vec12::Zero(), qd = Vec12::Zero(), q_raw = Vec12::Zero();
        Vec12 qd_raw = Vec12::Zero(), q_cmd = Vec12::Zero(), qd_cmd = Vec12::Zero();
        VecInt4 ik_q = VecInt4::Zero(), ik_qd = VecInt4::Zero();
        Vec4 fk_q = Vec4::Zero(), fk_qd = Vec4::Zero(), sigma_qd = Vec4::Zero();
        // 全部只用于诊断，不回写状态估计/控制目标。差分使用实际 ROS 时间间隔。
        double check_dt_s = 0.0, support_line_distance_m = -1.0;
        Vec3 com_p = Vec3::Zero();
        // 差分、积分、力矩分解改为离线计算，避免同一信息重复占用缓存和CSV。
        WbcSample wbc;

        // effort 为电机反馈估算力矩；缺失接口记 NaN，不能当作零力矩。
        Vec12 tau_feedback = Vec12::Constant(std::nan(""));
        Vec12 tau_ff_cmd = Vec12::Zero(), kp_cmd = Vec12::Zero(), kd_cmd = Vec12::Zero();
        // 落地候选仅诊断，严禁用于接触切换；代理力未扣除腿重力、惯性及反馈延迟。
        Vec4 touchdown_fz_proxy = Vec4::Constant(std::nan(""));
        Vec4 touchdown_candidate_s = Vec4::Zero();
        VecInt4 touchdown_gates = VecInt4::Zero(); // 位1末段、2高度、4下降误差、8相对速度、16代理力；全通过=31。
        VecInt4 touchdown_candidate = VecInt4::Zero();
        double touchdown_fz_threshold_N = 0.0;
    };

    explicit TrottingDebug(double dt)
        : dt_(dt), pre_(static_cast<size_t>(std::ceil(2.0 / dt))),
          post_(static_cast<size_t>(std::ceil(1.0 / dt))),
          ring_(std::max(pre_ + post_ + 1, static_cast<size_t>(std::ceil(10.0 / dt)) + 1)),
          worker_([this] { writeLoop(); }) {}
    ~TrottingDebug() {
        finish();
        stop_.store(true);
        worker_.join();
    }
    TrottingDebug(const TrottingDebug&) = delete;
    TrottingDebug& operator=(const TrottingDebug&) = delete;

    void reset() { reset_pending_ = true; }
    bool busy() const { return ready_.load(std::memory_order_acquire); }
    void push(Frame f) {
        if (ready_.load(std::memory_order_acquire)) return;
        const bool startup = reset_pending_;
        if (reset_pending_ || awaiting_dump_) {
            head_ = count_ = remaining_ = 0;
            // 起步保存十秒，覆盖起步和随后稳定阶段；普通重置不重复启动录制。
            if (startup) remaining_ = ring_.size();
            reset_pending_ = awaiting_dump_ = have_previous_ = false;
        }
        unsigned event = startup ? 512u : 0u;
        const RotMat rotation = rotz(f.rpy(2)) * roty(f.rpy(1)) * rotx(f.rpy(0));
        int support_count = 0;
        std::array<int, 2> support_legs{};
        for (int leg = 0; leg < 4; ++leg) {
            if (f.contact(leg) == 1) {
                if (support_count < 2) support_legs[support_count] = leg;
                ++support_count;
            }
        }
        if (support_count == 2) {
            const Vec2 a = (f.hold_G.col(support_legs[0]) - f.com_p).head<2>();
            const Vec2 b = (f.hold_G.col(support_legs[1]) - f.com_p).head<2>();
            const Vec2 delta = b - a;
            if (delta.norm() > 1e-9)
                f.support_line_distance_m = std::abs(delta(0) * a(1) - delta(1) * a(0)) / delta.norm();
        }
        const bool finite = f.imu.allFinite() && f.p.allFinite() && f.v.allFinite() &&
            f.rpy.allFinite() && f.goal_G.allFinite() && f.vgoal_G.allFinite() &&
            f.goal_B.allFinite() && f.vgoal_B.allFinite() && f.q_raw.allFinite() &&
            f.qd_raw.allFinite() && f.q.allFinite() && f.qd.allFinite() &&
            f.q_cmd.allFinite() && f.qd_cmd.allFinite() && f.phase.allFinite() &&
            f.feet_G.allFinite() && f.feet_B.allFinite() && f.start_G.allFinite() &&
            f.end_G.allFinite() && f.hold_G.allFinite() && f.fk_q.allFinite() &&
            f.fk_qd.allFinite() && f.sigma_qd.allFinite() && f.rpy_ref.allFinite() &&
            f.gyro_G.allFinite() && f.a_ref.allFinite() && f.alpha_ref.allFinite() &&
            f.feet_v_G.allFinite() && f.ground_force_P.allFinite() &&
            f.v_predicted.allFinite() && f.v_unfiltered.allFinite() && f.acc_G.allFinite();
        // 新链路失败也按异常沿触发留档，持续失败不重复刷文件；未执行不算失败。
        const bool wbc_failed = ((f.wbc.flags & WBC_ATTEMPT) && !(f.wbc.flags & WBC_OK)) ||
            ((f.wbc.flags & RELAX_ATTEMPT) && !(f.wbc.flags & RELAX_OK)) ||
            (f.wbc.flags & TORQUE_REJECTED);
        const unsigned bad = (!finite || wbc_failed || f.meta[FLAGS] != 0 ? 64u : 0u) |
            (f.fk_q.maxCoeff() > 0.01 || f.fk_qd.maxCoeff() > 0.01 ||
             f.sigma_qd.minCoeff() < 1e-3 ? 16u : 0u) |
            (std::abs(f.rpy(0)) > 0.12 || std::abs(f.rpy(1)) > 0.20 ? 32u : 0u);
        if (have_previous_) {
            const double wall_dt = f.meta[STEADY_S] - previous_.meta[STEADY_S];
            f.meta[WALL_DT_MS] = wall_dt * 1000.0;
            const double ros_dt = f.meta[ROS_S] - previous_.meta[ROS_S];
            // 跨回退/大间断不做差分；第一帧 check_dt_s=0 表示不可用。
            if (std::isfinite(ros_dt) && ros_dt > 0.0 && ros_dt <= 3.0 * dt_) {
                f.check_dt_s = ros_dt;
                // 用支撑脚的平均高度/速度作比较，公共估计位置/速度在差值中抵消。
                if (support_count >= 2 && f.meta[MODE] == 1) {
                    double support_z = 0.0, support_vz = 0.0;
                    for (int leg = 0; leg < 4; ++leg) if (f.contact(leg) == 1) {
                        support_z += (rotation * f.feet_B.col(leg))(2);
                        support_vz += f.feet_v_G(2, leg);
                    }
                    support_z /= support_count;
                    support_vz /= support_count;
                    for (int leg = 0; leg < 4; ++leg) {
                        const double height = (rotation * f.feet_B.col(leg))(2) - support_z;
                        int gates = 0;
                        if (f.contact(leg) == 0 && f.phase(leg) >= 0.85 && f.phase(leg) < 1.0) gates |= 1;
                        if (std::isfinite(height) && height >= -0.005 && height <= 0.025) gates |= 2;
                        if (f.goal_G(2, leg) - f.feet_G(2, leg) < -0.008 && f.vgoal_G(2, leg) < -0.05) gates |= 4;
                        if (std::isfinite(f.feet_v_G(2, leg)) && std::abs(f.feet_v_G(2, leg) - support_vz) < 0.05) gates |= 8;
                        if (std::isfinite(f.touchdown_fz_proxy(leg)) && f.touchdown_fz_threshold_N > 0.0 &&
                            f.touchdown_fz_proxy(leg) >= f.touchdown_fz_threshold_N) gates |= 16;
                        f.touchdown_gates(leg) = gates;
                        if (gates == 31 && previous_.contact(leg) == 0 && f.phase(leg) >= previous_.phase(leg))
                            f.touchdown_candidate_s(leg) = ros_dt + (previous_.touchdown_gates(leg) == 31
                                ? previous_.touchdown_candidate_s(leg) : 0.0);
                        f.touchdown_candidate(leg) = f.touchdown_candidate_s(leg) >= 0.012 - 1e-9;
                    }
                }
            }
            const double system_dt = f.meta[SYSTEM_S] - previous_.meta[SYSTEM_S];
            if (wall_dt > 3 * dt_ || ros_dt <= 0 || ros_dt > 3 * dt_ ||
                f.meta[PERIOD_S] > 3 * dt_ || std::abs(system_dt - wall_dt) > 0.002)
                event |= 1;
            if ((f.p - previous_.p).norm() > 0.015 || (f.v - previous_.v).norm() > 0.3)
                event |= 2;
            if ((f.goal_G - previous_.goal_G).cwiseAbs().maxCoeff() > 0.02) event |= 4;
            if ((f.q_raw - previous_.q_raw).cwiseAbs().maxCoeff() > 0.05 ||
                (f.qd_raw - previous_.qd_raw).cwiseAbs().maxCoeff() > 5.0) event |= 8;
            for (int leg = 0; leg < 4; ++leg) {
                if (f.contact(leg) != previous_.contact(leg)) event |= 1024;
                if (f.contact(leg) == previous_.contact(leg) &&
                    (f.phase(leg) < previous_.phase(leg) - 0.05 ||
                     f.phase(leg) > previous_.phase(leg) + 0.05)) event |= 128;
                if (f.ik_q(leg) < 0 && previous_.ik_q(leg) >= 0) event |= 16;
                if (f.ik_qd(leg) < 0 && previous_.ik_qd(leg) >= 0) event |= 16;
            }
            event |= bad & ~previous_bad_; // 持续的 IK 误差只记一次，避免反复导出。
        }
        f.meta[EVENT] = event;
        previous_ = f;
        previous_bad_ = bad;
        have_previous_ = true;
        ring_[head_] = f;
        head_ = (head_ + 1) % ring_.size();
        count_ = std::min(count_ + 1, remaining_ ? ring_.size() : pre_ + 1);
        if (remaining_) {
            if (--remaining_ == 0) publish();
        } else if ((event & ~1024u) && count_ >= pre_ + 1) {
            // 正常接触切换记入 CSV，但不单独触发反复导出。
            remaining_ = post_;
        }
    }
    // 退出时也保存最后一段，即使尚未触发或未录满后 1 秒。
    void finish() {
        if (!ready_.load(std::memory_order_acquire) && !awaiting_dump_ && count_) {
            ring_[(head_ + ring_.size() - 1) % ring_.size()].meta[EVENT] += 256;
            publish();
        }
    }

private:
    void publish() {
        awaiting_dump_ = true;
        ready_.store(true, std::memory_order_release); // 后台读取期间冻结缓冲。
    }
    static void row(std::ostream& out, const Frame& f, bool header) {
        static const char* names[] = {"cycle", "ros_s", "steady_s", "system_s", "phase_control_s",
            "period_s", "wall_dt_ms", "update_ms", "mode", "wave", "flags", "event",
            "solver_id", "solver_status", "solver_iter", "solver_result", "mpc_total_ms"};
        for (size_t i = 0; i < f.meta.size(); ++i) {
            if (i) out << ',';
            if (header) out << names[i]; else out << f.meta[i];
        }
        auto field = [&](const char* name, const auto& value) {
            for (int i = 0; i < value.size(); ++i) {
                out << ',';
                if (header) out << name << '_' << i; else out << value.data()[i];
            }
        };
        // 所有矩阵按列展开；四腿顺序 FR、FL、RR、RL，每腿 xyz / 三关节。
        field("imu", f.imu); field("rpy", f.rpy); field("p", f.p); field("v", f.v);
        field("p_ref", f.p_ref); field("v_ref", f.v_ref);
        field("rpy_ref", f.rpy_ref); field("gyro_G", f.gyro_G);
        field("a_ref", f.a_ref); field("alpha_ref", f.alpha_ref);
        field("v_predicted", f.v_predicted); field("v_unfiltered", f.v_unfiltered); field("acc_G", f.acc_G);
        field("contact", f.contact); field("transition", f.transition); field("phase", f.phase);
        field("feet_G", f.feet_G); field("feet_B", f.feet_B);
        field("feet_v_G", f.feet_v_G); field("ground_force_P", f.ground_force_P);
        field("goal_G", f.goal_G); field("vgoal_G", f.vgoal_G);
        field("goal_B", f.goal_B); field("vgoal_B", f.vgoal_B);
        field("start_G", f.start_G); field("end_G", f.end_G); field("hold_G", f.hold_G);
        field("q", f.q); field("qd", f.qd); field("q_raw", f.q_raw); field("qd_raw", f.qd_raw);
        field("q_cmd", f.q_cmd); field("qd_cmd", f.qd_cmd);
        field("ik_q", f.ik_q); field("ik_qd", f.ik_qd);
        field("fk_q", f.fk_q); field("fk_qd", f.fk_qd); field("sigma_qd", f.sigma_qd);
        field("com_p", f.com_p);
        if (header) out << ",check_dt_s,support_line_distance_m";
        else out << ',' << f.check_dt_s << ',' << f.support_line_distance_m;
        field("tau_feedback", f.tau_feedback); field("tau_ff_cmd", f.tau_ff_cmd);
        field("kp_cmd", f.kp_cmd); field("kd_cmd", f.kd_cmd);
        field("touchdown_fz_proxy", f.touchdown_fz_proxy);
        field("touchdown_gates", f.touchdown_gates);
        field("touchdown_candidate_s", f.touchdown_candidate_s);
        field("touchdown_candidate", f.touchdown_candidate);
        if (header) out << ",touchdown_fz_threshold_N";
        else out << ',' << f.touchdown_fz_threshold_N;
        // 新字段保留有符号修正和实际输出基准；仅有范数无法判断反馈方向。
        auto scalar = [&](const char* name, auto value) {
            out << ',';
            if (header) out << name; else out << value;
        };
        const auto& w = f.wbc;
        // float诊断保留9位有效数字，足以往返恢复；原double字段仍用17位。
        out << std::setprecision(9);
        scalar("wbc_flags", w.flags); scalar("wbc_blend", w.blend);
        scalar("model_wbc_ms", w.model_wbc_ms); scalar("relaxation_ms", w.relaxation_ms);
        scalar("relax_equality_residual", w.equality_residual);
        scalar("relax_inequality_margin", w.inequality_margin);
        field("wbc_rank", w.rank);
        field("wbc_position_residual", w.position_residual);
        field("wbc_velocity_residual", w.velocity_residual);
        field("wbc_acceleration_residual", w.acceleration_residual);
        field("wbc_qddot", w.qddot); field("relax_delta_qddot", w.delta_qddot);
        field("relax_delta_force", w.delta_force);
        field("relax_stance_acceleration", w.stance_acceleration);
        field("legacy_q", w.legacy_q); field("legacy_qd", w.legacy_qd);
        field("legacy_tau", w.legacy_tau); field("wbc_candidate_tau_delta", w.candidate_tau_delta);
        field("wbc_omega_d_O", w.omega_d);
        out << std::setprecision(17) << '\n';
    }
    void writeLoop() {
        while (!stop_.load() || ready_.load(std::memory_order_acquire)) {
            if (!ready_.load(std::memory_order_acquire)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            try {
                const size_t begin = (head_ + ring_.size() - count_) % ring_.size();
                const auto stamp = std::chrono::system_clock::now().time_since_epoch();
                const std::string path = "/tmp/trotting_debug_" + std::to_string(getpid()) + "_" +
                    std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(stamp).count()) + ".csv";
                std::ofstream out(path);
                out.exceptions(std::ios::failbit | std::ios::badbit);
                out << std::setprecision(17);
                row(out, ring_[begin], true);
                for (size_t i = 0; i < count_; ++i) row(out, ring_[(begin + i) % ring_.size()], false);
                out.close();
                size_t wbc_success = 0, relax_success = 0, applied = 0, rejected = 0;
                for (size_t i = 0; i < count_; ++i) {
                    const auto flags = ring_[(begin + i) % ring_.size()].wbc.flags;
                    wbc_success += !!(flags & WBC_OK);
                    relax_success += !!(flags & RELAX_OK);
                    applied += !!(flags & WBC_APPLIED);
                    rejected += !!(flags & TORQUE_REJECTED);
                }
                RCLCPP_INFO(rclcpp::get_logger("TrottingDebug"),
                    "[TROT_DEBUG] %s采样已保存：%s，共%zu帧，周期%.0f..%.0f；WBC成功%zu，松弛成功%zu，采用%zu，力矩回退%zu",
                    (static_cast<unsigned>(ring_[begin].meta[EVENT]) & 512u) ? "起步" : "事件",
                    path.c_str(), count_, ring_[begin].meta[CYCLE],
                    ring_[(head_ + ring_.size() - 1) % ring_.size()].meta[CYCLE],
                    wbc_success, relax_success, applied, rejected);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("TrottingDebug"), "[TROT_DEBUG] CSV保存失败：%s", e.what());
            }
            ready_.store(false, std::memory_order_release);
        }
    }
    double dt_;
    size_t pre_, post_, head_ = 0, count_ = 0, remaining_ = 0;
    std::vector<Frame, Eigen::aligned_allocator<Frame>> ring_;
    Frame previous_;
    unsigned previous_bad_ = 0;
    bool have_previous_ = false, reset_pending_ = false, awaiting_dump_ = false;
    std::atomic<bool> ready_{false}, stop_{false};
    std::thread worker_;
};
