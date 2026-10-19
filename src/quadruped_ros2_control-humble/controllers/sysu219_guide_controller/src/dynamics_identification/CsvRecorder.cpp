#include "sysu219_guide_controller/dynamics_identification/CsvRecorder.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <utility>
#include <unistd.h>

namespace leg_identification {
namespace {
void array(std::ostream& out, const Vec& values) {
    out << '[' << values[0] << ',' << values[1] << ',' << values[2] << ']';
}
}

CsvRecorder::CsvRecorder(std::string root, Vec center, double kp, double kd)
    : ring_(std::make_unique<std::array<Sample, capacity>>()), root_(std::move(root)),
      center_(center), kp_(kp), kd_(kd), writer_(&CsvRecorder::work, this) {}

CsvRecorder::~CsvRecorder() {
    accepting_.store(false);
    shutdown_.store(true);
    if (writer_.joinable()) writer_.join();
}

void CsvRecorder::setDescription(const std::string& xml) {
    std::lock_guard<std::mutex> lock(description_mutex_);
    description_ = xml;
    description_ready_.store(true, std::memory_order_release);
}

void CsvRecorder::begin() {
    accepting_.store(true);
    requested_.store(true, std::memory_order_release);
}

void CsvRecorder::finish() { accepting_.store(false); }

void CsvRecorder::report(const std::string& text) {
    std::lock_guard<std::mutex> lock(notice_mutex_);
    if (!notice_.empty()) notice_ += '\n';
    notice_ += text;
}

std::string CsvRecorder::takeNotice() {
    std::lock_guard<std::mutex> lock(notice_mutex_);
    std::string text;
    text.swap(notice_);
    return text;
}

void CsvRecorder::checkpoint(std::string label, uint64_t first, uint64_t last) {
    std::lock_guard<std::mutex> lock(checkpoint_mutex_);
    checkpoints_.push_back({std::move(label), first, last});
}

bool CsvRecorder::push(const Sample& sample) {
    if (!accepting_.load() || failed_.load()) return false;
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t next = (head + 1) % capacity;
    if (next == tail_.load(std::memory_order_acquire)) {
        ++dropped_;
        failed_.store(true);
        return false;
    }
    (*ring_)[head] = sample;
    head_.store(next, std::memory_order_release);
    return true;
}

void CsvRecorder::work() {
    using namespace std::chrono_literals;
    while (!shutdown_.load() && (!requested_.load(std::memory_order_acquire) ||
                                 !description_ready_.load(std::memory_order_acquire)))
        std::this_thread::sleep_for(10ms);
    if (!requested_.load() || !description_ready_.load()) return;
    std::ofstream csv;
    std::filesystem::path folder;
    uint64_t written = 0, previous_sequence = 0, sequence_gaps = 0, repeated = 0, flagged = 0;
    int64_t first_ns = 0, previous_ns = 0, last_ns = 0;
    double min_dt = 1e30, max_dt = 0.0;
    Vec previous_q{}, previous_dq{}, previous_tau{};
    try {
        const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        folder = std::filesystem::path(root_) / ("leg_id_" + std::to_string(stamp) + "_" + std::to_string(getpid()));
        if (!std::filesystem::create_directories(folder)) throw std::runtime_error("记录目录已存在");
        {
            std::lock_guard<std::mutex> lock(description_mutex_);
            std::ofstream urdf(folder / "robot_description.urdf");
            urdf.exceptions(std::ios::failbit | std::ios::badbit);
            urdf << description_;
        }
        csv.exceptions(std::ios::failbit | std::ios::badbit);
        csv.open(folder / "samples.csv");
        csv << "time_s,steady_ns,system_ns,ros_ns,period_s,dt_s,sequence,action,phase,trial,phase_time_s,flags,repeated_feedback";
        for (const char* joint : {"FL_hip", "FL_thigh", "FL_calf"})
            for (const char* field : {"q", "dq", "tau", "q_cmd", "dq_cmd", "tau_ff", "kp", "kd"})
                csv << ',' << joint << '_' << field;
        csv << ",imu_qw,imu_qx,imu_qy,imu_qz,imu_wx,imu_wy,imu_wz,imu_ax,imu_ay,imu_az\n";
        csv << std::setprecision(17);
        // Save metadata immediately, including for a recording interrupted by a process crash.
        const auto metadata = [&](bool complete) {
            std::ofstream out(folder / "metadata.json.tmp");
            out.exceptions(std::ios::failbit | std::ios::badbit);
            out << std::setprecision(17)
                << "{\n  \"schema_version\": 4,\n  \"closed_cleanly\": " << (complete ? "true" : "false")
                << ",\n  \"nominal_sample_rate_hz\": 250,\n  \"timestamp_kind\": \"controller interface snapshot; not motor receipt or measurement time\","
                << "\n  \"primary_clock\": \"std::chrono::steady_clock\","
                << "\n  \"units\": {\"q\":\"rad\",\"dq\":\"rad/s\",\"tau\":\"N*m\",\"imu_angular_velocity\":\"rad/s\",\"imu_acceleration\":\"m/s^2\"},"
                << "\n  \"joint_order\": [\"FL_hip_joint\",\"FL_thigh_joint\",\"FL_calf_joint\"],"
                << "\n  \"motor_model_in_existing_configuration\": \"DM10010\","
                << "\n  \"control_mode\": \"MIT; existing mapped output-shaft joint coordinates\","
                << "\n  \"feedback_torque\": \"existing effort estimate; no extra scaling or calibration\","
                << "\n  \"command_sampling_point\": \"controller output before existing hardware clamp and protocol quantization\","
                << "\n  \"filtering\": \"none; no measured acceleration computed\","
                << "\n  \"hardware_limits_source\": \"HardwareSysu219.cpp on_init (read only)\","
                << "\n  \"urdf_snapshot\": \"robot_description.urdf\",\n  \"center_rad\": "; array(out, center_);
            out << ",\n  \"amplitudes_rad\": "; array(out, amplitudes);
            out << ",\n  \"hardware_lower_rad\": "; array(out, hardware_lower);
            out << ",\n  \"hardware_upper_rad\": "; array(out, hardware_upper);
            out << ",\n  \"hip_excitation_lower_rad\": " << hip_lower
                << ",\n  \"hip_excitation_upper_rad\": " << hip_upper
                << ",\n  \"hip_profile\": \"positive shoulder-raising side only; e(t)*A/2*(1+s(t)); s(t) in [-1,1]; q=0 at start/end\""
                << ",\n  \"hip_window_scope\": \"excitation only; initial preparation uses existing hardware/URDF limits\""
                << ",\n  \"position_fault_margin_rad\": " << position_fault_margin
                << ",\n  \"measured_velocity_fault_rad_s\": " << measured_velocity_limit
                << ",\n  \"stop_behavior\": \"latch current finite position once; retain gains; never clear left-front gains on an identification stop\""
                << ",\n  \"ready_hold_s\": " << ready_hold_seconds
                << ",\n  \"ready_position_error_rad\": "; array(out, ready_position_error);
            out << ",\n  \"ready_velocity_rad_s\": "; array(out, ready_velocity);
            out
                << ",\n  \"joint_limit_margin_rad\": " << margin
                << ",\n  \"stand_kp\": " << kp_ << ",\n  \"stand_kd\": " << kd_
                << ",\n  \"transition_s\": " << transition_seconds << ",\n  \"gain_ramp_s\": " << gain_seconds
                << ",\n  \"interrupted_transfer_candidates_s\": [4,2,1]"
                << ",\n  \"motion_time_scale\": " << motion_time_scale
                << ",\n  \"single_joint_frequency_hz\": " << single_frequency
                << ",\n  \"combined_frequencies_hz\": "; array(out, train_frequency);
            out << ",\n  \"validation_base_frequencies_hz\": "; array(out, test_frequency);
            out << ",\n  \"validation_second_trial_frequency_shift_hz\": " << validation_frequency_shift
                << ",\n  \"multisine_weights\": "; array(out, multisine_weights);
            out << ",\n  \"envelope_ramp_s\": " << envelope_seconds
                << ",\n  \"phase_formula\": \"0.7*j + 0.4*trial + 0.3*j*k; validation adds 0.9 rad\""
                << ",\n  \"actions\": {\"0\":\"hold\",\"1\":\"static\",\"2\":\"hip\",\"3\":\"thigh\",\"4\":\"calf\",\"5\":\"combined\",\"6\":\"validation\"},"
                << "\n  \"phases\": {\"0\":\"stopped\",\"1\":\"gain_ramp\",\"2\":\"transition\",\"3\":\"ready_hold\",\"4\":\"experiment\",\"5\":\"settling\",\"6\":\"holding_after_stop\"},"
                << "\n  \"flag_bits\": {\"1\":\"stopped_or_holding\",\"2\":\"nonfinite_joint_feedback\",\"4\":\"imu_missing_or_nonfinite\",\"8\":\"sample_interval_abnormal\"},"
                << "\n  \"repeated_feedback_note\": \"exact repetition is observable; it does not prove stale CAN data\","
                << "\n  \"written_samples\": " << written << ",\n  \"sequence_gaps\": " << sequence_gaps
                << ",\n  \"queue_dropped\": " << dropped_.load() << ",\n  \"flagged_samples\": " << flagged
                << ",\n  \"exact_feedback_repetitions\": " << repeated
                << ",\n  \"first_steady_ns\": " << first_ns << ",\n  \"last_steady_ns\": " << last_ns
                << ",\n  \"min_dt_s\": " << (written > 1 ? min_dt : 0.0)
                << ",\n  \"max_dt_s\": " << max_dt
                << ",\n  \"average_sample_rate_hz\": " << (last_ns > first_ns ? (written-1)*1e9/(last_ns-first_ns) : 0.0)
                << ",\n  \"writer_failed\": " << (failed_.load() ? "true" : "false") << "\n}\n";
            out.close();
            std::filesystem::rename(folder / "metadata.json.tmp", folder / "metadata.json");
        };
        metadata(false);
        report("CSV保存目录：" + folder.string());
        ready_.store(true, std::memory_order_release);
        const auto flush_checkpoints = [&] {
            for (;;) {
                Checkpoint point{};
                {
                    std::lock_guard<std::mutex> lock(checkpoint_mutex_);
                    if (checkpoints_.empty() || written == 0 || previous_sequence < checkpoints_.front().last) break;
                    point = std::move(checkpoints_.front());
                    checkpoints_.pop_front();
                }
                csv.flush(); metadata(false);
                report(point.label + "：数据记录完成，已写入并刷新CSV：" + (folder / "samples.csv").string() +
                       "；正式段样本序号 [" + std::to_string(point.first) + ", " + std::to_string(point.last) + "]。");
            }
        };
        auto last_flush = std::chrono::steady_clock::now();
        for (;;) {
            flush_checkpoints();
            const size_t tail = tail_.load(std::memory_order_relaxed);
            if (tail == head_.load(std::memory_order_acquire)) {
                if (shutdown_.load() || !accepting_.load()) break;
                if (std::chrono::steady_clock::now() - last_flush > 1s) {
                    csv.flush(); metadata(false); last_flush = std::chrono::steady_clock::now();
                }
                std::this_thread::sleep_for(2ms);
                continue;
            }
            const Sample s = (*ring_)[tail];
            tail_.store((tail + 1) % capacity, std::memory_order_release);
            if (written == 0) first_ns = s.steady_ns;
            const double dt = written ? (s.steady_ns - previous_ns)*1e-9 : 0.0;
            if (written) {
                if (s.sequence > previous_sequence + 1) sequence_gaps += s.sequence - previous_sequence - 1;
                if (s.steady_ns <= previous_ns) throw std::runtime_error("单调时间戳未递增");
                min_dt = std::min(min_dt, dt); max_dt = std::max(max_dt, dt);
            }
            const bool identical = written && s.q == previous_q && s.dq == previous_dq && s.tau == previous_tau;
            repeated += identical; flagged += s.flags != 0;
            csv << (s.steady_ns-first_ns)*1e-9 << ',' << s.steady_ns << ',' << s.system_ns << ',' << s.ros_ns
                << ',' << s.period_s << ',' << dt << ',' << s.sequence << ',' << s.action << ',' << s.phase
                << ',' << s.trial << ',' << s.phase_time_s << ',' << s.flags << ',' << identical;
            for (size_t j = 0; j < 3; ++j)
                csv << ',' << s.q[j] << ',' << s.dq[j] << ',' << s.tau[j] << ',' << s.q_cmd[j]
                    << ',' << s.dq_cmd[j] << ',' << s.tau_ff[j] << ',' << s.kp[j] << ',' << s.kd[j];
            for (double value : s.imu) csv << ',' << value;
            csv << '\n';
            ++written; previous_ns = last_ns = s.steady_ns; previous_sequence = s.sequence;
            previous_q = s.q; previous_dq = s.dq; previous_tau = s.tau;
            if (std::chrono::steady_clock::now() - last_flush > 1s) {
                csv.flush(); metadata(false); last_flush = std::chrono::steady_clock::now();
            }
        }
        csv.flush(); flush_checkpoints(); metadata(true);
        report("记录结束：" + std::to_string(written) + "行，队列丢失=" + std::to_string(dropped_.load()));
    } catch (const std::exception& e) {
        failed_.store(true);
        report("记录失败：" + std::string(e.what()) + "，目录=" + folder.string());
    }
    ready_.store(false, std::memory_order_release);
}
}  // namespace leg_identification
