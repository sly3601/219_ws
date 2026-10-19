#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include "Trajectory.h"

namespace leg_identification {
struct Sample {
    uint64_t sequence = 0;
    int64_t steady_ns = 0, system_ns = 0, ros_ns = 0;
    double period_s = 0.0, phase_time_s = 0.0;
    int action = 0, phase = 0, trial = 0;
    uint32_t flags = 0;
    Vec q{}, dq{}, tau{}, q_cmd{}, dq_cmd{}, tau_ff{}, kp{}, kd{};
    std::array<double, 10> imu{};
};

// One producer (FSM update), one consumer (file writer). Never writes files or locks
// a mutex in push(). Overflow is explicit and causes the experiment to stop.
class CsvRecorder {
public:
    CsvRecorder(std::string root, Vec center, double kp, double kd);
    ~CsvRecorder();
    CsvRecorder(const CsvRecorder&) = delete;
    CsvRecorder& operator=(const CsvRecorder&) = delete;
    void setDescription(const std::string& xml);
    void begin();
    void finish();
    bool push(const Sample& sample);
    bool ready() const { return ready_.load(std::memory_order_acquire) && !failed_.load(); }
    bool failed() const { return failed_.load(); }
    // Called only by the non-real-time status timer; never by the control update.
    std::string takeNotice();
    void checkpoint(std::string label, uint64_t first, uint64_t last);

private:
    static constexpr size_t capacity = 4096;
    void work();
    void report(const std::string& text);
    std::unique_ptr<std::array<Sample, capacity>> ring_;
    std::atomic<size_t> head_{0}, tail_{0};
    std::atomic<bool> requested_{false}, accepting_{false}, ready_{false};
    std::atomic<bool> failed_{false}, shutdown_{false}, description_ready_{false};
    std::atomic<uint64_t> dropped_{0};
    std::string root_, description_;
    Vec center_;
    double kp_, kd_;
    std::mutex description_mutex_;
    std::mutex notice_mutex_;
    std::string notice_;
    struct Checkpoint { std::string label; uint64_t first, last; };
    std::mutex checkpoint_mutex_;
    std::deque<Checkpoint> checkpoints_;
    std::thread writer_;
};
}  // namespace leg_identification
