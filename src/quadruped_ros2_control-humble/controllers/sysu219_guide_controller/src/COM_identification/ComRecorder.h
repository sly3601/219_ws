#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace com_identification {
struct Sample {
    int64_t steady_ns = 0, ros_ns = 0;
    uint64_t sequence = 0;
    int pose = 0, phase = 0;
    double period = 0;
    int motion_quality_ok = 0;
    uint64_t motion_quality_bad_cycles = 0;
    std::array<double, 12> q{}, dq{}, tau{}, q_cmd{}, kp{}, kd{};
    std::array<double, 10> imu{}; // wxyz, gyro xyz, specific acceleration xyz
};

// Single control-thread producer; filesystem work belongs only to the writer.
class Recorder {
public:
    Recorder();
    ~Recorder();
    void start(std::string root, std::string description, std::string metadata);
    bool push(const Sample& sample);
    void finish(bool completed, const char* static_reason = nullptr);
    bool ready() const { return ready_.load(std::memory_order_acquire); }
    bool failed() const { return failed_.load(); }
    bool done() const { return done_.load(std::memory_order_acquire); }
    const std::string& directory() const { return directory_; } // read only after done()
private:
    void work();
    static constexpr size_t capacity = 4096;
    std::unique_ptr<std::array<Sample, capacity>> queue_;
    std::atomic<size_t> head_{0}, tail_{0};
    std::atomic<bool> start_{false}, finish_{false}, shutdown_{false};
    std::atomic<bool> ready_{false}, failed_{false}, done_{false}, completed_{false};
    std::atomic<const char*> finish_reason_{nullptr}; // Static string literals; no allocation in control loop.
    std::string root_, description_, metadata_, directory_;
    std::thread writer_;
};
} // namespace com_identification
