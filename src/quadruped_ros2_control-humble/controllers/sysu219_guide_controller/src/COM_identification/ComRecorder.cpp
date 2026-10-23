#include "ComRecorder.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <utility>
#include <unistd.h>

namespace com_identification {
Recorder::Recorder() : queue_(std::make_unique<std::array<Sample, capacity>>()),
                       writer_(&Recorder::work, this) {}
Recorder::~Recorder() {
    shutdown_.store(true);
    if (writer_.joinable()) writer_.join();
}
void Recorder::start(std::string root, std::string description, std::string metadata) {
    // Called once, from the non-control ROS callback, before any samples exist.
    root_ = std::move(root); description_ = std::move(description); metadata_ = std::move(metadata);
    start_.store(true, std::memory_order_release);
}
void Recorder::finish(bool completed) {
    if (finish_.load()) return; // A later keyboard exit must not invalidate an already completed run.
    completed_.store(completed);
    finish_.store(true, std::memory_order_release);
}
bool Recorder::push(const Sample& sample) {
    if (!ready() || failed() || finish_.load()) return false;
    const auto head = head_.load(std::memory_order_relaxed);
    const auto next = (head + 1) % capacity;
    if (next == tail_.load(std::memory_order_acquire)) { failed_.store(true); return false; }
    (*queue_)[head] = sample;
    head_.store(next, std::memory_order_release);
    return true;
}
void Recorder::work() {
    using namespace std::chrono_literals;
    while (!start_.load(std::memory_order_acquire)) {
        if (shutdown_.load()) return;
        std::this_thread::sleep_for(5ms);
    }
    uint64_t written = 0;
    try {
        const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto folder = std::filesystem::path(root_) /
            ("com_id_" + std::to_string(stamp) + "_" + std::to_string(getpid()));
        if (!std::filesystem::create_directories(folder)) throw std::runtime_error("COM directory exists");
        directory_ = folder.string();
        std::ofstream urdf(folder / "robot_description.urdf");
        urdf << description_; urdf.close();
        if (!urdf) throw std::runtime_error("Cannot save COM URDF");
        std::ofstream metadata(folder / "metadata.json");
        metadata << metadata_; metadata.close();
        if (!metadata) throw std::runtime_error("Cannot save COM metadata");
        std::ofstream csv(folder / "samples.csv");
        csv << std::setprecision(17) << "steady_ns,ros_ns,sequence,period_s,pose,phase";
        for (const auto* leg : {"FR", "FL", "RR", "RL"})
            for (const auto* kind : {"hip", "thigh", "calf"})
                for (const auto* field : {"q", "dq", "tau", "q_cmd", "kp", "kd"})
                    csv << ',' << leg << '_' << kind << '_' << field;
        for (const auto* name : {"qw", "qx", "qy", "qz", "gx", "gy", "gz", "ax", "ay", "az"})
            csv << ',' << name;
        csv << '\n'; csv.flush();
        if (!csv) throw std::runtime_error("Cannot open COM CSV");
        ready_.store(true, std::memory_order_release);
        auto flushed = std::chrono::steady_clock::now();
        for (;;) {
            const auto tail = tail_.load(std::memory_order_relaxed);
            if (tail != head_.load(std::memory_order_acquire)) {
                const auto s = (*queue_)[tail];
                tail_.store((tail + 1) % capacity, std::memory_order_release);
                csv << s.steady_ns << ',' << s.ros_ns << ',' << s.sequence << ',' << s.period
                    << ',' << s.pose << ',' << s.phase;
                for (size_t j = 0; j < 12; ++j)
                    csv << ',' << s.q[j] << ',' << s.dq[j] << ',' << s.tau[j] << ',' << s.q_cmd[j]
                        << ',' << s.kp[j] << ',' << s.kd[j];
                for (double value : s.imu) csv << ',' << value;
                csv << '\n'; ++written;
            } else {
                if (finish_.load(std::memory_order_acquire) || shutdown_.load() || failed()) break;
                std::this_thread::sleep_for(2ms);
            }
            if (std::chrono::steady_clock::now() - flushed > 1s) {
                csv.flush(); flushed = std::chrono::steady_clock::now();
            }
            if (!csv) throw std::runtime_error("COM CSV write failed");
        }
        csv.close();
        if (!csv) throw std::runtime_error("COM CSV flush failed");
        // Marker is written last: consumers never process a still-growing CSV.
        std::ofstream result(folder / "acquisition.json");
        result << "{\"completed\":" << (completed_.load() && !failed() ? "true" : "false")
               << ",\"writer_failed\":" << (failed() ? "true" : "false")
               << ",\"written_samples\":" << written << "}\n";
        result.close();
        if (!result) throw std::runtime_error("COM completion marker failed");
    } catch (...) { failed_.store(true); }
    done_.store(true, std::memory_order_release);
}
} // namespace com_identification
