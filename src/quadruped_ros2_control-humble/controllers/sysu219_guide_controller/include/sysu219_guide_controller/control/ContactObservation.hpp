#pragma once
#include <array>
#include <cstdint>

namespace sysu219_guide_controller {
// 与通过连续性保护的里程计结果同帧传递；不包含估计器的可写引用。
// contact 是受力接触状态，不能用滑移 q 替代。fz 是力矩反馈推算值，单位 N。
struct ContactObservation {
    std::int64_t stamp_ns = 0, steady_ns = 0;
    std::array<int, 4> contact{}, force_valid{};
    std::array<double, 4> fz{};
    std::array<double, 4> relative_height{}, relative_vz{}; // 相对支撑候选足，m / m/s。
    double force_on_N = 0.0; // 与接触检测器一致的门限，供限力预加载使用。
};
}
