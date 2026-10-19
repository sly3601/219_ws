#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace leg_identification {
using Vec = std::array<double, 3>;
constexpr double pi = 3.14159265358979323846;
constexpr double sample_period = 0.004;
// User-confirmed fixture: FL positive hip angles raise the leg; negative angles lower it.
// Zero is the preparation/return reference, not the midpoint of this one-sided motion.
constexpr double hip_lower = 0.0;
constexpr double hip_upper = 37.0 * pi / 180.0;
// Thigh/calf centers are the midpoints of the current hardware/URDF limit intersections.
constexpr Vec identification_center{0.0, 0.5135, -1.605};
constexpr double margin = 0.1;
constexpr double transition_seconds = 4.0;
constexpr double gain_seconds = 1.5;
// Hip: maximum positive excursion from zero. Thigh/calf: symmetric amplitudes.
constexpr Vec amplitudes{hip_upper, 0.80, 0.48};
constexpr double motion_time_scale = 1.0;
constexpr double envelope_seconds = 2.0 * motion_time_scale;
constexpr double single_frequency = 0.5 / motion_time_scale;
constexpr Vec train_frequency{0.3 / motion_time_scale, 0.6 / motion_time_scale, 1.2 / motion_time_scale};
constexpr Vec test_frequency{0.4 / motion_time_scale, 0.8 / motion_time_scale, 1.1 / motion_time_scale};
constexpr double validation_frequency_shift = 0.05 / motion_time_scale;
constexpr Vec multisine_weights{0.5, 0.3, 0.2};
// Gross-motion fault thresholds, separate from the planned excitation window.
constexpr double position_fault_margin = 20.0 * pi / 180.0;
constexpr double measured_velocity_limit = 10.0;
constexpr double keyboard_timeout = 2.0;
constexpr double ready_hold_seconds = 0.5;
// Preserve the validated arrival criteria independently of the enlarged excitation.
constexpr Vec ready_position_error{0.03, 0.12, 0.12};
constexpr Vec ready_velocity{0.0075, 0.03, 0.03};
constexpr Vec velocity_limit{0.5, 2.0, 2.0};
constexpr Vec acceleration_limit{4.0, 15.0, 15.0};
// Copied from HardwareSysu219::on_init, read only. The hardware files are untouched.
constexpr Vec hardware_lower{-0.638, -0.623, -2.322};
constexpr Vec hardware_upper{0.748, 1.65, -0.672};

enum class Action { Hold = 0, Static = 1, Hip = 2, Thigh = 3, Calf = 4,
                    Combined = 5, Validation = 6 };
enum class Phase { Stopped = 0, GainRamp = 1, Transition = 2, Ready = 3, Experiment = 4,
                   Settling = 5, Holding = 6 };
enum class MotionRange { Preparation, Excitation };

struct Target { Vec q{}, dq{}, ddq{}; };

inline double duration(Action a) {
    if (a == Action::Static) return 15.0;
    if (a == Action::Hip || a == Action::Thigh || a == Action::Calf) return 20.0 * motion_time_scale;
    return 40.0 * motion_time_scale;
}
inline int repetitions(Action a) {
    if (a == Action::Static || a == Action::Combined) return 3;
    return a == Action::Validation ? 2 : 1;
}
// C2 envelope: starts and ends at rest, including noninteger-period validation signals.
inline std::array<double, 3> smooth(double t, double T) {
    const double s = std::clamp(t / T, 0.0, 1.0);
    return {s*s*s*(10.0 + s*(-15.0 + 6.0*s)),
            30.0*s*s*(1.0-s)*(1.0-s)/T,
            60.0*s*(1.0-s)*(1.0-2.0*s)/(T*T)};
}

inline Vec staticPose(const Vec& center, int trial) {
    Vec q = center;
    if (trial > 0) {
        const double sign = trial == 1 ? 1.0 : -1.0;
        q[0] += trial == 1 ? 0.5 * amplitudes[0] : amplitudes[0];
        q[1] += sign * (2.0 / 3.0) * amplitudes[1];
        q[2] -= sign * (2.0 / 3.0) * amplitudes[2];
    }
    return q;
}

inline Target excitation(const Vec& center, Action action, int trial, double t) {
    Target out;
    out.q = action == Action::Static ? staticPose(center, trial) : center;
    if (action == Action::Static || action == Action::Hold) return out;
    const double T = duration(action);
    auto envelope = smooth(t, envelope_seconds);
    if (t >= envelope_seconds && t <= T - envelope_seconds) envelope = {1.0, 0.0, 0.0};
    if (t > T - envelope_seconds) {
        envelope = smooth(T - t, envelope_seconds);
        envelope[1] = -envelope[1];
    }
    const bool single = action >= Action::Hip && action <= Action::Calf;
    for (int j = 0; j < 3; ++j) {
        if (single && j != static_cast<int>(action) - 2) continue;
        double p = 0.0, v = 0.0, a = 0.0;
        for (int k = 0; k < (single ? 1 : 3); ++k) {
            const bool test = action == Action::Validation;
            const double f = single ? single_frequency : (test ? test_frequency[k] + validation_frequency_shift * trial
                                                                 : train_frequency[k]);
            const double phase = single ? 0.0 : (0.7*j + 0.4*trial + 0.3*j*k + (test ? 0.9 : 0.0));
            const double w = 2.0*pi*f;
            const double scale = amplitudes[j] * (single ? 1.0 : multisine_weights[k]);
            p += scale * std::sin(w*t + phase);
            v += scale * w * std::cos(w*t + phase);
            a -= scale * w*w * std::sin(w*t + phase);
        }
        // Nonnegative hip profile with the same smooth envelope: 0 .. +37 degrees.
        // Transform p/dp/ddp together; the start/end remain q=dq=ddq=0 relative to center.
        if (j == 0) { p = 0.5 * (amplitudes[0] + p); v *= 0.5; a *= 0.5; }
        out.q[j] += envelope[0] * p;
        out.dq[j] = envelope[1]*p + envelope[0]*v;
        out.ddq[j] = envelope[2]*p + 2.0*envelope[1]*v + envelope[0]*a;
    }
    return out;
}

// Quintic transfer preserves the current commanded q/dq/ddq when switching actions.
class Transfer {
public:
    void reset(const Target& start, const Vec& goal, double seconds = transition_seconds) {
        duration_ = seconds;
        goal_ = goal;
        for (int j = 0; j < 3; ++j) {
            const double T = duration_;
            auto& c = coefficients_[j];
            c[0] = start.q[j]; c[1] = start.dq[j]; c[2] = 0.5*start.ddq[j];
            const double p = goal[j] - c[0] - c[1]*T - c[2]*T*T;
            const double v = -c[1] - 2.0*c[2]*T;
            const double a = -2.0*c[2];
            c[3] = (10*p - 4*v*T + 0.5*a*T*T)/(T*T*T);
            c[4] = (-15*p + 7*v*T - a*T*T)/(T*T*T*T);
            c[5] = (6*p - 3*v*T + 0.5*a*T*T)/(T*T*T*T*T);
        }
    }
    Target at(double t) const {
        // Return the exact endpoint so roundoff cannot cross a joint limit.
        if (t >= duration_) return Target{goal_};
        t = std::clamp(t, 0.0, duration_);
        Target out;
        for (int j = 0; j < 3; ++j) {
            const auto& c = coefficients_[j];
            out.q[j] = c[0]+t*(c[1]+t*(c[2]+t*(c[3]+t*(c[4]+t*c[5]))));
            out.dq[j] = c[1]+t*(2*c[2]+t*(3*c[3]+t*(4*c[4]+t*5*c[5])));
            out.ddq[j] = 2*c[2]+t*(6*c[3]+t*(12*c[4]+t*20*c[5]));
        }
        return out;
    }
    double duration() const { return duration_; }
private:
    double duration_ = transition_seconds;
    Vec goal_{};
    std::array<std::array<double, 6>, 3> coefficients_{};
};
}  // namespace leg_identification
