#include "sysu219_guide_controller/control/ContactCoordinator.hpp"
#include "sysu219_guide_controller/gait/WaveGenerator.h"
#include <algorithm>
#include <cmath>

namespace sysu219_guide_controller {
namespace {
constexpr double kSearchSpeed = ContactCoordinator::search_speed_m_s;
constexpr double kSearchDepth = ContactCoordinator::search_depth_m, kSearchTimeout = ContactCoordinator::search_timeout_s;
double ramp(double elapsed) { return std::clamp(elapsed / ContactCoordinator::transition_s, 0.0, 1.0); }
bool geometry(const VecInt4& mask) {
    const int n = mask.sum();
    // 这里只保留本机已使用的对角双支撑；三/四足交接由原MPC/WBC检验可行性。
    return n >= 3 || (n == 2 && ((mask(0) && mask(3)) || (mask(1) && mask(2))));
}
}

void ContactCoordinator::reset(const Vec34& feet) {
    stage_.setConstant(Stance); support_.setOnes(); motion_contact_.setOnes();
    requested_.setZero(); armed_.setOnes(); timer_.setZero(); request_age_.setZero();
    provisional_.setZero(); blocked_s_.setZero(); load_start_.setZero(); minimum_force_.setZero();
    touch_loss_s_.setZero(); contact_missing_s_.setZero(); fresh_contact_.setOnes();
    phase_.setConstant(0.5); load_.setOnes();
    last_p_ = anchor_ = start_p_ = feet; last_v_.setZero(); start_v_.setZero();
    fault_ = false; fault_leg_ = -1; reason_ = "ready";
}
bool ContactCoordinator::fail(const char* reason, int leg) { fault_ = true; fault_leg_ = leg; reason_ = reason; return false; }
void ContactCoordinator::enter(int leg, Stage stage) {
    stage_(leg) = stage; timer_(leg) = 0.0;
    touch_loss_s_(leg) = 0.0;
    if (stage == Swing) { armed_(leg) = 0; contact_missing_s_(leg) = 0.0; }
    start_p_.col(leg) = last_p_.col(leg); start_v_.col(leg) = last_v_.col(leg);
    if (stage == Braking) {
        // 搜地目标可能已低于实测足点。不能拉回实测z，否则五次曲线会抬脚、丢接触。
        // delta_z <= T*v0_z/2 且 v0_z<=0，使原五次曲线整个区间的z速度都不为正。
        start_v_(2, leg) = std::min(0.0, start_v_(2, leg));
        anchor_(2, leg) = std::min(anchor_(2, leg), start_p_(2, leg) + 0.5 * transition_s * start_v_(2, leg));
    }
}
bool ContactCoordinator::canLift(const VecInt4& remove, double force_max, double weight) const {
    VecInt4 remaining = support_;
    double capacity = 0.0;
    for (int leg = 0; leg < 4; ++leg) {
        if (remove(leg) || stage_(leg) == Unloading || provisional_(leg) || !fresh_contact_(leg) ||
            load_(leg) < 1.0 - 1e-9) remaining(leg) = 0;
        if (remaining(leg)) capacity += force_max * load_(leg);
    }
    // 垂直容量只是必要条件；交接中还要求上一帧实际分配成功，失败不继续撤脚。
    return geometry(remaining) && capacity >= 1.1 * weight;
}

bool ContactCoordinator::update(double dt, const WaveGenerator& wave, const ContactObservation& observation,
    bool fresh, const Vec34& feet, double force_max, double weight, bool allocation_ok) {
    if (fault_) return false;
    if (!fresh || !std::isfinite(dt) || dt <= 0.0 || dt > 0.10 || !feet.allFinite() ||
        !std::isfinite(force_max) || force_max <= 0.0 || !std::isfinite(weight) || weight <= 0.0)
        return fail("stale contact / invalid timing or geometry");
    swing_s_ = wave.get_t_swing();
    if (!std::isfinite(swing_s_) || swing_s_ <= 0.0) return fail("invalid swing duration");
    if (!std::isfinite(observation.force_on_N) || observation.force_on_N <= 0.0 ||
        probe_force_ratio * observation.force_on_N > force_max) return fail("invalid preload force limit");
    probe_fraction_ = probe_force_ratio * observation.force_on_N / force_max;
    const int ticks = static_cast<int>(std::min(std::ceil(transition_s / dt), 62.0)) + 2;
    const auto near = wave.getMpcContactTable(ticks, dt);
    VecInt4 observed = VecInt4::Zero();
    VecInt4 touchdown = VecInt4::Zero();
    for (int leg = 0; leg < 4; ++leg) {
        if ((observation.contact[leg] != 0 && observation.contact[leg] != 1) ||
            !std::isfinite(wave.phase_(leg))) return fail("invalid contact value");
        observed(leg) = observation.force_valid[leg] && observation.contact[leg] &&
                        std::isfinite(observation.fz[leg]);
        fresh_contact_(leg) = observed(leg);
        contact_missing_s_(leg) = observed(leg) ? 0.0 : contact_missing_s_(leg) + dt;
        // 已承重脚的短时力估计波动只保留现有支撑，不能用保留状态授权其他腿撤脚。
        const double hold_s = observation.force_valid[leg] ? contact_hold_s : contact_unknown_hold_s;
        if (!observed(leg) && support_(leg) && !provisional_(leg) && contact_missing_s_(leg) <= hold_s &&
            (stage_(leg) == Stance || stage_(leg) == Unloading || stage_(leg) == Loading)) observed(leg) = 1;
        // 初次接载增加近地面/未上升检查；不凭力估计的拖尾在空中授权。
        touchdown(leg) = observed(leg) && std::isfinite(observation.relative_height[leg]) &&
            std::abs(observation.relative_height[leg]) <= touchdown_height_m &&
            std::isfinite(observation.relative_vz[leg]) && observation.relative_vz[leg] <= touchdown_rise_m_s;
    }
    // 先处理同一帧全部观测，避免前一条腿使用后一条腿上一帧的接触。
    support_ = support_.cwiseProduct(observed);
    for (int leg = 0; leg < 4; ++leg) {
        timer_(leg) += dt;
        if (stage_(leg) == Stance) {
            if (wave.contact_(leg)) armed_(leg) = 1;
            bool upcoming = armed_(leg) && wave.contact_(leg) == 0;
            for (std::size_t k = 1; k < near.size() && k * dt <= transition_s; ++k)
                upcoming = upcoming || (armed_(leg) && near[k][leg] == 0);
            requested_(leg) = upcoming;
            request_age_(leg) = upcoming ? request_age_(leg) + dt : 0.0;
            if (request_age_(leg) > kSearchTimeout) {
                requested_(leg) = 0; armed_(leg) = 0; request_age_(leg) = 0.0; // 本拍不具备交接条件，跳过这一步。
            }
            if (!observed(leg) && !upcoming) enter(leg, Searching);
        } else if (stage_(leg) == Unloading) {
            if (timer_(leg) >= transition_s && wave.contact_(leg) == 0) {
                VecInt4 remove = VecInt4::Zero(); remove(leg) = 1;
                if (allocation_ok && canLift(remove, force_max, weight)) {
                    enter(leg, Swing); requested_(leg) = 0;
                } // 等待其余脚接稳，不因一次分配/接触波动退出整机。
            }
        } else if (stage_(leg) == Swing) {
            if (touchdown(leg) && timer_(leg) / swing_s_ >= 0.65) {
                provisional_(leg) = 0; anchor_.col(leg) = feet.col(leg); enter(leg, Braking);
            } else if (timer_(leg) >= swing_s_) { blocked_s_(leg) = 0.0; enter(leg, Searching); }
        } else if (stage_(leg) == Searching) {
            // 轻触脚还没承重，不能要求先达到承重后的力门限。仅在近地、实际停止下降、
            // 下压目标持续落后实测足点且有有效正力残差时，允许限力预加载；不直接全承重。
            const bool blocked = observation.force_valid[leg] && observation.fz[leg] > 0.0 &&
                std::isfinite(observation.relative_height[leg]) &&
                std::abs(observation.relative_height[leg]) <= touchdown_height_m &&
                std::isfinite(observation.relative_vz[leg]) &&
                std::abs(observation.relative_vz[leg]) <= touchdown_rise_m_s &&
                (last_v_(2, leg) < -0.01 || timer_(leg) >= kSearchTimeout) &&
                feet(2, leg) - last_p_(2, leg) >= blocked_gap_m;
            blocked_s_(leg) = blocked ? blocked_s_(leg) + dt : 0.0;
            if (touchdown(leg) || blocked_s_(leg) >= blocked_confirm_s) {
                provisional_(leg) = !touchdown(leg);
                anchor_.col(leg) = feet.col(leg); enter(leg, Braking);
            }
            // 超过搜地时间后目标有界保持，继续等待触地；不无限向下推，也不整机退出。
        } else if (stage_(leg) == Braking) {
            const bool near = observation.force_valid[leg] && std::isfinite(observation.relative_height[leg]) &&
                std::abs(observation.relative_height[leg]) <= touchdown_height_m &&
                std::isfinite(observation.relative_vz[leg]) && observation.relative_vz[leg] <= touchdown_rise_m_s;
            // 制动期间仍未承重。轻触力可回落、接触可抖动，不能单帧整机退出，
            // 也不能在几何证据丢失时直接预加载；持续失配只退回有总期限的搜地。
            touch_loss_s_(leg) = near ? 0.0 : touch_loss_s_(leg) + dt;
            if (touch_loss_s_(leg) >= blocked_confirm_s) {
                provisional_(leg) = 0; blocked_s_(leg) = 0.0; enter(leg, Searching);
            } else if (near && timer_(leg) >= transition_s) {
                load_start_(leg) = 0.0;
                if (touchdown(leg)) { provisional_(leg) = 0; enter(leg, Loading); }
                else { provisional_(leg) = 1; enter(leg, Probing); }
            }
        } else if (stage_(leg) == Probing) {
            if (touchdown(leg)) {
                load_start_(leg) = load_(leg); provisional_(leg) = 0; enter(leg, Loading);
            } // 尚未确认时保持限力预加载，其余脚继续独立承重，禁止据此撤脚。
        } else if (stage_(leg) == Loading) {
            if (!observed(leg)) { blocked_s_(leg) = 0.0; enter(leg, Searching); }
            else if (timer_(leg) >= transition_s) {
                enter(leg, Stance); request_age_(leg) = 0.0;
            }
        }
        const int st = stage_(leg);
        motion_contact_(leg) = st == Stance || st == Unloading || st == Braking || st == Loading || st == Probing;
        phase_(leg) = st == Swing ? std::min(1.0, timer_(leg) / swing_s_) : st == Searching ? 1.0 : 0.5;
        load_(leg) = st == Stance ? 1.0 : st == Unloading ? 1.0 - ramp(timer_(leg)) :
                     st == Loading ? load_start_(leg) + (1.0 - load_start_(leg)) * ramp(timer_(leg)) :
                     st == Probing ? probe_fraction_ * ramp(timer_(leg)) : 0.0;
        support_(leg) = (observed(leg) && (st == Stance || st == Unloading || st == Loading)) || st == Probing;
        minimum_force_(leg) = st == Probing ? load_(leg) * force_max :
            st == Loading ? load_start_(leg) * (1.0 - ramp(timer_(leg))) * force_max : 0.0;
    }
    VecInt4 confirmed = support_;
    for (int leg = 0; leg < 4; ++leg) if (provisional_(leg)) confirmed(leg) = 0;
    if (confirmed.sum() < 2) return fail("persistent loss of confirmed support");
    // 当前支撑只负责保持与恢复；容量/几何裕度仅用于下面批准新的卸载。
    // 同时评估本批准备抬起的腿，禁止两条腿各自看到对方还在支撑而同时撤脚。
    VecInt4 batch = VecInt4::Zero();
    for (int leg = 0; leg < 4; ++leg) batch(leg) = requested_(leg) && stage_(leg) == Stance;
    if (batch.sum() && allocation_ok && canLift(batch, force_max, weight)) {
        for (int leg = 0; leg < 4; ++leg) if (batch(leg)) enter(leg, Unloading);
    }
    return true;
}

void ContactCoordinator::filterTargets(Vec34& position, Vec34& velocity) {
    for (int leg = 0; leg < 4; ++leg) {
        const double t = timer_(leg);
        if (stage_(leg) == Braking) {
            const double u = ramp(t), u2 = u*u, u3 = u2*u, u4 = u3*u, u5 = u4*u;
            const Vec3 delta = anchor_.col(leg) - start_p_.col(leg);
            const Vec3 tv = transition_s * start_v_.col(leg);
            const Vec3 a = 10.0*delta - 6.0*tv, b = -15.0*delta + 8.0*tv, c = 6.0*delta - 3.0*tv;
            position.col(leg) = start_p_.col(leg) + u*tv + u3*a + u4*b + u5*c;
            velocity.col(leg) = (tv + 3.0*u2*a + 4.0*u3*b + 5.0*u4*c) / transition_s;
        } else if (stage_(leg) == Loading || stage_(leg) == Probing) {
            position.col(leg) = anchor_.col(leg); velocity.col(leg).setZero();
        } else if (stage_(leg) == Searching) {
            const double bounded_t = std::min(t, kSearchTimeout);
            const double decay = std::exp(-bounded_t / transition_s), integrated = transition_s * (1.0 - decay);
            const double depth = std::min(kSearchDepth, kSearchSpeed * (bounded_t - integrated));
            position.col(leg) = start_p_.col(leg) + integrated * start_v_.col(leg);
            position(2, leg) -= depth;
            velocity.col(leg) = decay * start_v_.col(leg);
            velocity(2, leg) -= kSearchSpeed * (1.0 - decay);
            if (t >= kSearchTimeout || depth >= kSearchDepth) velocity.col(leg).setZero();
        }
    }
    if (!position.allFinite() || !velocity.allFinite()) { fail("nonfinite transition target"); return; }
    last_p_ = position; last_v_ = velocity;
}

void ContactCoordinator::preview(std::vector<std::array<int, 4>>& table, std::vector<Vec4>& limits, std::vector<Vec4>& minimum,
    double dt, double force_max) const {
    limits.resize(table.size());
    minimum.assign(table.size(), Vec4::Zero());
    for (std::size_t k = 0; k < table.size(); ++k) {
        const double t = k * dt;
        for (int leg = 0; leg < 4; ++leg) {
            double fraction = 1.0;
            const int st = stage_(leg);
            if (st == Unloading) {
                fraction = 1.0 - ramp(timer_(leg) + t);
                table[k][leg] = fraction > 0.0;
            } else if (st == Probing) {
                fraction = probe_fraction_ * ramp(timer_(leg) + t); table[k][leg] = 1;
                minimum[k](leg) = fraction * force_max;
            } else if (st == Swing || st == Braking || st == Loading) {
                const double delay = st == Swing ? std::max(0.0, swing_s_ - timer_(leg)) + transition_s :
                                     st == Braking ? transition_s - timer_(leg) : -timer_(leg);
                fraction = ramp(t - delay); table[k][leg] = t >= delay;
                if (st == Loading) {
                    minimum[k](leg) = load_start_(leg) * (1.0 - fraction) * force_max;
                    fraction = load_start_(leg) + (1.0 - load_start_(leg)) * fraction;
                }
                if (st == Braking && provisional_(leg)) {
                    fraction *= probe_fraction_; minimum[k](leg) = table[k][leg] ? fraction * force_max : 0.0;
                }
            } else if (st == Searching) { table[k][leg] = 0; fraction = 0.0; }
            else if (requested_(leg)) table[k][leg] = 1; // 尚未获准卸载，不预测凭空撤脚。
            limits[k](leg) = table[k][leg] ? fraction * force_max : 0.0;
        }
        if (k == 0) {
            for (int leg = 0; leg < 4; ++leg) {
                table[k][leg] = support_(leg); limits[k](leg) = support_(leg) ? load_(leg) * force_max : 0.0;
                minimum[k](leg) = minimum_force_(leg);
            }
        } else {
            VecInt4 predicted;
            for (int leg = 0; leg < 4; ++leg) predicted(leg) = table[k][leg];
            if (!geometry(predicted)) for (int leg = 0; leg < 4; ++leg)
                if (support_(leg) && stage_(leg) == Stance) {
                    table[k][leg] = 1; limits[k](leg) = force_max;
                }
        }
    }
}
}
