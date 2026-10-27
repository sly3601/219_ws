#pragma once
#include "ContactObservation.hpp"
#include "sysu219_guide_controller/common/mathTypes.h"
#include <vector>

class WaveGenerator;
namespace sysu219_guide_controller {
// 控制线程独占：将计划、受力观测、轨迹交接和承重授权分开，不改写 WaveGenerator。
class ContactCoordinator final {
public:
    enum Stage { Stance = 0, Unloading = 1, Swing = 2, Searching = 3, Braking = 4, Loading = 5, Probing = 6 };
    void reset(const Vec34& feet);
    bool update(double dt, const WaveGenerator& wave, const ContactObservation& observation,
                bool fresh, const Vec34& feet, double force_max, double weight, bool allocation_ok);
    void filterTargets(Vec34& position, Vec34& velocity);
    void preview(std::vector<std::array<int, 4>>& table, std::vector<Vec4>& limits, std::vector<Vec4>& minimum,
                 double dt, double force_max) const;
    const VecInt4& support() const { return support_; }
    const VecInt4& motionContact() const { return motion_contact_; }
    const Vec4& phase() const { return phase_; }
    const Vec4& load() const { return load_; }
    const Vec4& minimumForce() const { return minimum_force_; }
    const Vec4& blockedTime() const { return blocked_s_; }
    const VecInt4& stage() const { return stage_; }
    bool fault() const { return fault_; }
    const char* reason() const { return reason_; }
    int faultLeg() const { return fault_leg_; }
    bool recovering() const {
        for (int leg = 0; leg < 4; ++leg)
            if (stage_(leg) == Searching || stage_(leg) == Probing || request_age_(leg) > transition_s) return true;
        return false;
    }
    static constexpr double transition_s = 0.04; // 沿用原有增益交接时间，非已验证真机参数。
    static constexpr double search_speed_m_s = 0.04, search_depth_m = 0.02, search_timeout_s = 0.30;
    static constexpr double touchdown_height_m = search_depth_m, touchdown_rise_m_s = 0.10;
    static constexpr double contact_hold_s = 0.08;
    static constexpr double contact_unknown_hold_s = 0.20; // 覆盖力观测器120 ms重建期。
    static constexpr double blocked_confirm_s = 0.02, blocked_gap_m = 0.002, probe_force_ratio = 1.25;
private:
    void enter(int leg, Stage stage);
    bool canLift(const VecInt4& remove, double force_max, double weight) const;
    bool fail(const char* reason, int leg = -1);
    VecInt4 stage_ = VecInt4::Zero(), support_ = VecInt4::Zero();
    VecInt4 motion_contact_ = VecInt4::Ones(), requested_ = VecInt4::Zero();
    VecInt4 armed_ = VecInt4::Ones(); // 落地后须先经过名义支撑期，才能响应下一次抬腿请求。
    VecInt4 provisional_ = VecInt4::Zero();
    Vec4 phase_ = Vec4::Constant(0.5), load_ = Vec4::Ones(), timer_ = Vec4::Zero();
    Vec4 request_age_ = Vec4::Zero();
    Vec4 blocked_s_ = Vec4::Zero(), load_start_ = Vec4::Zero(), minimum_force_ = Vec4::Zero();
    Vec4 touch_loss_s_ = Vec4::Zero(), contact_missing_s_ = Vec4::Zero();
    VecInt4 fresh_contact_ = VecInt4::Zero();
    Vec34 last_p_ = Vec34::Zero(), last_v_ = Vec34::Zero(), anchor_ = Vec34::Zero();
    Vec34 start_p_ = Vec34::Zero(), start_v_ = Vec34::Zero();
    double swing_s_ = 0.325;
    double probe_fraction_ = 0.0;
    bool fault_ = false;
    int fault_leg_ = -1;
    const char* reason_ = "ready";
};
}
