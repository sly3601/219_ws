#ifndef STATERLWALK_H
#define STATERLWALK_H

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "controller_common/FSM/FSMState.h"
#include "sysu219_guide_controller/control/CtrlComponent.h"
#include "sysu219_guide_controller/control/RLPolicy.h"

class StateRLWalk final : public FSMState {
public:
    explicit StateRLWalk(CtrlInterfaces& ctrl_interfaces, CtrlComponent& ctrl_component);
    void enter() override;
    void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    void resolveInterfaces();
    void buildObservation(double elapsed_s);
    bool runPolicy(double elapsed_s);
    void sendJointCommand();
    void fail(const std::string& message);

    RLPolicy policy_;
    bool configured_ = false;
    bool mapped_ = false;
    bool fault_ = false;
    bool policy_started_ = false;
    std::string configuration_error_;
    std::string model_path_;

    // Independent name maps for each interface family; never assume USD order.
    std::array<std::size_t, 12> q_index_{}, dq_index_{}, q_cmd_index_{}, dq_cmd_index_{};
    std::array<std::size_t, 12> torque_index_{}, kp_index_{}, kd_index_{};
    std::array<std::size_t, 7> imu_index_{};  // orientation w/x/y/z, angular velocity x/y/z
    std::array<double, 12> q_start_{}, kp_start_{}, kd_start_{};
    // Shared with traditional FIXEDSTAND; must match the saved actor contract.
    std::array<double, 12> default_position_{};
    std::array<double, 12> q_target_{}, kp_target_{}, kd_target_{};
    std::array<double, 12> last_action_{};  // clipped action, before scaling/default offset
    std::vector<float> observation_;

    double transition_s_ = 0.5;
    double entry_timeout_s_ = 5.0;
    double entry_tolerance_rad_ = 0.25;
    double entry_velocity_rad_s_ = 1.0;
    double max_period_s_ = 0.1;
    double entry_elapsed_s_ = 0.0;
    double policy_elapsed_s_ = 0.0;
    double since_policy_s_ = 0.0;
};

#endif  // STATERLWALK_H
