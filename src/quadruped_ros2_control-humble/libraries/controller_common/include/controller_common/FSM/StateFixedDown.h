//
// Created by tlab-uav on 24-9-11.
//

#ifndef STATEFIXEDDOWN_H
#define STATEFIXEDDOWN_H

#include "FSMState.h"

class StateFixedDown final : public FSMState
{
public:
    explicit StateFixedDown(CtrlInterfaces& ctrl_interfaces,
                            const std::vector<double>& target_pos,
                            double kp,
                            double kd
    );

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    double target_pos_[12] = {};
    double start_pos_[12] = {};
    rclcpp::Time start_time_;

    double kp_, kd_;
        // 加在现有成员后面
    double kp_start_; // 切换瞬间给一个安全起步的 kp（避免关节失稳自由落体），run() 中再升到 kp_
    double kd_start_; // 同 kd

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;
};


#endif //STATEFIXEDDOWN_H
