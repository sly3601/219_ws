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
    double kp_start_; // 初始kp（设为0）
    double kd_start_; // 初始kd（设为0）

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;
};


#endif //STATEFIXEDDOWN_H
