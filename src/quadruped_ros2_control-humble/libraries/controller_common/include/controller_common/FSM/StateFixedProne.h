//
// Created by tlab-uav on 24-9-11.
//

#ifndef STATEFIXEDPRONE_H
#define STATEFIXEDPRONE_H

#include "FSMState.h"

/**
 * 完全趴着（比 FIXEDDOWN 更低）。
 *
 * 2 键在三个固定姿态之间往返，全趴和站立都是折返点：
 *   PASSIVE -2-> 全趴 -> 半趴 -> 站立 -> 半趴 -> 全趴 -> 半趴 -> 站立 -> ...
 * 1 键：任意状态 -> PASSIVE（失能）
 *
 * 也就是说 2 键永远不会失能，只有 1 键会。
 */
class StateFixedProne final : public FSMState
{
public:
    explicit StateFixedProne(CtrlInterfaces& ctrl_interfaces,
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

    double kp_, kd_;

    // enter() 时从 kp/kd 命令接口读到的"当前实际生效值"，
    // run() 中从它平滑升到 kp_/kd_：
    //   PASSIVE 切来 -> 0 / 1，刚度从 0 缓慢建立，避免电机 kp 突变冲击
    //   FIXEDDOWN 切来 -> 已是满值，等于不做刚度过渡
    double kp_start_[12] = {};
    double kd_start_[12] = {};

    double duration_ = 600; // steps
    double percent_ = 0; //%
};


#endif //STATEFIXEDPRONE_H
