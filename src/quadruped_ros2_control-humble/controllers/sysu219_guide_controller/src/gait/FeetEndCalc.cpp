//
// Created by biao on 24-9-18.
//

#include "sysu219_guide_controller/gait/FeetEndCalc.h"

#include <sysu219_guide_controller/control/CtrlComponent.h>
#include <sysu219_guide_controller/control/Estimator.h>

FeetEndCalc::FeetEndCalc(CtrlComponent &ctrl_component)
    : ctrl_component_(ctrl_component),
      robot_model_(ctrl_component.robot_model_),
      estimator_(ctrl_component.estimator_) {
    k_x_ = 0.005; // x 方向速度误差修正系数
    k_y_ = 0.005; // y 方向速度误差修正系数
    k_yaw_ = 0.005; // yaw 角速度误差修正系数
}

void FeetEndCalc::init() {
    t_stance_ = ctrl_component_.wave_generator_->get_t_stance(); // 返回每条腿支撑相时间：t_stance = period * st_ratio  例如一个周期 period = 0.5s，支撑比例 st_ratio = 0.6，那么：t_stance_ = 0.5 * 0.6 = 0.3s
    t_swing_ = ctrl_component_.wave_generator_->get_t_swing(); //摆动相持续时间，单位秒。

    Vec34 feet_pos_body = estimator_->getFeetPos2Body();
    // Vec34 feet_pos_body = robot_model_.feet_pos_normal_stand_;
    for (int i(0); i < 4; ++i) {
        feet_radius_(i) =
                sqrt(pow(feet_pos_body(0, i), 2) + pow(feet_pos_body(1, i), 2)); // feet_radius_(i) = sqrt(x^2 + y^2);
        feet_init_angle_(i) = atan2(feet_pos_body(1, i), feet_pos_body(0, i));   // feet_init_angle_(i) = atan2(y, x);
    }
}


// index: 第几条腿，0 FR / 1 FL / 2 RR / 3 RL
// vxy_goal_global: 期望的全局 x/y 速度
// d_yaw_global: 期望的全局yaw角速度
// phase: 当前步态周期的阶段，范围 [0, 1)
Vec3 FeetEndCalc::calcFootPos(const int index, Vec2 vxy_goal_global, const double d_yaw_global, const double phase) {
    Vec3 body_vel_global = estimator_->getVelocity();
    Vec3 next_step;

    next_step(0) = body_vel_global(0) * (1 - phase) * t_swing_ +        // x轴机身速度在摆动相中的贡献，摆动相越后面，贡献越小
                   body_vel_global(0) * t_stance_ / 2 +                 // x轴机身速度在支撑相中的贡献，平均分布在整个周期
                   k_x_ * (body_vel_global(0) - vxy_goal_global(0));    // x方向的速度误差修正项
    // next_step(1) = body_vel_global(1) * (1 - phase) * t_swing_ +        // y轴机身速度在摆动相中的贡献，摆动相越后面，贡献越小
    //                body_vel_global(1) * t_stance_ / 2 +
    //                k_y_ * (body_vel_global(1) - vxy_goal_global(1));    // y方向的速度误差修正项
    next_step(2) = 0;

    const double yaw = estimator_->getYaw();    // 当前的全局yaw角
    const double d_yaw = estimator_->getDYaw(); // 当前的全局yaw角速度
    const double next_yaw = d_yaw * (1 - phase) * t_swing_ + d_yaw * t_stance_ / 2 +  // yaw角速度在摆动相中的贡献，摆动相越后面，贡献越小
                            k_yaw_ * (d_yaw_global - d_yaw);  // yaw角速度误差修正项

    next_step(0) +=
            feet_radius_(index) * cos(yaw + feet_init_angle_(index) + next_yaw);  // x = r * cos(theta)
    // next_step(1) +=
    //         feet_radius_(index) * sin(yaw + feet_init_angle_(index) + next_yaw);

    Vec3 foot_pos = estimator_->getPosition() + next_step;
    foot_pos(2) = 0.0;

    return foot_pos;
}
