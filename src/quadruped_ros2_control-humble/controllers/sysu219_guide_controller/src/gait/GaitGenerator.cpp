//
// Created by biao on 24-9-18.
//

#include "sysu219_guide_controller/gait/GaitGenerator.h"

#include <utility>
#include <sysu219_guide_controller/control/CtrlComponent.h>
#include <sysu219_guide_controller/control/Estimator.h>
#include <sysu219_guide_controller/gait/WaveGenerator.h>
// 【新增】包含 StateTrotting 头文件
#include "sysu219_guide_controller/FSM/StateTrotting.h"
#include <sysu219_guide_controller/common/mathTools.h>
#include <cmath>
#include <kdl/jntarray.hpp>

// GaitGenerator::GaitGenerator(CtrlComponent &ctrl_component)
//     : wave_generator_(ctrl_component.wave_generator_),
//       estimator_(ctrl_component.estimator_),
//       feet_end_calc_(ctrl_component) {
//     first_run_ = true;
// }
GaitGenerator::GaitGenerator(CtrlComponent &ctrl_component, StateTrotting* trotting_ptr)
    : wave_generator_(ctrl_component.wave_generator_),
      estimator_(ctrl_component.estimator_),
      feet_end_calc_(ctrl_component),
      first_run_(true),
      trotting_ptr_(trotting_ptr), 
      ctrl_component_(ctrl_component){
}



/* 纯赋值函数，无分析意义 */
void GaitGenerator::setGait(Vec2 vxy_goal_global, const double d_yaw_goal, const double gait_height) {
    vxy_goal_ = std::move(vxy_goal_global);
    d_yaw_goal_ = d_yaw_goal;
    gait_height_ = gait_height;
}

/* 核心：步态生成函数 */
void GaitGenerator::generate(Vec34 &feet_pos, Vec34 &feet_vel) {
    if (first_run_) 
    {
        if (trotting_ptr_ && trotting_ptr_->troting_kalman == 2) 
        {
            // 闭环：原来的逻辑，用 estimator 的全局足端位置
            // 初始化记录当前脚位置；迈步后保留计划落点。
            start_p_ = estimator_->getFeetPos();
            end_p_ = start_p_;
            // 使用 FixStand 的腿部姿态；髋关节名义角保持零，避免向外劈叉。
            for (int leg = 0; leg < 4; ++leg)
            {
                KDL::JntArray q_nominal(3);
                q_nominal(0) = 0.0;
                q_nominal(1) = nominal_joint_positions_[3 * leg + 1];
                q_nominal(2) = nominal_joint_positions_[3 * leg + 2];

                KDL::Frame foot_nominal =
                    ctrl_component_.robot_model_->calcPEe2B_four_feet(leg, q_nominal);

                nominal_feet_yaw_.col(leg) = nominal_body_to_yaw_
                    * Vec3(foot_nominal.p.x(), foot_nominal.p.y(), foot_nominal.p.z());
            }
        }
        else if (trotting_ptr_ && (trotting_ptr_->troting_kalman == 0))
        {
            // 开环：直接用正运动学计算当前足端位置，完全不依赖估计器
            for (int i = 0; i < 4; i++) 
            {
                KDL::JntArray fixed_q(3);

                // FR FL RR RL
                static const double stand_q[4][3] = {
                    {0.0, 0.9, -1.53},  // FR
                    {0.0, 0.9, -1.53},  // FL
                    {0.0, 0.9, -1.3 },  // RR
                    {0.0, 0.9, -1.3 }   // RL
                };

                fixed_q(0) = stand_q[i][0];
                fixed_q(1) = stand_q[i][1];
                fixed_q(2) = stand_q[i][2];

                KDL::Frame foot_frame = ctrl_component_.robot_model_->calcPEe2B_four_feet(i, fixed_q);

                start_p_(0, i) = foot_frame.p.x();
                start_p_(1, i) = foot_frame.p.y();
                start_p_(2, i) = foot_frame.p.z();
                end_p_.col(i) = start_p_.col(i);
            }
        }
        contact_last_ = wave_generator_->contact_;
        swing_start_offset_.setZero();
        swing_pos_ = start_p_;
        swing_vel_.setZero();
        swing_phase_last_.setZero();
        first_run_ = false;
    }

    // 起步全支撑期间刷新；首次抬脚时冻结四腿布局，后续始终作为落点几何基准。
    if (trotting_ptr_ && trotting_ptr_->troting_kalman == 2 && startup_swing_started_.sum() == 0) {
        const Vec34 feet = estimator_->getFeetPos();
        const Vec3 body = estimator_->getPosition();
        const RotMat global_to_yaw = rotz(estimator_->getYaw()).transpose();
        for (int leg = 0; leg < 4; ++leg)
            startup_feet_yaw_.col(leg) = global_to_yaw * (feet.col(leg) - body);
        // 前后脚分别采用 FixStand 模型的名义间距，保留起步横向中心与前后位置。
        const double front_center_y = 0.5 * (startup_feet_yaw_(1, 0) + startup_feet_yaw_(1, 1));
        const double rear_center_y = 0.5 * (startup_feet_yaw_(1, 2) + startup_feet_yaw_(1, 3));
        const double front_half_width = 0.5 * (nominal_feet_yaw_(1, 1) - nominal_feet_yaw_(1, 0));
        const double rear_half_width = 0.5 * (nominal_feet_yaw_(1, 3) - nominal_feet_yaw_(1, 2));
        startup_feet_yaw_(1, 0) = front_center_y - front_half_width;
        startup_feet_yaw_(1, 1) = front_center_y + front_half_width;
        startup_feet_yaw_(1, 2) = rear_center_y - rear_half_width;
        startup_feet_yaw_(1, 3) = rear_center_y + rear_half_width;
    }

    // 遍历机器人的4条腿（0:右前 1:左前 2:右后 3:左后）
    for (int i = 0; i < 4; i++) 
    {
        // 条件1：当前腿处于支撑相（踩地）
        if (wave_generator_->contact_(i) == 1) 
        {
            if (contact_last_(i) == 0)
                start_p_.col(i) = end_p_.col(i);
            else if (wave_generator_->phase_(i) < 0.5 &&
                     wave_generator_->status_ == WaveStatus::STANCE_ALL)
            {
                if (trotting_ptr_ &&
                    (trotting_ptr_->troting_kalman == 1 || trotting_ptr_->troting_kalman == 2))
                    start_p_.col(i) = estimator_->getFootPos(i);
            }
            feet_pos.col(i) = start_p_.col(i);
            feet_vel.col(i).setZero();// 这里本来就应该是零的，支撑相不需要移动，速度为0
        } 
        // 条件2：当前腿处于摆动相（抬脚迈步）
        else 
        {
            if (contact_last_(i) == 1 && trotting_ptr_ &&
                (trotting_ptr_->troting_kalman == 1 || trotting_ptr_->troting_kalman == 2))
                swing_start_offset_.col(i) = estimator_->getFootPos(i) - start_p_.col(i);
            if (contact_last_(i) == 1) {
                if (trotting_ptr_ && trotting_ptr_->troting_kalman == 2)
                    startup_swing_started_(i) = 1;
                swing_pos_.col(i) = start_p_.col(i) + swing_start_offset_.col(i);
                swing_vel_.col(i).setZero();
                swing_phase_last_(i) = 0.0;
            }
            // foot not contact, swing
            // ==============================================
            // 核心修复：开环/闭环 分两套逻辑计算「迈步终点」
            // ==============================================
            // 【开环模式】：纯身体坐标系，不用估计器，不用calcFootPos
            if (trotting_ptr_ && (trotting_ptr_->troting_kalman == 0))
            {
                // ==============================================
                // 核心修复：永远基于【固定的start_p】计算终点
                // 绝不基于上一次的end_p累加，彻底杜绝发散！
                // ==============================================
                end_p_.col(i) = start_p_.col(i);
            }
            else if (trotting_ptr_ && (trotting_ptr_->troting_kalman == 2))
            {
                Vec3 body_vel_global = estimator_->getVelocity();
                Vec3 next_step;
                next_step.setZero();

                const double t_stance = wave_generator_->get_t_stance();
                const double t_swing  = wave_generator_->get_t_swing();

                const double k_x = 0.5;
                const double k_y = 0.25;
                // k_x/k_y 为无量纲速度反馈比例：1 保留原响应，0.5 减半速度偏差引起的落点修正。
                // 期望速度前馈保留；剩余摆动时间预测、半摆动时间修正和原 5 ms 额外修正均保留。
                next_step(0) = (vxy_goal_(0) + k_x * (body_vel_global(0) - vxy_goal_(0)))
                            * (1.0 - wave_generator_->phase_(i)) * t_swing
                            + vxy_goal_(0) * t_stance / 2.0
                            + k_x * (body_vel_global(0) - vxy_goal_(0)) * t_swing / 2.0
                            + k_x * 0.005 * (body_vel_global(0) - vxy_goal_(0));

                next_step(1) = (vxy_goal_(1) + k_y * (body_vel_global(1) - vxy_goal_(1)))
                            * (1.0 - wave_generator_->phase_(i)) * t_swing
                            + vxy_goal_(1) * t_stance / 2.0
                            + k_y * (body_vel_global(1) - vxy_goal_(1)) * t_swing / 2.0
                            + k_y * 0.005 * (body_vel_global(1) - vxy_goal_(1));

                // 给速度预测项限幅，防止一步修太猛
                next_step(0) = saturation(next_step(0), Vec2(-10.135, 10.135));
                next_step(1) = saturation(next_step(1), Vec2(-10.135, 10.135)); 
                const double yaw = estimator_->getYaw();
                const double d_yaw = trotting_ptr_->force_solver_mode_ == StateTrotting::ForceSolverMode::MPC
                    ? trotting_ptr_->getFilteredYawRateGlobal()
                    : estimator_->getDYaw();

                const double k_yaw = 0.005;
                double next_yaw = d_yaw * (1.0 - wave_generator_->phase_(i)) * t_swing
                    + d_yaw_goal_ * t_stance / 2.0
                    + (d_yaw - d_yaw_goal_) * t_swing / 2.0
                    + k_yaw * (d_yaw_goal_ - d_yaw);

                // yaw 预测也限一下，防止 cos/sin 的目标点跳太远
                next_yaw = saturation(next_yaw, Vec2(-0.1, 0.1));
                

                // 始终使用起步实际布局，保留完整的实时速度/转向修正。
                const Vec2 feet_layout = startup_feet_yaw_.col(i).head<2>();
                const double feet_radius = feet_layout.norm();

                const double feet_init_angle =
                atan2(feet_layout(1), feet_layout(0));

                next_step(0) +=
                        feet_radius * cos(yaw + feet_init_angle + next_yaw);
                next_step(1) +=
                        feet_radius * sin(yaw + feet_init_angle + next_yaw);

                Vec3 foot_pos = estimator_->getPosition() + next_step;

                // 更新 x y 方向的落脚点预测值
                double target_x = foot_pos(0);
                double target_y = foot_pos(1);
                
                const double step_limit_xy = 0.84; // 步长限幅，防止迈步目标跳太远

                // 最终 x 落点相对当前摆动起点限幅
                target_x = saturation(target_x, Vec2(start_p_(0, i) - step_limit_xy,
                                                    start_p_(0, i) + step_limit_xy));
                target_y = saturation(target_y, Vec2(start_p_(1, i) - step_limit_xy,
                                                    start_p_(1, i) + step_limit_xy));

                end_p_.col(i) = start_p_.col(i);
                end_p_(0, i) = target_x;
                end_p_(1, i) = target_y;



                // // 关键：先把当前真实起点从外部系转回 B 系，
                // // 只在 B 系里修 x，再变回外部系。
                // const Vec3 pos_ext = estimator_->getPosition();
                // const RotMat B2P = estimator_->getRotation();
                // const RotMat P2B = B2P.transpose();

                // // 当前摆动起点（真实落脚点）转到 B 系
                // Vec3 start_body = P2B * (start_p_.col(i) - pos_ext);

                // // 在 B 系里构造摆动终点：只修 x，y/z 保持当前起点
                // Vec3 end_body = start_body;
                // const double alpha = 0.005;   // 先小一点，0.10~0.20 比较稳
                // end_body(0) = (1.0 - alpha) * start_body(0) + alpha * nominal_feet_yaw_(0, i);

                // // 再从 B 系变回当前 generate() 使用的外部系
                // end_p_.col(i) = pos_ext + B2P * end_body;
            }
            // 【闭环模式】：完全保留你原来的代码，用官方轨迹规划器
            else if (trotting_ptr_ && trotting_ptr_->troting_kalman == 1)
            {
                // 闭环：保持原来的逻辑不变
                end_p_.col(i) = feet_end_calc_.calcFootPos(i, vxy_goal_, d_yaw_goal_, wave_generator_->phase_(i));
            }

            // 调用你原有的摆线函数：计算足端轨迹/速度
            feet_pos.col(i) = getFootPos(i);
            feet_vel.col(i) = getFootVel(i);
            // 修正本步起点，仍落回原实时终点/地面高度；速度与位置修正一致。
            const double angle = 2.0 * M_PI * wave_generator_->phase_(i);
            const double blend = (angle - std::sin(angle)) / (2.0 * M_PI);
            feet_pos.col(i) += (1.0 - blend) * swing_start_offset_.col(i);
            feet_vel.col(i) -= (1.0 - std::cos(angle)) / wave_generator_->get_t_swing()
                              * swing_start_offset_.col(i);
            if (trotting_ptr_ && trotting_ptr_->troting_kalman == 2) {
                // 从上一目标位置/速度重规划 XY 三次曲线，终点仍逐帧更新。
                // 末端速度为零，避免直接替换摆线终点导致位置跳变和速度漏项。
                const double remaining = (1.0 - swing_phase_last_(i))
                    * wave_generator_->get_t_swing();
                const double u = (wave_generator_->phase_(i) - swing_phase_last_(i))
                    / (1.0 - swing_phase_last_(i));
                const Vec2 delta = end_p_.col(i).head<2>() - swing_pos_.col(i).head<2>();
                feet_pos.col(i).head<2>() = swing_pos_.col(i).head<2>()
                    + (3.0 * u * u - 2.0 * u * u * u) * delta
                    + remaining * u * (1.0 - u) * (1.0 - u) * swing_vel_.col(i).head<2>();
                feet_vel.col(i).head<2>() = 6.0 * u * (1.0 - u) / remaining * delta
                    + (1.0 - 4.0 * u + 3.0 * u * u) * swing_vel_.col(i).head<2>();
                swing_pos_.col(i) = feet_pos.col(i);
                swing_vel_.col(i) = feet_vel.col(i);
                swing_phase_last_(i) = wave_generator_->phase_(i);
            }
        }
    }
    contact_last_ = wave_generator_->contact_;
}

void GaitGenerator::restart() {
    first_run_ = true;
    startup_swing_started_.setZero();
    vxy_goal_.setZero();
    feet_end_calc_.init();
}


Vec3 GaitGenerator::getFootPos(const int i) {
    Vec3 foot_pos;

    foot_pos(0) =
            cycloidXYPosition(start_p_.col(i)(0), end_p_.col(i)(0), wave_generator_->phase_(i));
    foot_pos(1) =
            cycloidXYPosition(start_p_.col(i)(1), end_p_.col(i)(1), wave_generator_->phase_(i));
    foot_pos(2) = cycloidZPosition(start_p_.col(i)(2), gait_height_, wave_generator_->phase_(i));

    return foot_pos;
}


Vec3 GaitGenerator::getFootVel(const int i) {
    Vec3 foot_vel;

    foot_vel(0) =
            cycloidXYVelocity(start_p_.col(i)(0), end_p_.col(i)(0), wave_generator_->phase_(i));
    foot_vel(1) =
            cycloidXYVelocity(start_p_.col(i)(1), end_p_.col(i)(1), wave_generator_->phase_(i));
    foot_vel(2) = cycloidZVelocity(gait_height_, wave_generator_->phase_(i));

    return foot_vel;
}

double GaitGenerator::cycloidXYPosition(const double startXY, const double endXY, const double phase) {
    const double phase_pi = 2 * M_PI * phase;
    return (endXY - startXY) * (phase_pi - sin(phase_pi)) / (2 * M_PI) + startXY;
}

double GaitGenerator::cycloidZPosition(const double startZ, const double height, const double phase) {
    const double phase_pi = 2 * M_PI * phase;
    return height * (1 - cos(phase_pi)) / 2 + startZ;
}

double GaitGenerator::cycloidXYVelocity(const double startXY, const double endXY, const double phase) const {
    const double phase_pi = 2 * M_PI * phase;
    return (endXY - startXY) * (1 - cos(phase_pi)) / wave_generator_->get_t_swing();
}

double GaitGenerator::cycloidZVelocity(const double height, const double phase) const {
    const double phase_pi = 2 * M_PI * phase;
    return height * M_PI * sin(phase_pi) / wave_generator_->get_t_swing();
}
