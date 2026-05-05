//
// Created by biao on 24-9-18.
//

#include "unitree_guide_controller/gait/GaitGenerator.h"

#include <utility>
#include <unitree_guide_controller/control/CtrlComponent.h>
#include <unitree_guide_controller/control/Estimator.h>
#include <unitree_guide_controller/gait/WaveGenerator.h>
// 【新增】包含 StateTrotting 头文件
#include "unitree_guide_controller/FSM/StateTrotting.h"
#include <unitree_guide_controller/common/mathTools.h>
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
      ctrl_component_(ctrl_component),  // 【新增】必须初始化！
      trotting_ptr_(trotting_ptr), // 【新增】初始化指针
      first_run_(true) {
}



/* 纯赋值函数，无分析意义 */
void GaitGenerator::setGait(Vec2 vxy_goal_global, const double d_yaw_goal, const double gait_height) {
    vxy_goal_ = std::move(vxy_goal_global);
    d_yaw_goal_ = d_yaw_goal;
    gait_height_ = gait_height;
}

/* 核心：步态生成函数 */
void GaitGenerator::generate(Vec34 &feet_pos, Vec34 &feet_vel) {
    Vec34 current_feet_pos;
    if (first_run_) 
    {
        if (trotting_ptr_ && trotting_ptr_->troting_kalman == 2) 
        {
            // 闭环：原来的逻辑，用 estimator 的全局足端位置
            // 初始化时，start_p_就是当前的足端位置，后续每次落地都会刷新足底起始位置
            start_p_ = estimator_->getFeetPos();
            end_p_ = start_p_;
            // 同时记录“当时那一刻”的 B 系名义足底位置
            // 以后摆动终点就往这个名义位置回
            nominal_feet_body_ = estimator_->getFeetPos2Body();
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
        first_run_ = false;
    }

    // 遍历机器人的4条腿（0:右前 1:左前 2:右后 3:左后）
    for (int i = 0; i < 4; i++) 
    {
        // 条件1：当前腿处于支撑相（踩地）
        if (wave_generator_->contact_(i) == 1) 
        {
            // 支撑相位小于0.5：刷新支撑点（脚刚落地，锁定当前位置）
            if (wave_generator_->phase_(i) < 0.5) 
            {
                // foot contact the ground
                if (trotting_ptr_ && trotting_ptr_->troting_kalman == 1) 
                {
                    start_p_.col(i) = estimator_->getFootPos(i);
                }
                else if (trotting_ptr_ && trotting_ptr_->troting_kalman == 0)
                {
                    // 开环：只在first_run_初始化一次 start_p_，之后就完全不更新了（不依赖 estimator）
                }
                else if (trotting_ptr_ && trotting_ptr_->troting_kalman == 2)
                {
                    start_p_.col(i) = estimator_->getFootPos(i);   // 世界系落脚点
                }
                
            }
            feet_pos.col(i) = start_p_.col(i);
            feet_vel.col(i).setZero();// 这里本来就应该是零的，支撑相不需要移动，速度为0
        } 
        // 条件2：当前腿处于摆动相（抬脚迈步）
        else 
        {
            // foot not contact, swing
            // ==============================================
            // 核心修复：开环/闭环 分两套逻辑计算「迈步终点」
            // ==============================================
            // 【开环模式】：纯身体坐标系，不用估计器，不用calcFootPos
            if (trotting_ptr_ && (trotting_ptr_->troting_kalman == 0))
            {
                // 步长：每次迈步向前移动5厘米（可调节）
                double step_length = 0;
                // 方向规则：1向前，-1向后
                double dir = 1;

                // ==============================================
                // 核心修复：永远基于【固定的start_p】计算终点
                // 绝不基于上一次的end_p累加，彻底杜绝发散！
                // ==============================================
                end_p_.col(i) = start_p_.col(i);
                // end_p_(0, i) = start_p_.col(i)[0] + dir * step_length;
            }
            else if (trotting_ptr_ && (trotting_ptr_->troting_kalman == 2))
            {
                Vec3 body_vel_global = estimator_->getVelocity();
                Vec3 next_step;
                next_step.setZero();

                const double t_stance = wave_generator_->get_t_stance();
                const double t_swing  = wave_generator_->get_t_swing();

                const double k_x = 0.005;
                const double k_y = 0.005;
                // 原作者 x 方向落脚点预测项：
                // 1) 剩余摆动时间机身位移
                // 2) 半个支撑相机身位移
                // 3) x 方向速度误差修正
                next_step(0) = body_vel_global(0) * (1.0 - wave_generator_->phase_(i)) * t_swing
                            + body_vel_global(0) * t_stance / 2.0
                            + k_x * (body_vel_global(0) - vxy_goal_(0));

                next_step(1) = body_vel_global(1) * (1.0 - wave_generator_->phase_(i)) * t_swing
                            + body_vel_global(1) * t_stance / 2.0
                            + k_y * (body_vel_global(1) - vxy_goal_(1));

                // 给速度预测项限幅，防止一步修太猛
                next_step(0) = saturation(next_step(0), Vec2(-0.025, 0.025));
                next_step(1) = saturation(next_step(1), Vec2(-0.025, 0.025)); 
                const double yaw = estimator_->getYaw();
                const double d_yaw = estimator_->getDYaw();

                const double k_yaw = 0.005;
                double next_yaw = d_yaw * (1.0 - wave_generator_->phase_(i)) * t_swing
                    + d_yaw * t_stance / 2.0
                    + k_yaw * (d_yaw_goal_ - d_yaw);

                // yaw 预测也限一下，防止 cos/sin 的目标点跳太远
                next_yaw = saturation(next_yaw, Vec2(-0.1, 0.1));
                

                const double feet_radius =
                sqrt(pow(nominal_feet_body_(0, i), 2) + pow(nominal_feet_body_(1, i), 2));

                const double feet_init_angle =
                atan2(nominal_feet_body_(1, i), nominal_feet_body_(0, i));

                next_step(0) +=
                        feet_radius * cos(yaw + feet_init_angle + next_yaw);
                next_step(1) +=
                        feet_radius * sin(yaw + feet_init_angle + next_yaw);

                Vec3 foot_pos = estimator_->getPosition() + next_step;

                // 更新 x y 方向的落脚点预测值
                double target_x = foot_pos(0);
                double target_y = foot_pos(1);


                // 最终 x 落点相对当前摆动起点限幅
                target_x = saturation(target_x, Vec2(start_p_(0, i) - 0.020,
                                                    start_p_(0, i) + 0.020));
                target_y = saturation(target_y, Vec2(start_p_(1, i) - 0.020,
                                    start_p_(1, i) + 0.020));

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
                // end_body(0) = (1.0 - alpha) * start_body(0) + alpha * nominal_feet_body_(0, i);

                // // 再从 B 系变回当前 generate() 使用的外部系
                // end_p_.col(i) = pos_ext + B2P * end_body;
            }
            // 【闭环模式】：完全保留你原来的代码，用官方轨迹规划器
            else if (trotting_ptr_ && trotting_ptr_->troting_kalman == 1)
            {
                // 闭环：保持原来的逻辑不变
                end_p_.col(i) = feet_end_calc_.calcFootPos(i, vxy_goal_, d_yaw_goal_, wave_generator_->phase_(i));
            }

            // 调用你原有的摆线函数：计算平滑的足端轨迹/速度
            feet_pos.col(i) = getFootPos(i);
            feet_vel.col(i) = getFootVel(i);
        }
    }
}

void GaitGenerator::restart() {
    first_run_ = true;
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
