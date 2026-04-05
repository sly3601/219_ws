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



/* 纯赋值函数 */
void GaitGenerator::setGait(Vec2 vxy_goal_global, const double d_yaw_goal, const double gait_height) {
    vxy_goal_ = std::move(vxy_goal_global);
    d_yaw_goal_ = d_yaw_goal;
    gait_height_ = gait_height;
}

void GaitGenerator::generate(Vec34 &feet_pos, Vec34 &feet_vel) {
    // 【修复】先定义变量！你之前没定义，所以报错未定义！
    Vec34 current_feet_pos;
    if (first_run_) {
        if (trotting_ptr_ && trotting_ptr_->troting_kalman == 1) 
        {
            // 闭环：原来的逻辑，用 estimator 的全局足端位置
            start_p_ = estimator_->getFeetPos();
        }
        else if (trotting_ptr_ && trotting_ptr_->troting_kalman == 0) 
        {
            // 开环：用正运动学，完全不用 estimator
            auto feet_2b = ctrl_component_.robot_model_->getFeet2BPositions();
            for(int i=0; i<4; i++)
            {
                // 【修复】KDL::Vector → Eigen::Vector3d
                current_feet_pos.col(i) = Eigen::Vector3d(
                    feet_2b[i].p.x(),
                    feet_2b[i].p.y(),
                    feet_2b[i].p.z()
                );
            }
            // 【关键】开环时把正确的足端位置给 start_p_
            start_p_ = current_feet_pos;
        }
        first_run_ = false;
    }

    // 遍历机器人的4条腿（0:右前 1:左前 2:右后 3:左后）
    for (int i = 0; i < 4; i++) {
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
                
            }
            feet_pos.col(i) = start_p_.col(i);
            feet_vel.col(i).setZero();
        } 
        // 条件2：当前腿处于摆动相（抬脚迈步）
        else 
        {
            // foot not contact, swing
            // ==============================================
            // 核心修复：开环/闭环 分两套逻辑计算「迈步终点」
            // ==============================================
            // 【开环模式】：纯身体坐标系，不用估计器，不用calcFootPos
            if (trotting_ptr_ && trotting_ptr_->troting_kalman == 0)
            {
                // 步长：每次迈步向前移动5厘米（可调节）
                double step_length = 0;
                // 方向规则：对角腿(0+3)向前，(1+2)向后 → 标准Trot步态
                double dir = (i == 0 || i == 3) ? 1.0 : -1.0;

                // ==============================================
                // 核心修复：永远基于【固定的start_p】计算终点
                // 绝不基于上一次的end_p累加，彻底杜绝发散！
                // ==============================================
                end_p_.col(i) = start_p_.col(i);
                end_p_(0, i) = start_p_.col(i)[0] + dir * step_length;
            }
            // 【闭环模式】：完全保留你原来的代码，用官方轨迹规划器
            else
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
