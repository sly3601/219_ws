//
// Created by tlab-uav on 25-2-27.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <sysu219_guide_controller/gait/WaveGenerator.h>

#include "BalanceCtrl.h"
#include "Estimator.h"
#include "ContactObservation.hpp"

#include <sysu219_guide_controller/control/ConvexMpcSolver.h>

struct CtrlComponent {
    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;
    std::shared_ptr<BalanceCtrl> balance_ctrl_;
    std::shared_ptr<WaveGenerator> wave_generator_;
    std::shared_ptr<ConvexMpcSolver> convex_mpc_;
    int odometry_mode = 1; // 0=旧KF，1=原InEKF，2=接触/滑移修复InEKF；仅配置时更换。
    bool contact_feedback_enabled = false, contact_observation_ready = false;
    sysu219_guide_controller::ContactObservation contact_observation;

    CtrlComponent() = default;
};
#endif //CTRLCOMPONENT_H
