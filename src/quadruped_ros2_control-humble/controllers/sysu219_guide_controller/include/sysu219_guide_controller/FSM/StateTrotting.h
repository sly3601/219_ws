//
// Created by tlab-uav on 24-9-18.
//

#ifndef STATETROTTING_H
#define STATETROTTING_H
#include <sysu219_guide_controller/control/BalanceCtrl.h>
#include <sysu219_guide_controller/gait/GaitGenerator.h>
#include <sysu219_guide_controller/control/ConvexMpcSolver.h>

#include "controller_common/FSM/FSMState.h"
// marker array可视化插件
#include "sysu219_guide_controller/debug/foot_marker_publisher.hpp"

class StateTrotting final : public FSMState {
public:
    explicit StateTrotting(CtrlInterfaces &ctrl_interfaces,
                           CtrlComponent &ctrl_component);

    void enter() override;

    void run(const rclcpp::Time &time,
             const rclcpp::Duration &period) override;

    void exit() override;

    FSMStateName checkChange() override;



    // 调试参数
    int troting_kalman; // 【已弃用】使用卡尔曼滤波位姿闭环（1），纯开环（0），仅roll/pitch闭环（2）
    
    enum class ForceSolverMode : uint8_t {
        QP  = 0,   // 走原来的 BalanceCtrl QP
        MPC = 1    // 走 Convex MPC
    };
    ForceSolverMode force_solver_mode_ = ForceSolverMode::MPC;

private:
    void getUserCmd();

    void calcCmd();

    /**
    * Calculate the torque command
    */
    void calcTau();

    /**
    * Calculate the joint space velocity and acceleration
    */
    void calcQQd();

    /**
    * Calculate the PD gain for the joints
    */
    void calcGain() const;

    /**
     * Check whether the robot should take a step or not
     * @return
     */
    bool checkStepOrNot();

    std::shared_ptr<Estimator> &estimator_;
    std::shared_ptr<QuadrupedRobot> &robot_model_;
    std::shared_ptr<BalanceCtrl> &balance_ctrl_;
    std::shared_ptr<WaveGenerator> &wave_generator_;
    std::shared_ptr<ConvexMpcSolver> &convex_mpc_;

    GaitGenerator gait_generator_;

    // Robot State
    Vec3 pos_body_, vel_body_;

    // P系原点跟随机身中心，但轴方向与G系保持平行，不随机身姿态转动
    // P: 定向本体系（origin at body center, axes parallel to G, no body rotation）
    // B系原点也在机身上，但轴方向随机身姿态转动
    RotMat B2G_RotMat, G2B_RotMat; // G是世界系，固定在地面上
    RotMat B2P_RotMat, P2B_RotMat; // P系原点跟随机身中心，轴方向不随机身转动，与G平行

    // Robot command
    Vec3 pcd_;
    Vec3 vel_target_, v_cmd_body_;
    double dt_;
    double yaw_cmd_{}, d_yaw_cmd_{}, d_yaw_cmd_past_{};
    // G系下目标期望角速度
    Vec3 w_cmd_global_;
    // G系下目标足底位置和速度
    Vec34 pos_feet_global_goal_, vel_feet_global_goal_;
    // G系下目标足底位置和速度
    Vec34 pos_feet_goal_G, vel_feet_goal_G;
    double hip_q_range; // 髋关节范围限制
    double hip_qd_range; // 髋关节速度限制

    RotMat Rd;  // 期望的躯体姿态的B2G旋转矩阵

    // Control Parameters
    double gait_height_;
    Vec3 pos_error_, vel_error_;
    Mat3 Kpp, Kdp, Kd_w_;
    double kp_w_; // lost
    double kp_roll_,kp_pitch_,kp_yaw_;
    Mat3 Kp_swing_, Kd_swing_;
    Vec2 v_x_limit_, v_y_limit_, w_yaw_limit_;
    double tau_ff_scale; 
    double tau_ff_limit_hip;
    double tau_ff_limit_thigh;
    double tau_ff_limit_calf;
    Vec3 dd_pcb_saturation;
    Vec3 d_wbd_saturation;
    Vec3 swing_force_limit;
    double Kp_motor_stance, Kd_motor_stance, Kp_motor_swing, Kd_motor_swing; // 电机增益参数

    // debug 参数
    Vec12 q_goal_debug;
    double debug_z_[4]; // 新增：足底高度调试信息
    std::unique_ptr<quadruped_controller::FootMarkerPublisher> foot_marker_pub_; // 新增：足底可视化发布器

    // 新增：发布频率控制计数器
    int publish_counter_ = 0;


    bool first_run = true; // 新增：是否第一次进入trotting状态的标志

    // MPC足底接触点记忆：记录每条腿最近一次进入支撑相时的足底世界坐标
    Vec34 mpc_foot_hold_G_;

    // MPC接触状态记忆：用于判断 swing -> stance 的落地瞬间
    // 上一个控制周期第 i 条腿的接触状态
    VecInt4 mpc_contact_last_;

    // MPC足底接触点是否已经初始化
    // 第一次进入 trotting 后，先初始化一次所有腿的足底点
    bool mpc_foot_hold_initialized_ = false;

};


#endif //STATETROTTING_H
