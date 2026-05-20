//
// modified by sly on 26-05-07.
//

#include "unitree_guide_controller/FSM/StateTrotting.h"
#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/control/CtrlComponent.h>
#include <unitree_guide_controller/control/Estimator.h>
#include <unitree_guide_controller/gait/WaveGenerator.h>

/* 
P系：定向本体系
B系：机身坐标系
G系：全局坐标系
*/

/**
 * @brief Trotting状态类构造函数
 * @param ctrl_interfaces 控制接口（包含指令输入、关节控制接口等）
 * @param ctrl_component 控制组件（包含估计器、机器人模型等核心模块）
 */
StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces,
                             CtrlComponent &ctrl_component) : 
    FSMState(FSMStateName::TROTTING, "trotting", ctrl_interfaces),
    estimator_(ctrl_component.estimator_),
    robot_model_(ctrl_component.robot_model_),
    balance_ctrl_(ctrl_component.balance_ctrl_),
    convex_mpc_(ctrl_component.convex_mpc_),
    wave_generator_(ctrl_component.wave_generator_),
    gait_generator_(ctrl_component, this){

    troting_kalman = 2;                             //总模式开关【已弃用】
    force_solver_mode_ = ForceSolverMode::QP;       //总模式开关

    hip_q_range = 0.16;                               // 髋关节限制范围（±0.16 rad，约 ±9.2°）
    hip_qd_range = 1.0;                                // 髋关节速度限制（±1.0 rad/s）

    Kp_motor_stance = 300;     // 支撑相电机位置增益
    Kd_motor_stance = 4.5;     // 支撑相电机速度增益
    Kp_motor_swing = 220;       // 摆动相电机位置增益
    Kd_motor_swing = 3.8;         // 摆动相电机速度增益

    gait_height_ = 0.07;                            // 足底摆动高度
    Kpp = Vec3(36, 36, 300.1).asDiagonal();         // 身体位置比例增益
    Kdp = Vec3(5.2, 5.2, 5.0).asDiagonal();         // 身体速度阻尼增益

    // roll/pitch/yaw 姿态比例增益
    kp_pitch_ = 450;
    kp_roll_ = 450;
    kp_yaw_ = 16.2;
    Kd_w_ = Vec3(4.1, 5.1, 3.1).asDiagonal();       // 姿态角速度阻尼增益

    Kp_swing_ = Vec3(0.3, 0.3, 0.3).asDiagonal();   // 摆动相位置增益
    Kd_swing_ = Vec3(0.1, 0.1, 0.1).asDiagonal();   // 摆动相速度阻尼

    // QP/MPC优化相关参数
    if (force_solver_mode_ == ForceSolverMode::QP) 
    {
        tau_ff_scale = 1.0;                             // QP力分配衰减系数
        tau_ff_limit_hip = 34.0;                        // QP力分配前馈力矩限制：髋关节
        tau_ff_limit_thigh = 82.0;                      // QP力分配前馈力矩限制：大腿关节
        tau_ff_limit_calf = 100.0;                      // QP力分配前馈力矩限制：小腿关节
    }
    else if (force_solver_mode_ == ForceSolverMode::MPC)
    {
        tau_ff_scale = 0.0;                             // MPC力分配衰减系数
        tau_ff_limit_hip = 30.0;                        // MPC力分配前馈力矩限制：髋关节
        tau_ff_limit_thigh = 30.0;                      // MPC力分配前馈力矩限制：大腿关节
        tau_ff_limit_calf = 30.0;                       // MPC力分配前馈力矩限制：小腿关节
    }

    // 摆动腿闭环相关参数
    swing_force_limit = Vec3(5.0, 5.0, 10.0);       // 摆动腿期望足底力限制：x/y/z方向（N）

    dd_pcb_saturation = Vec3(3.2, 3.2, 10.5);       // 身体最大期望加速度限制(m/s2)
    d_wbd_saturation = Vec3(60.0, 70.0, 36.0);      // 身体最大期望角加速度限制(rad/s2)

    v_x_limit_ << -0.2, 0.2;                        // 机身期望x速度限制
    v_y_limit_ << -0.1, 0.1;                        // 机身期望y速度限制
    w_yaw_limit_ << -0.2, 0.2;                      // 机身期望yaw角速度限制
    

    
    dt_ = 1.0 / ctrl_interfaces_.frequency_;        // 控制周期dt
    // 初始化足底可视化发布器
    foot_marker_pub_ = std::make_unique<quadruped_controller::FootMarkerPublisher>(this->ctrl_interfaces_.node);
}



/**
 * @brief 进入Trotting状态时的初始化操作
 */
void StateTrotting::enter() {
    // 原地稳定踏步，世界系位置目标取进入时当前位置
    pcd_ = estimator_->getPosition();                   // 机身期望位置初始化
    v_cmd_body_.setZero();                              // 机身期望速度初始化
    yaw_cmd_ = estimator_->getYaw();                    // 机身期望yaw角初始化
    const double roll_des = 0.0;                        // 机身期望roll角初始化
    const double pitch_des = 0.05;                      // 机身期望pitch角初始化
    Rd = rotz(yaw_cmd_) * roty(pitch_des) * rotx(roll_des);
    w_cmd_global_.setZero();                            //机身期望角速度初始化
       
    first_run = true; // 标记为第一次进入trotting状态
    wave_generator_->status_ = WaveStatus::STANCE_ALL;  // 初始化过渡状态：全支撑
    ctrl_interfaces_.control_inputs_.command = 0;       // 将控制输入指令重置为0（避免残留指令影响）
    gait_generator_.restart();                          // 重启步态生成器

    mpc_foot_hold_G_.setZero();
    mpc_contact_last_.setZero();
    mpc_foot_hold_initialized_ = false;
}

/**
 * @brief Trotting状态run函数
 * @param time 当前时间
 * @param period 时间间隔
 */
void StateTrotting::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {

    pos_body_ = estimator_->getPosition();          // 获取当前身体位置（G系）
    vel_body_ = estimator_->getVelocity();          // 获取当前机身速度（G系）
    B2P_RotMat = estimator_->getRotation();         // 获取B2G_RotMat
    P2B_RotMat = B2P_RotMat.transpose();

    getUserCmd();                                   // 上位机输入指令
    calcCmd();                                      // 解析上位机指令  

    /**
     * @brief 步态生成器代码段（核心）
     * @param vel_target_               输入：期望的机身xyz速度（G系）
     * @param w_cmd_global_           输入：期望的机身yaw角速度（G系）
     * @param gait_height_              输入：足底摆动高度
     * 
     * @param pos_feet_goal_G   输出：期望的足底位置（G系）
     * @param vel_feet_goal_G   输出：期望的足底速度（G系）
     */
    gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_global_(2), gait_height_);
    gait_generator_.generate(pos_feet_goal_G, vel_feet_goal_G);
    
    calcTau();                                      // 动力学计算总函数（核心）
    calcQQd();                                      // 运动学计算总函数（核心)

    // 首次状态切换，防止死锁
    if(first_run)
    {
        wave_generator_->status_ = WaveStatus::STANCE_ALL;
        first_run = false;
    }
    else
    {
        wave_generator_->status_ = WaveStatus::WAVE_ALL;
    }

    
    calcGain();                                     // 设置关节MIT控制增益

    // 更新并发布足底Marker1
    // 发布频率为控制频率的1/10
    std::array<geometry_msgs::msg::Point, 4> foot_positions;
    foot_positions[0].x = pos_feet_goal_G(0,0);  
    foot_positions[0].y = pos_feet_goal_G(1,0); 
    foot_positions[0].z = pos_feet_goal_G(2,0); // FR
    foot_positions[1].x = pos_feet_goal_G(0,1);  
    foot_positions[1].y = pos_feet_goal_G(1,1); 
    foot_positions[1].z = pos_feet_goal_G(2,1); // FL
    foot_positions[2].x = pos_feet_goal_G(0,2); 
    foot_positions[2].y = pos_feet_goal_G(1,2); 
    foot_positions[2].z = pos_feet_goal_G(2,2); // RR
    foot_positions[3].x = pos_feet_goal_G(0,3); 
    foot_positions[3].y = pos_feet_goal_G(1,3); 
    foot_positions[3].z = pos_feet_goal_G(2,3); // RL
    publish_counter_++;
    if (publish_counter_ >= 10) {
        foot_marker_pub_->update(foot_positions);
        foot_marker_pub_->publish();
        publish_counter_ = 0;
    }
}

/**
 * @brief 退出Trotting状态时的收尾操作
 */
void StateTrotting::exit() {
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

/**
 * @brief 检查状态切换条件（判断是否需要从Trotting切换到其他状态）
 * @return 目标FSM状态名称
 */
FSMStateName StateTrotting::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::TROTTING;
    }
}

/**
 * @brief 解析用户输入指令（将遥控器摇杆值转换为机器人速度指令）
 */
void StateTrotting::getUserCmd() { // 该函数P/B/G坐标系混乱 还没改

    // 读取上位机输入期望值
    double ly_raw = ctrl_interfaces_.control_inputs_.ly;
    double lx_raw = ctrl_interfaces_.control_inputs_.lx;
    double rx_raw = ctrl_interfaces_.control_inputs_.rx;

    const double deadzone = 0.05; // 死区阈值
    if (fabs(ly_raw) < deadzone) ly_raw = 0.0;
    if (fabs(lx_raw) < deadzone) lx_raw = 0.0;
    if (fabs(rx_raw) < deadzone) rx_raw = 0.0;

    /* 平移速度指令解析 */
    // 将遥控器左摇杆y轴（ly）值反归一化，转换为身体坐标系x方向速度指令
    v_cmd_body_(0) = invNormalize(ctrl_interfaces_.control_inputs_.ly, v_x_limit_(0), v_x_limit_(1));
    // 将遥控器左摇杆x轴（lx）值反归一化并取反，转换为身体坐标系y方向速度指令
    v_cmd_body_(1) = -invNormalize(ctrl_interfaces_.control_inputs_.lx, v_y_limit_(0), v_y_limit_(1));
    // 身体坐标系z方向速度指令设为0（Trotting步态不考虑垂直平移）
    v_cmd_body_(2) = 0;

    /* 旋转角速度指令解析：提高滤波权重，增强平滑性，减少姿态扰动 */
    // 将遥控器右摇杆x轴（rx）值反归一化并取反，转换为yaw轴角速度指令
    d_yaw_cmd_ = -invNormalize(ctrl_interfaces_.control_inputs_.rx, w_yaw_limit_(0), w_yaw_limit_(1));
    // 提高历史值权重至0.98，大幅降低指令突变带来的失衡风险
    d_yaw_cmd_ = 0.98 * d_yaw_cmd_past_ + (1 - 0.98) * d_yaw_cmd_;
    // 保存当前yaw轴角速度指令为历史值，用于下一个周期滤波
    d_yaw_cmd_past_ = d_yaw_cmd_;
}

/**
 * @brief 计算全局坐标系下的期望控制指令（位置、速度、姿态）
 */
void StateTrotting::calcCmd() {
    // 这个函数还没有改完，troting_kalman =1的逻辑即将移植出来，然后删除troting_kalman这个变量
    if(troting_kalman == 1)
    {
        /* 平移指令：身体坐标系转全局坐标系 */
        // 将身体坐标系下的速度指令v_cmd_body_通过旋转矩阵转换为全局坐标系下的目标速度vel_target_
        vel_target_ = B2G_RotMat * v_cmd_body_;

        // 如果你想加限制，限制速度本身就好，不要限制位置跟随实际值
        vel_target_(0) = saturation(vel_target_(0), Vec2(-0.2, 0.2));
        vel_target_(1) = saturation(vel_target_(1), Vec2(-0.1, 0.1));
        vel_target_(2) = 0; // Z轴速度始终为0// 全局坐标系z方向目标速度设为0（Trotting步态保持身体高度稳定）

        // 直接更新期望位置，不要用 pos_body_ 来做饱和边界！
        pcd_(0) += vel_target_(0) * dt_;
        pcd_(1) += vel_target_(1) * dt_;

        // // 限幅，默认注释掉，要用的时候再加
        // // 进一步缩小速度饱和范围，减少大惯性下的姿态突变
        // vel_target_(0) = saturation(vel_target_(0), Vec2(vel_body_(0) - 0.08, vel_body_(0) + 0.08));
        // vel_target_(1) = saturation(vel_target_(1), Vec2(vel_body_(1) - 0.08, vel_body_(1) + 0.08));

        // // 更新期望身体x/y位置：缩小调节范围，增强稳定性
        // pcd_(0) = saturation(pcd_(0) + vel_target_(0) * dt_, Vec2(pos_body_(0) - 0.05, pos_body_(0) + 0.05));
        // pcd_(1) = saturation(pcd_(1) + vel_target_(1) * dt_, Vec2(pos_body_(1) - 0.05, pos_body_(1) + 0.05));
        // // 显式更新z轴期望位置：适配Kpp(z)的高增益，有效抑制身体下沉
        // pcd_(2) = saturation(pcd_(2) + vel_target_(2) * dt_, Vec2(pos_body_(2) - 0.1, pos_body_(2) + 0.1));


        /* 旋转指令：更新期望偏航角和角速度 */
        // 积分yaw轴角速度指令，得到期望偏航角（当前期望角+角速度×控制周期）
        yaw_cmd_ = yaw_cmd_ + d_yaw_cmd_ * dt_;
        // 更新期望旋转矩阵Rd为绕z轴旋转当前期望偏航角的矩阵
        Rd = rotz(yaw_cmd_);
        // 全局坐标系下的yaw轴角速度指令设为滤波后的d_yaw_cmd_
        w_cmd_global_(2) = d_yaw_cmd_;
    }

    else if(troting_kalman == 2)
    {
        vel_target_.setZero();          // 目标平移速度 = 0，还未引入除了原地踏步以外的运动指令
    }
}

/**
 * @brief 动力学计算总函数
 * 
 */

void StateTrotting::calcTau() {
    // 运动学/动力学参数
    Vec3 dd_pcd;                                // 期望机身xyz加速度（m/s²）
    dd_pcd.setZero();
    Vec3 d_wbd;                                 // 期望机身角加速度（rad/s²）
    d_wbd.setZero();

    Vec34 pos_feet_G;                           // 足端位置（G系）
    pos_feet_G.setZero();
    Vec34 pos_feet_P;                           // 足端位置（P系）
    pos_feet_P.setZero();
    Vec34 pos_feet_B;                           // 足端位置（B系）
    pos_feet_B.setZero();

    Vec34 vel_feet_G;                           // 足端速度（G系）
    vel_feet_G.setZero();

    Vec34 force_feet_P;                         // 足底反力（P系）
    force_feet_P.setZero();
    Vec34 force_feet_B;                         // 足底反力（B系）
    force_feet_B.setZero();
    Vec3 gyro_global;                           // 机身角速度（G/P系）
    gyro_global.setZero();

    // PD控制相关变量
    Vec3 rot_err;                               // 姿态误差旋转矩阵
    rot_err.setZero();

    // 计算过程中间变量
    std::vector<KDL::Frame> feet_frames_body;

    // 机身x/y/z 平面闭环 PD控制
    pos_error_ = pcd_ - pos_body_;
    vel_error_ = vel_target_ - vel_body_;
    dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-dd_pcb_saturation(0), dd_pcb_saturation(0)));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-dd_pcb_saturation(1), dd_pcb_saturation(1)));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-dd_pcb_saturation(2), dd_pcb_saturation(2)));

    // 机身roll/pitch/yaw 姿态闭环 PD控制
    rot_err = rotMatToExp(Rd * P2B_RotMat);
    gyro_global = estimator_->getGyroGlobal();
    d_wbd(0) = kp_roll_  * rot_err(0) + Kd_w_(0,0) * (0.0 - gyro_global(0));
    d_wbd(1) = kp_pitch_ * rot_err(1) + Kd_w_(1,1) * (0.0 - gyro_global(1));
    d_wbd(2) = kp_yaw_   * rot_err(2) + Kd_w_(2,2) * (0.0 - gyro_global(2));
    d_wbd(0) = saturation(d_wbd(0), Vec2(-d_wbd_saturation(0), d_wbd_saturation(0)));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-d_wbd_saturation(1), d_wbd_saturation(1)));
    d_wbd(2) = saturation(d_wbd(2), Vec2(-d_wbd_saturation(2), d_wbd_saturation(2)));

    // 获取当前B系足端位置
    feet_frames_body = robot_model_->getFeet2BPositions();
    for (int i = 0; i < 4; ++i) {
        pos_feet_B.col(i) = Vec3(feet_frames_body[i].p.data);
    }
    
    pos_feet_P = B2P_RotMat * pos_feet_B;       // 得到P系下的足端位置
    pos_feet_G = estimator_->getFeetPos();      // 当前四个足端在G系下的实际位置
    vel_feet_G = estimator_->getFeetVel();      // 当前四个足端在G系下的实际速度
    
    if (!B2P_RotMat.allFinite() || 
    !pos_feet_P.allFinite() || 
    !pos_feet_G.allFinite() || 
    !vel_feet_G.allFinite()) 
    {
        for (int k = 0; k < 12; ++k)
            ctrl_interfaces_.joint_torque_command_interface_[k].get().set_value(0.0);
        return;
    }


    // 以下都是为了MPC作准备：
    // 初始化 MPC 接触点记忆
    // 第一次进入 trotting 后，直接用当前实际足端 G 系位置初始化
    if (!mpc_foot_hold_initialized_) {
        mpc_foot_hold_G_ = pos_feet_G;
        mpc_contact_last_ = wave_generator_->contact_;
        mpc_foot_hold_initialized_ = true;
    }

    // 只在 swing -> stance 的落地瞬间更新该腿的支撑点
    for (int leg = 0; leg < 4; ++leg) {
        const bool touchdown =
            (mpc_contact_last_(leg) == 0 && wave_generator_->contact_(leg) == 1);

        if (touchdown) {
            mpc_foot_hold_G_.col(leg) = pos_feet_G.col(leg);
        }
    }

    // 记录上一周期接触状态
    mpc_contact_last_ = wave_generator_->contact_;
    // MPC步态准备结束




    /**
     * @brief QP优化足底力分配（核心）
     * @param dd_pcd                    输入：期望的机身xyz加速度
     * @param d_wbd                     输入：期望的机身角加速度
     * @param B2P_RotMat                输入：B系到P系的旋转矩阵
     * @param pos_feet_P                输入：P系下的足端位置
     * @param wave_generator_->contact_ 输入：当前步态周期内的足端接触状态（0或1）
     * @param force_feet_P              输出：QP优化得到的P系下的足底反力
     */
    try 
    {
        if (force_solver_mode_ == ForceSolverMode::QP) {
            // calF函数内部计算出来的是P系下的地面对机身的反作用力，我们需要的是足端对地面的力，所以加负号取反
            force_feet_P = -balance_ctrl_->calF(dd_pcd, d_wbd, B2P_RotMat, pos_feet_P, wave_generator_->contact_);
        } 
        else if (force_solver_mode_ == ForceSolverMode::MPC) {
            const double gait_period = wave_generator_->get_t();
            const double stance_ratio = wave_generator_->get_t_stance() / wave_generator_->get_t();

            // Convex MPC 里动力学方程默认用的是“地面对机身的接触力”，所以下游做 J^T f 时同样需要取负号得到“足端对地的力”
            force_feet_P = -convex_mpc_->solveFromDogWrench(
                dd_pcd,
                mpc_foot_hold_G_,
                gait_generator_.getEndFeetPos(),
                wave_generator_->contact_,
                wave_generator_->phase_,
                dt_,
                gait_period,
                stance_ratio,
                pos_body_,
                vel_body_,
                B2P_RotMat,
                gyro_global,
                Rd,
                vel_target_
            );
        }

    }
    catch (...) 
    {
        for (int k = 0; k < 12; ++k) 
            ctrl_interfaces_.joint_torque_command_interface_[k].get().set_value(0.0); 
        return; 
    }

    // 摆动腿跟随闭环PD控制
    for (int i = 0; i < 4; ++i)
    {
        // contact == 0 表示摆动腿
        if (wave_generator_->contact_(i) == 0)
        {
            Vec3 swing_force =
                Kp_swing_ * (pos_feet_goal_G.col(i) - pos_feet_G.col(i)) +
                Kd_swing_ * (vel_feet_goal_G.col(i) - vel_feet_G.col(i));

            swing_force(0) = saturation(swing_force(0), Vec2(-swing_force_limit(0), swing_force_limit(0)));
            swing_force(1) = saturation(swing_force(1), Vec2(-swing_force_limit(1), swing_force_limit(1)));
            swing_force(2) = saturation(swing_force(2), Vec2(-swing_force_limit(2), swing_force_limit(2)));

            force_feet_P.col(i) = swing_force;
        }
    }

    // 将足端力从P系转换为B系
    force_feet_B = P2B_RotMat * force_feet_P;

    // 遍历4条腿，计算每条腿的关节力矩并赋值给控制接口
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_->getTorque(force_feet_B.col(i), i);  // 逆解
        for (int j = 0; j < 3; j++) 
        {
            double tau_cmd = tau_ff_scale * torque(j);

            if (j == 0) {
                tau_cmd = saturation(tau_cmd, Vec2(-tau_ff_limit_hip, tau_ff_limit_hip));
            } else if (j == 1) {
                tau_cmd = saturation(tau_cmd, Vec2(-tau_ff_limit_thigh, tau_ff_limit_thigh));
            } else {
                tau_cmd = saturation(tau_cmd, Vec2(-tau_ff_limit_calf, tau_ff_limit_calf));
            }
            ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(tau_cmd);
        }
    }

    
    // 发布调试信息
    if (ctrl_interfaces_.debug_pub) {
        std_msgs::msg::Float64MultiArray msg;

        // 0: 高度误差 pos_error_z
        msg.data.push_back(pos_error_(2));
        // 1-3: xyz方向期望控制输出机身加速度 dd_pcd
        msg.data.push_back(dd_pcd(0));
        msg.data.push_back(dd_pcd(1));
        msg.data.push_back(dd_pcd(2));
        msg.data.push_back(d_wbd(0));     // 4: roll方向目标角加速度项
        msg.data.push_back(d_wbd(1));     // 5: pitch方向目标角加速度项
        msg.data.push_back(d_wbd(2));     // 6: yaw方向目标角加速度项

        ctrl_interfaces_.debug_pub->publish(msg);
    }
}

/**
 * @brief 运动学计算总函数
 */
void StateTrotting::calcQQd() {
    // 逆运动学准备
    Vec12 q_goal;
    Vec12 qd_goal;
    q_goal.setZero();
    qd_goal.setZero();

    Vec34 pos_feet_target_B, vel_feet_target_B;

    // 将足端目标位置和速度从G系转换为B系
    for (int i = 0; i < 4; ++i) {
        pos_feet_target_B.col(i) = P2B_RotMat * (pos_feet_goal_G.col(i) - pos_body_);
        vel_feet_target_B.col(i) = P2B_RotMat * (vel_feet_goal_G.col(i) - vel_body_);
    }
    // 关节位置/速度逆解
    q_goal = robot_model_->getQ(pos_feet_target_B);
    std::vector<KDL::Frame> pos_feet_target_frame(4);
    for (int i = 0; i < 4; ++i) {
        pos_feet_target_frame[i].p = KDL::Vector(
            pos_feet_target_B(0, i),
            pos_feet_target_B(1, i),
            pos_feet_target_B(2, i)
        );
        pos_feet_target_frame[i].M = KDL::Rotation::Identity();
    }
    qd_goal = robot_model_->getQd(pos_feet_target_frame, vel_feet_target_B);

    // hip小范围限制
    const double hip_center = 0.0;    // 髋关节中心位置（0 rad）
    const double hip_min = hip_center - hip_q_range;
    const double hip_max = hip_center + hip_q_range;

    // 关节限幅
    for (int leg_idx = 0; leg_idx < 4; ++leg_idx) 
    {
        const int hip_idx   = leg_idx * 3 + 0;
        const int thigh_idx = leg_idx * 3 + 1;
        const int calf_idx  = leg_idx * 3 + 2;

        // 髋关节位置限幅
        q_goal(hip_idx)  = saturation(q_goal(hip_idx),   Vec2(hip_min,   hip_max));
        // 髋关节速度限制，防止突然抽动
        qd_goal(hip_idx) = saturation(qd_goal(hip_idx), Vec2(-hip_qd_range, hip_qd_range));

    }

    // 将关节目标位置和速度赋值给控制接口
    for (int i = 0; i < 12; i++) {
        q_goal_debug = q_goal; // 用于调试，发布到ROS2话题
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }
}

/**
 * @brief 设置关节MIT kp kd增益
 */
void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) {
        // 大腿和小腿
        for (int j = 1; j < 3; j++) {
            if (wave_generator_->contact_(i) == 0) {
                // ================= 摆动相 =================
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(Kp_motor_swing);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(Kd_motor_swing);  
            } else {
                // ================= 支撑相 =================
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(Kp_motor_stance);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(Kd_motor_stance);  
            }
        }

        // 单独设置髋关节
        int hip_idx = i * 3 + 0;
        if (wave_generator_->contact_(i) == 0) 
        {
            ctrl_interfaces_.joint_kp_command_interface_[hip_idx].get().set_value(Kp_motor_swing);
            ctrl_interfaces_.joint_kd_command_interface_[hip_idx].get().set_value(Kd_motor_swing);
        } 
        else 
        {
            ctrl_interfaces_.joint_kp_command_interface_[hip_idx].get().set_value(Kp_motor_stance);
            ctrl_interfaces_.joint_kd_command_interface_[hip_idx].get().set_value(Kd_motor_stance);
        }
    }
}

/**
 * @brief 判断机器人是否需要迈步（大幅降低阈值，优先保持全支撑状态）
 * @return true-需要迈步，false-不需要迈步
 */
bool StateTrotting::checkStepOrNot() {
    // 极低阈值：减少迈步频率，让四条腿长期处于全支撑状态，通过均匀受力解决后腿塌陷
    if (fabs(v_cmd_body_(0)) > 0.01 || fabs(v_cmd_body_(1)) > 0.01 ||
        fabs(pos_error_(0)) > 0.08 || fabs(pos_error_(1)) > 0.08 ||
        fabs(vel_error_(0)) > 0.02 || fabs(vel_error_(1)) > 0.02 ||
        fabs(d_yaw_cmd_) > 0.1) {
        return true; // 满足条件则迈步
    }
    return false; // 优先保持全支撑状态，增强整体稳定性
}