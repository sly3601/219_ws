//
// modified by sly on 26-05-07.
//

#include "sysu219_guide_controller/FSM/StateTrotting.h"
#include <sysu219_guide_controller/common/mathTools.h>
#include <sysu219_guide_controller/control/CtrlComponent.h>
#include <sysu219_guide_controller/control/Estimator.h>
#include <sysu219_guide_controller/gait/WaveGenerator.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include "sysu219_guide_controller/debug/DebugConfig.h"

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
    wave_generator_(ctrl_component.wave_generator_),
    convex_mpc_(ctrl_component.convex_mpc_),
    gait_generator_(ctrl_component, this){

    troting_kalman = 2;                             //总模式开关【已弃用】
    force_solver_mode_ = ForceSolverMode::MPC;       //总模式开关

    hip_q_range = 16.16;                               // 髋关节限制范围（±0.16 rad，约 ±9.2°）
    hip_qd_range = 10.0;                                // 髋关节速度限制（±1.0 rad/s）

    // MPC 支撑腿不叠加位置弹簧，避免足端位置估计误差产生反向收腿力矩。
    Kp_motor_stance = force_solver_mode_ == ForceSolverMode::MPC ? 0.0 : 300.0;
    Kd_motor_stance = 4.5;     // 支撑相电机速度增益
    Kp_motor_swing = 160;       // 摆动相电机位置增益
    Kd_motor_swing = 3.8;         // 摆动相电机速度增益

    gait_height_ = 0.07;                            // 足底摆动高度
    Kpp = Vec3(36, 36, 300.1).asDiagonal();         // 仅用于 QP 模式：机身位置比例增益
    Kdp = Vec3(12.2, 12.2, 35.0).asDiagonal();      // 仅用于 QP 模式：机身速度阻尼增益；MPC 调 Q/R

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
        tau_ff_scale = 1.0;                             // MPC力分配衰减系数
        tau_ff_limit_hip = 100.0;                        // MPC力分配前馈力矩限制：髋关节
        tau_ff_limit_thigh = 100.0;                      // MPC力分配前馈力矩限制：大腿关节
        tau_ff_limit_calf = 100.0;                      // MPC力分配前馈力矩限制：小腿关节
    }

    // 摆动腿闭环相关参数
    swing_force_limit = Vec3(5.0, 5.0, 10.0);       // 摆动腿期望足底力限制：x/y/z方向（N）

    dd_pcb_saturation = Vec3(3.2, 3.2, 10.5);       // 身体最大期望加速度限制(m/s2)
    d_wbd_saturation = Vec3(60.0, 70.0, 36.0);      // 身体最大期望角加速度限制(rad/s2)

    v_x_limit_ << -0.2, 0.2;                        // 机身期望x速度限制
    v_y_limit_ << -0.1, 0.1;                        // 机身期望y速度限制
    w_yaw_limit_ << -0.2, 0.2;                      // 机身期望yaw角速度限制
    

    
    dt_ = 1.0 / ctrl_interfaces_.frequency_;        // 控制周期dt

    // 新增：WBC采用较温和的任务PD，电机原有MIT增益仍由calcGain管理。
    sysu219::wbc::WbcSettings wbc_settings;
    wbc_settings.kp_orientation = Vec3(40.0, 40.0, 20.0);
    wbc_settings.kd_orientation = Vec3(8.0, 8.0, 4.0);
    wbc_settings.kp_body_position = Vec3(16.0, 16.0, 40.0);
    wbc_settings.kd_body_position = Vec3(8.0, 8.0, 12.0);
    wbc_settings.kp_swing = Vec3(80.0, 80.0, 100.0);
    wbc_settings.kd_swing = Vec3(12.0, 12.0, 16.0);
    wbc_settings.svd_absolute_tolerance = 1e-8;
    wbc_settings.svd_relative_tolerance = 1e-6;
    wbc_ = std::make_unique<sysu219::wbc::WbcController>(wbc_settings);

    // 两组权重分别惩罚基座加速度修正和足底力修正，按当前设置协调两者。
    sysu219::wbc::RelaxationSettings relaxation_settings;
    relaxation_settings.Q_WBC = 1 * Mat6::Identity();
    relaxation_settings.Q_MPC = 1e4 * Mat12::Identity();
    relaxation_ = std::make_unique<sysu219::wbc::MpcWbcRelaxation>(relaxation_settings);

    if (quadruped_debug::Csv_DebugMode)               // 如果开启了输出CSV数据文件的debug模式
    {
        trotting_debug_ = std::make_unique<TrottingDebug>(dt_);  // 创建一个 TrottingDebug 对象，把 dt_ 传给它的构造函数，并返回管理该对象的智能指针。
    }
    
    // 初始化足底可视化发布器
    foot_marker_pub_ = std::make_unique<quadruped_controller::FootMarkerPublisher>(this->ctrl_interfaces_.node);
}



/**
 * @brief 进入Trotting状态时的初始化操作
 */
void StateTrotting::enter() {
    if (trotting_debug_) trotting_debug_->reset();
    debug_cycle_ = 0;
    pcd_ = estimator_->getPosition();                   // 机身期望位置初始化，设为当前机身位置
    height_target_ = pcd_(2);                                                   // 保留原来的最终目标高度
    height_ramp_elapsed_ = 0.0;                                                 // 累计高度过渡持续的时间，初始化
    height_ramp_duration_ = std::max(dt_, wave_generator_->getGaitPeriod());            // 高度过渡持续时间，至少为一个控制周期或一个步态周期

    v_cmd_body_.setZero();                              // 机身期望速度初始化
    yaw_cmd_ = estimator_->getYaw();                    // 机身期望yaw角初始化
    const double roll_des = 0.0;                        // 机身期望roll角初始化
    const double pitch_des = rotMatToRPY(estimator_->getRotation())(1);         // 目标pitch是进入troting时的 FixStand pitch。
    Rd = rotz(yaw_cmd_) * roty(pitch_des) * rotx(roll_des);                     // 机身期望姿态初始化，保持进入时的水平站姿
    
    // 名义关节角与 FixStand 共用 stand_pos；足点只旋转到 yaw 系，不额外平移。
    gait_generator_.setNominalStand(
        ctrl_interfaces_.node->get_parameter("stand_pos").as_double_array(),    // 获取站立关节角数组
        rotz(yaw_cmd_).transpose() * Rd);               // 这里是yaw系下的目标yaw，yaw系就是“只保留yaw、去掉俯仰和横滚的机身系”。
    w_cmd_global_.setZero();                            //机身期望角速度初始化
       
    first_run = true; // 标记为第一次进入trotting状态
    wave_generator_->status_ = WaveStatus::STANCE_ALL;  // 首个控制周期先建立全支撑
    ctrl_interfaces_.control_inputs_.command = 0;       // 将控制输入指令重置为0（避免残留指令影响）
    gait_generator_.restart();                          // 重启步态生成器

    mpc_foot_hold_G_.setZero();                         // 初始化MPC足底支撑点位置（G系）                    
    mpc_contact_last_.setZero();                        // 初始化MPC上一次足底接触状态
    mpc_foot_hold_initialized_ = false;                 // 标记MPC足底支撑点位置未初始化
    mpc_cycle_ = 0;                                     // 初始化MPC循环计数器,外部控制250hz，mpc目前是50hz，mpc_cycle_每5个外部控制周期增加1
    mpc_force_P_.setZero();                             // 初始化MPC计算的足底力（P系）
    mpc_frame_valid_ = false;
    wbc_output_ = sysu219::wbc::WbcOutput{};
    gyro_control_B_ = estimator_->getGyro();            // 初始化机身角速度（B系）

    // 初始化 角速度滤波器 的历史值，确保第一次运行时不会出现异常
    // gyro_raw_history1_是上一次的原始角速度，gyro_raw_history2_是上上次的原始角速度
    // gyro_filtered_history1_是上一次的滤波角速度，gyro_filtered_history2_是上上次的滤波角速度
    gyro_raw_history1_ = gyro_raw_history2_ = gyro_filtered_history1_ = gyro_filtered_history2_ = gyro_control_B_;

    convex_mpc_->reset(); // 清除上次进入 Trotting 的成功解，避免跨状态复用。
}

/**
 * @brief Trotting状态run函数
 * @param time 当前时间
 * @param period 时间间隔
 */
void StateTrotting::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    
    if (quadruped_debug::Csv_DebugMode) {
        debug_frame_ = TrottingDebug::Frame{};                          // 初始化debug_frame_为默认构造的Frame对象
        debug_frame_.contact = wave_generator_->contact_;               // 获取当前足底接触状态
        debug_frame_.phase = wave_generator_->phase_;                   // 获取当前足底相位
        debug_frame_.transition = wave_generator_->getSwitchStatus();   // 获取当前足底接触切换状态
        debug_frame_.meta[TrottingDebug::WAVE] = static_cast<int>(wave_generator_->status_);    // 获取当前步态状态，例如全支撑 STANCE_ALL、正常步态 WAVE_ALL
    }

    pos_body_ = estimator_->getPosition();          // 获取当前身体位置（G系）
    vel_body_ = estimator_->getVelocity();          // 获取当前机身速度（G系）
    B2P_RotMat = estimator_->getRotation();         // 获取B2G_RotMat
    P2B_RotMat = B2P_RotMat.transpose();

    // 20 Hz 二阶 Butterworth：让缓慢变化的角速度通过，减弱快速抖动和高频噪声。
    // 在控制频率下滤波，再抽取给 MPC，减轻高频混叠。
    // 保留低频姿态反馈；不修改估计器中的原始 IMU 数据。
    if (force_solver_mode_ == ForceSolverMode::MPC) 
    {
        const Vec3 gyro = estimator_->getGyro();                    // 获取当前机身角速度（B系）
        const double k = std::tan(M_PI * 20.0 * dt_);               // 计算二阶Butterworth滤波器的截止频率参数k，20.0是截止频率，dt_是控制周期
        const double a0 = 1.0 + std::sqrt(2.0) * k + k * k;         // 计算二阶Butterworth滤波器的归一化系数a0
        const double b0 = k * k / a0;                               // 计算二阶Butterworth滤波器的归一化系数b0
        const double a1 = 2.0 * (k * k - 1.0) / a0;                 // 计算二阶Butterworth滤波器的归一化系数a1
        const double a2 = (1.0 - std::sqrt(2.0) * k + k * k) / a0;  // 计算二阶Butterworth滤波器的归一化系数a2
        
        // 使用二阶Butterworth滤波器对角速度进行滤波，得到滤波后的角速度gyro_control_B_
        gyro_control_B_ = b0 * (gyro + 2.0 * gyro_raw_history1_ + gyro_raw_history2_)
                          - a1 * gyro_filtered_history1_ - a2 * gyro_filtered_history2_;

        gyro_raw_history2_ = gyro_raw_history1_;
        gyro_raw_history1_ = gyro;
        gyro_filtered_history2_ = gyro_filtered_history1_;
        gyro_filtered_history1_ = gyro_control_B_;
    }

    getUserCmd();                                   // 上位机输入指令
    calcCmd();                                      // 解析上位机指令  


    // 以下模块：平滑调整机身高度 z
    // 首帧取接触切换后的估计高度，一个步态周期内平滑衔接原目标。
    if (height_ramp_elapsed_ < height_ramp_duration_)
    {
        if (first_run) height_start_ = pos_body_(2);
        const double u = std::min(1.0, height_ramp_elapsed_ / height_ramp_duration_); // 计算高度过渡的归一化时间u，范围在[0,1]之间
        const double blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));              // 计算高度过渡的平滑插值因子blend，使高度平滑变化，起止速度和加速度均为零
        const double height_delta = height_target_ - height_start_;                   // 计算高度变化量height_delta，即目标高度与起始高度的差值
        pcd_(2) = height_start_ + height_delta * blend;                               // 计算当前机身期望高度pcd_(2)，根据起始高度、目标高度和插值因子blend进行平滑过渡
        // 速度参考与高度曲线一致，起止速度均为零。
        vel_target_(2) = height_delta * 30.0 * u * u * (1.0 - u) * (1.0 - u)          // 计算当前机身期望高度的速度参考vel_target_(2)，由高度曲线求导得到，确保起止速度为零
            / height_ramp_duration_;
        height_ramp_elapsed_ = std::min(height_ramp_duration_,                        // 累计高度过渡时间，确保不超过设定的高度过渡持续时间
            height_ramp_elapsed_ + wave_generator_->getControlDt());
    }


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

    if (quadruped_debug::Csv_DebugMode)
    {
        auto& sample = debug_frame_.wbc;
        sample.blend = static_cast<float>(wbc_command_blend_);
        if (wbc_enabled_) sample.flags |= TrottingDebug::WBC_ENABLED;
        if (mpc_frame_valid_) sample.flags |= TrottingDebug::MPC_FRAME_VALID;
        if (robot_model_->wbc_model_) sample.flags |= TrottingDebug::MODEL_READY;
        if (std::isfinite(wbc_command_blend_) && wbc_command_blend_ > 0.0 && wbc_command_blend_ <= 1.0)
            sample.flags |= TrottingDebug::BLEND_VALID;
    }

    // 新增：MPC和WBC均已完成，本周期再协调加速度和接触力。
    // calcTau/calcQQd已写好旧输出；只有整条新链路成功，才一起替换位置、速度和力矩。
    if (wbc_enabled_ && force_solver_mode_ == ForceSolverMode::MPC && mpc_frame_valid_ &&
        wbc_output_.success && std::isfinite(wbc_command_blend_) && wbc_command_blend_ > 0.0 &&
        wbc_command_blend_ <= 1.0)
    {
        relaxation_input_.qddot_cmd = wbc_output_.qddot_d;
        relaxation_input_.contact = wbc_input_.contact;
        relaxation_input_.J_f_O = wbc_input_.J_f_O;
        relaxation_input_.Jdot_f_qdot_O = wbc_input_.Jdot_f_qdot_O;
        // 电机内部已做MIT PD，这里只输出逆动力学前馈，避免再次叠加PD力矩。
        relaxation_input_.tau_PD_j.setZero();
        const auto relaxation_begin = quadruped_debug::Csv_DebugMode ?
            std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
        if (quadruped_debug::Csv_DebugMode) debug_frame_.wbc.flags |= TrottingDebug::RELAX_ATTEMPT;
        const auto result = relaxation_->solve(relaxation_input_);
        if (quadruped_debug::Csv_DebugMode)
        {
            auto& sample = debug_frame_.wbc;
            sample.relaxation_ms = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - relaxation_begin).count();
            if (result.success)
            {
                sample.flags |= TrottingDebug::RELAX_OK;
                sample.delta_qddot = result.delta_qddot.cast<float>();
                sample.delta_force = Eigen::Map<const Vec12>(result.delta_f_O.data()).cast<float>();
                sample.stance_acceleration = Eigen::Map<const Vec12>(result.stance_acceleration_residual_O.data()).cast<float>();
                sample.equality_residual = static_cast<float>(result.equality_residual_inf);
                sample.inequality_margin = static_cast<float>(result.minimum_inequality_margin);
                sample.candidate_tau_delta = (tau_ff_scale * result.tau_j - legacy_tau_).cast<float>();
            }
        }

        // 力修正量仅用于日志，不再作为接入回退门槛。
        const double force_change = result.delta_f_O.cwiseAbs().maxCoeff();
        bool accept = result.success;
        const Vec12 wbc_tau = tau_ff_scale * result.tau_j;
        for (int joint = 0; accept && joint < 12; ++joint)
        {
            const double limit = joint % 3 == 0 ? tau_ff_limit_hip :
                (joint % 3 == 1 ? tau_ff_limit_thigh : tau_ff_limit_calf);
            // 超过前馈限幅时整帧回退，避免裁剪逆动力学力矩后仍使用新关节目标。
            accept = std::isfinite(wbc_tau(joint)) && std::abs(wbc_tau(joint)) <= limit;
        }

        static rclcpp::Clock wbc_clock(RCL_STEADY_TIME);
        if (accept)
        {
            // [调试过渡] 小比例混合用于从已有稳定控制逐步切到完整WBC，不属于论文QP。
            // 比例为1才完全采用WBC/逆动力学；混合期间不宣称执行力矩严格满足新模型动力学。
            Vec12 q = legacy_q_ + wbc_command_blend_ * (wbc_output_.q_j_d - legacy_q_);
            Vec12 qd = legacy_qd_ + wbc_command_blend_ * (wbc_output_.qdot_j_d - legacy_qd_);
            const Vec12 tau = legacy_tau_ + wbc_command_blend_ * (wbc_tau - legacy_tau_);
            for (int joint = 0; joint < 12; ++joint)
            {
                double lower = robot_model_->joint_lower_(joint);
                double upper = robot_model_->joint_upper_(joint);
                double velocity = robot_model_->joint_velocity_limit_(joint);
                if (joint % 3 == 0)
                {
                    lower = std::max(lower, -hip_q_range);
                    upper = std::min(upper, hip_q_range);
                    velocity = std::min(velocity, hip_qd_range);
                }
                const double sampled_q = q(joint), sampled_qd = qd(joint);
                q(joint) = std::clamp(q(joint), lower, upper);
                qd(joint) = std::clamp(qd(joint), -velocity, velocity);
                if ((q(joint) <= lower && qd(joint) < 0.0) ||
                    (q(joint) >= upper && qd(joint) > 0.0)) qd(joint) = 0.0;
                if (quadruped_debug::Csv_DebugMode && (q(joint) != sampled_q || qd(joint) != sampled_qd))
                    debug_frame_.wbc.flags |= TrottingDebug::COMMAND_CLAMPED;
                ctrl_interfaces_.joint_position_command_interface_[joint].get().set_value(q(joint));
                ctrl_interfaces_.joint_velocity_command_interface_[joint].get().set_value(qd(joint));
                ctrl_interfaces_.joint_torque_command_interface_[joint].get().set_value(tau(joint));
            }
            q_goal_debug = q;
            if (quadruped_debug::Csv_DebugMode)
            {
                debug_frame_.wbc.flags |= TrottingDebug::WBC_APPLIED;
                debug_frame_.q_cmd = q;
                debug_frame_.qd_cmd = qd;
                RCLCPP_INFO_THROTTLE(ctrl_interfaces_.node->get_logger(), wbc_clock, 10000,
                    "[MPC_WBC] 本帧采用WBC，混合比例=%.2f，最大力修正=%.3e N，基座残差=%.2e，支撑足加速度残差=%.2f m/s²",
                    wbc_command_blend_, force_change, result.floating_base_dynamics_residual.norm(),
                    result.stance_acceleration_residual_O.norm());
            }
        }
        else
        {
            if (quadruped_debug::Csv_DebugMode && result.success)
                debug_frame_.wbc.flags |= TrottingDebug::TORQUE_REJECTED;
            RCLCPP_WARN_THROTTLE(ctrl_interfaces_.node->get_logger(), wbc_clock, 10000,
                "[MPC_WBC] 本帧沿用原控制输出：%s",
                result.success ? "前馈力矩无效或超过关节前馈限值" : result.message.c_str());
        }
    }
    if (first_run) wave_generator_->status_ = WaveStatus::WAVE_ALL;
    first_run = false;


    // 设置关节MIT控制增益，在第一版MPC，支撑相KP值为0，支撑相KD值为4.5，完全依赖MPC计算的力矩来控制支撑相关节。
    // 摆动相KP值为160，KD值为3.8，保证摆动相关节快速到达目标位置。
    calcGain();                                     
    

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
    if (publish_counter_ >= 10) 
    {
        foot_marker_pub_->update(foot_positions);
        foot_marker_pub_->publish();
        publish_counter_ = 0;
    }
}

/**
 * @brief 退出Trotting状态时的收尾操作
 */
void StateTrotting::exit() {
    if (trotting_debug_) trotting_debug_->finish();
    wave_generator_->status_ = WaveStatus::STANCE_ALL;
}

void StateTrotting::recordDebug(const rclcpp::Time& time, const rclcpp::Duration& period,
                              std::chrono::steady_clock::time_point update_begin, long long system_begin) {
    if (!quadruped_debug::Csv_DebugMode || !trotting_debug_) return;
    auto& f = debug_frame_;
    f.meta[TrottingDebug::CYCLE] = debug_cycle_++;
    f.meta[TrottingDebug::ROS_S] = time.seconds();
    f.meta[TrottingDebug::STEADY_S] = std::chrono::duration<double>(update_begin.time_since_epoch()).count();
    f.meta[TrottingDebug::SYSTEM_S] = system_begin * 1e-6;
    f.meta[TrottingDebug::PHASE_CONTROL_S] = wave_generator_->getPhaseControlTime();
    f.meta[TrottingDebug::PERIOD_S] = period.seconds();
    f.meta[TrottingDebug::MODE] = static_cast<int>(force_solver_mode_);
    const auto& solver = convex_mpc_->debugInfo();
    const bool mpc = force_solver_mode_ == ForceSolverMode::MPC;
    f.meta[TrottingDebug::SOLVER_ID] = mpc ? solver.id : balance_ctrl_->debugId();
    f.meta[TrottingDebug::SOLVER_STATUS] = mpc ? solver.status : balance_ctrl_->debugStatus();
    f.meta[TrottingDebug::SOLVER_ITER] = mpc ? solver.iter : -1;
    f.meta[TrottingDebug::SOLVER_RESULT] = mpc ? solver.result : balance_ctrl_->debugStatus() == 0 ? 1 : 0;
    for (int i = 0; i < 10; ++i) f.imu(i) = ctrl_interfaces_.imu_state_interface_[i].get().get_value();
    f.rpy = rotMatToRPY(B2P_RotMat);
    f.rpy_ref = rotMatToRPY(Rd);
    f.gyro_G = B2P_RotMat * (mpc ? gyro_control_B_ : estimator_->getGyro());
    // WBC成功时记录其实际使用的角速度，包含姿态副本的微小归一化修正。
    if (f.wbc.flags & TrottingDebug::WBC_OK)
        f.gyro_G = wbc_input_.R_OB * wbc_input_.qdot.head<3>();
    f.wbc.omega_d = w_cmd_global_.cast<float>();
    f.wbc.legacy_q = legacy_q_.cast<float>();
    f.wbc.legacy_qd = legacy_qd_.cast<float>();
    f.wbc.legacy_tau = legacy_tau_.cast<float>();
    f.p = pos_body_; f.v = vel_body_; f.p_ref = pcd_; f.v_ref = vel_target_;
    f.com_p = pos_body_ + B2P_RotMat * convex_mpc_->comOffsetBody();
    f.feet_G = estimator_->getFeetPos();
    f.feet_v_G = estimator_->getFeetVel();
    f.v_predicted = estimator_->debugVelocityPredicted();
    f.v_unfiltered = estimator_->debugVelocityUnfiltered();
    f.acc_G = estimator_->debugAcceleration();
    f.goal_G = pos_feet_goal_G; f.vgoal_G = vel_feet_goal_G;
    f.start_G = gait_generator_.getStartFeetPos(); f.end_G = gait_generator_.getEndFeetPos();
    f.hold_G = mpc_foot_hold_G_;
    for (int leg = 0; leg < 4; ++leg) {
        f.feet_B.col(leg) = Vec3(robot_model_->getFeet2BPositions(leg).p.data);
        f.q.segment<3>(3 * leg) = robot_model_->current_joint_pos_[leg].data;
        f.qd.segment<3>(3 * leg) = robot_model_->current_joint_vel_[leg].data;
    }
    for (int j = 0; j < 12; ++j) {
        if (static_cast<size_t>(j) < ctrl_interfaces_.joint_effort_state_interface_.size())
            f.tau_feedback(j) = ctrl_interfaces_.joint_effort_state_interface_[j].get().get_value();
        f.tau_ff_cmd(j) = ctrl_interfaces_.joint_torque_command_interface_[j].get().get_value();
        f.kp_cmd(j) = ctrl_interfaces_.joint_kp_command_interface_[j].get().get_value();
        f.kd_cmd(j) = ctrl_interfaces_.joint_kd_command_interface_[j].get().get_value();
        // 保留实际发送的前馈、增益和目标；MIT力矩及相对原输出的变化可离线重算。
    }
    // 仅记录电机反馈力矩对应的力代理，不能当作实测接触力或用于提前分配支撑力。
    f.touchdown_fz_threshold_N = 0.15 * robot_model_->mass_ * 9.81;
    for (int leg = 0; mpc && leg < 4; ++leg) {
        const Mat3 jacobian_G = B2P_RotMat * robot_model_->getJacobian(leg).data.topRows(3);
        if (!jacobian_G.allFinite()) continue;
        const Eigen::FullPivLU<Mat3> lu(jacobian_G.transpose());
        if (lu.isInvertible() && lu.rcond() > 1e-3 && f.tau_feedback.segment<3>(3 * leg).allFinite()) {
            const Vec3 force_proxy = lu.solve(-f.tau_feedback.segment<3>(3 * leg));
            f.touchdown_fz_proxy(leg) = force_proxy(2);
        }
    }
    f.meta[TrottingDebug::UPDATE_MS] = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - update_begin).count();
    trotting_debug_->push(f);
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
    mpc_frame_valid_ = false; // 不允许异常周期使用上一周期的WBC/松弛结果。
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

    // 误差保留用于调试；机身外层 PD 仅在 QP 模式计算。
    pos_error_ = pcd_ - pos_body_;
    vel_error_ = vel_target_ - vel_body_;
    gyro_global = estimator_->getGyroGlobal();
    if (force_solver_mode_ == ForceSolverMode::QP) 
    {
        dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
        dd_pcd(0) = saturation(dd_pcd(0), Vec2(-dd_pcb_saturation(0), dd_pcb_saturation(0)));
        dd_pcd(1) = saturation(dd_pcd(1), Vec2(-dd_pcb_saturation(1), dd_pcb_saturation(1)));
        dd_pcd(2) = saturation(dd_pcd(2), Vec2(-dd_pcb_saturation(2), dd_pcb_saturation(2)));

        rot_err = rotMatToExp(Rd * P2B_RotMat);
        d_wbd(0) = kp_roll_  * rot_err(0) + Kd_w_(0,0) * (0.0 - gyro_global(0));
        d_wbd(1) = kp_pitch_ * rot_err(1) + Kd_w_(1,1) * (0.0 - gyro_global(1));
        d_wbd(2) = kp_yaw_   * rot_err(2) + Kd_w_(2,2) * (0.0 - gyro_global(2));
        d_wbd(0) = saturation(d_wbd(0), Vec2(-d_wbd_saturation(0), d_wbd_saturation(0)));
        d_wbd(1) = saturation(d_wbd(1), Vec2(-d_wbd_saturation(1), d_wbd_saturation(1)));
        d_wbd(2) = saturation(d_wbd(2), Vec2(-d_wbd_saturation(2), d_wbd_saturation(2)));
    }


    // 获取当前B系足端位置
    feet_frames_body = robot_model_->getFeet2BPositions();
    for (int i = 0; i < 4; ++i) {
        pos_feet_B.col(i) = Vec3(feet_frames_body[i].p.data);    // 获取当前B系足端位置
    }
    
    pos_feet_P = B2P_RotMat * pos_feet_B;       // 得到P系下的足端位置
    pos_feet_G = estimator_->getFeetPos();      // 当前四个足端在G系下的实际位置
    vel_feet_G = estimator_->getFeetVel();      // 当前四个足端在G系下的实际速度
    
    if (!B2P_RotMat.allFinite() || 
    !pos_feet_P.allFinite() || 
    !pos_feet_G.allFinite() || 
    !vel_feet_G.allFinite())    // 检查旋转矩阵和足端位置、速度是否为有限值
    {
        if (quadruped_debug::Csv_DebugMode) debug_frame_.meta[TrottingDebug::FLAGS] = 1;    // 标记异常状态
        for (int k = 0; k < 12; ++k)
            ctrl_interfaces_.joint_torque_command_interface_[k].get().set_value(0.0);       // 将所有关节力矩命令设置为0，避免异常行为
        return;
    }


    // 以下都是为了MPC作准备：
    // 初始化 MPC 接触点记忆
    // 第一次进入 trotting 后，直接用当前实际足端 G 系位置初始化
    if (!mpc_foot_hold_initialized_) {
        mpc_foot_hold_G_ = pos_feet_G;                      // 初始化 MPC 足底支撑点位置为当前实际足端 G 系位置
        mpc_contact_last_ = wave_generator_->contact_;      // 初始化 MPC 上一次足底接触状态为当前接触状态
        mpc_foot_hold_initialized_ = true;                  // 标记 MPC 足底支撑点位置已初始化
    }

    const bool mpc_contact_changed = (mpc_contact_last_ != wave_generator_->contact_);  // 检查当前接触状态是否与上一次接触状态不同，判断是否发生接触切换
    // 接触切换时，下面会立即额外同步求解，而不是继续复用旧力。

    // 仅在调试模式下打印MPC接触切换信息，避免频繁打印影响性能
    if (quadruped_debug::Csv_DebugMode && force_solver_mode_ == ForceSolverMode::MPC && mpc_cycle_ != 0 &&
        mpc_contact_changed) 
    {
        static rclcpp::Clock hold_clock(RCL_STEADY_TIME);
        RCLCPP_INFO_THROTTLE(ctrl_interfaces_.node->get_logger(), hold_clock, 200,
            "[MPC_CONTACT_UPDATE] cycle=%d sync_solve=1 "
            "prev_contact=[%d %d %d %d] contact=[%d %d %d %d] "
            "previous_ground_fz=[%.3f %.3f %.3f %.3f]",
            mpc_cycle_,
            mpc_contact_last_(0), mpc_contact_last_(1), mpc_contact_last_(2), mpc_contact_last_(3),
            wave_generator_->contact_(0), wave_generator_->contact_(1),
            wave_generator_->contact_(2), wave_generator_->contact_(3),
            -mpc_force_P_(2, 0), -mpc_force_P_(2, 1),
            -mpc_force_P_(2, 2), -mpc_force_P_(2, 3));
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


        // 这段是抬腿前提前卸载：脚即将抬起时，先减小它允许承担的支撑力，让其他腿接过去。
        Vec4 liftoff_fz_limit = Vec4::Constant(1e19);   // 初始化足底离地前的垂直力限制为一个非常大的值，表示没有限制
        bool liftoff_pending = false;                   // 标记是否有足底即将离地的情况，初始为false
        
        // 仅全支撑起步时有其他支撑腿接载；双足交替不能提前卸空当前两腿。
        if (force_solver_mode_ == ForceSolverMode::MPC && wave_generator_->contact_.sum() == 4) 
        {
            const auto near_contact = wave_generator_->getMpcContactTable(
                4, wave_generator_->getControlDt());    // 获取未来4个控制周期内的接触状态表，判断哪些腿即将离地，getControlDt函数是获取dt_的值，250hz就是0.004秒

            for (int leg = 0; leg < 4; ++leg) 
            {
                if (wave_generator_->contact_(leg) != 1) continue;      // 如果当前腿不是接触状态，则跳过该腿

                // 检查未来接触状态表中该腿是否即将离地
                for (int tick = 1; tick < static_cast<int>(near_contact.size()); ++tick) 
                {
                    if (near_contact[tick][leg] != 0) continue;     // 如果未来接触状态表中该腿仍然是接触状态，则继续检查下一个tick

                    // 这句是在计算：第 leg 条腿即将抬起前，允许承担的竖直支撑力上限。
                    liftoff_fz_limit(leg) = std::max(1.0,           
                        -mpc_force_P_(2, leg) * static_cast<double>(tick - 1) / tick);      // mpc_force_P_(2, leg)：缓存的上次 MPC 足端力。2 是 z 方向。tick：预测还有几个控制周期，这条腿会进入摆动
                    liftoff_pending = true;
                    break;
                }
            }
        }


        if (force_solver_mode_ == ForceSolverMode::QP) 
        {
            // calF函数内部计算出来的是P系下的地面对机身的反作用力，我们需要的是足端对地面的力，所以加负号取反
            force_feet_P = -balance_ctrl_->calF(dd_pcd, d_wbd, B2P_RotMat, pos_feet_P, wave_generator_->contact_);
        } 
        // 正常每 5 周期同步求解（50 Hz）；接触变化时当周期额外求解。
        else if (force_solver_mode_ == ForceSolverMode::MPC &&
                 (mpc_cycle_ == 0 || mpc_contact_changed || liftoff_pending))   // 在 MPC 模式下，如果当前周期是第 0 周期，或者接触状态发生变化，或者有足底即将离地的情况，则进行同步求解
        {
            // 每次求解前刷新当前支撑脚；该次预测内仍按固定支点建模。
            for (int leg = 0; leg < 4; ++leg) {
                if (wave_generator_->contact_(leg) == 1)
                    mpc_foot_hold_G_.col(leg) = pos_feet_G.col(leg);    // 如果当前腿是接触状态，则将该腿的 MPC 足底支撑点位置更新为当前实际足端 G 系位置
            }
            const double gait_period = wave_generator_->getGaitPeriod();        // 获取当前一个步态周期的时间

            const auto mpc_begin = quadruped_debug::Csv_DebugMode ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point{};
            const double prediction_dt = 5.0 * dt_;
            
            const auto contact_table = wave_generator_->getMpcContactTable(
                convex_mpc_->predictionSteps(prediction_dt, gait_period), prediction_dt);   // predictionSteps函数限制预测时长不超过半个步态周期
            
            
                // Convex MPC 里动力学方程默认用的是“地面对机身的接触力”，所以下游做 J^T f 时同样需要取负号得到“足端对地的力”
            force_feet_P = -convex_mpc_->solveFromDogWrench(
                pcd_,                               // 期望机身位置，G 系
                mpc_foot_hold_G_,                   // MPC 足底支撑点位置，G 系
                gait_generator_.getEndFeetPos(),    // 预测期望足端位置，G 系
                wave_generator_->contact_,          // 当前接触状态，0 或 1
                contact_table,                      // 当前及未来预测接触状态表，0 或 1
                prediction_dt,                      // MPC 每 5 个控制周期更新：预测步长 20 ms（50 Hz）。
                gait_period,                        // 当前完整步态周期，单位秒
                pos_body_,                          // 当前机身位置，G 系
                vel_body_,                          // 当前机身速度，G 系
                B2P_RotMat,                         // B 系到 P 系的旋转矩阵
                B2P_RotMat * gyro_control_B_,       // 当前机身角速度，P 系
                Rd,                                 // 期望机身姿态，G 系   
                vel_target_,                        // 期望机身速度，G 系
                liftoff_fz_limit                    // 四脚当前允许的竖直支撑力上限，单位 N，用于离地前卸载
            );

            // 完整 MPC 调用耗时；每次调用都统计，每秒输出一次，避免漏掉尖峰。
            // 包含准备、求解及保底返回；不包含估计器、IK、硬件 read/write。
            if (quadruped_debug::Csv_DebugMode) 
            {
                const auto mpc_end = std::chrono::steady_clock::now();
                const double total_ms =
                    std::chrono::duration<double, std::milli>(mpc_end - mpc_begin).count();
                debug_frame_.meta[TrottingDebug::MPC_TOTAL_MS] = total_ms;
                static auto report_begin = mpc_begin;
                static double sum_ms = 0.0, max_ms = 0.0;
                static size_t samples = 0, over_budget = 0;
                sum_ms += total_ms;
                max_ms = std::max(max_ms, total_ms);
                ++samples;
                if (total_ms > dt_ * 1000.0) ++over_budget;

                // 统计MPC求解耗时，每秒输出一次平均值、最大值和超预算次数，便于性能分析和调优。
                if (mpc_end - report_begin >= std::chrono::seconds(1)) {
                    RCLCPP_INFO(
                        ctrl_interfaces_.node->get_logger(),
                        "[MPC_TOTAL] last_ms=%.3f avg_ms=%.3f max_ms=%.3f "
                        "nominal_step_ms=%.3f over_budget=%zu/%zu",
                        total_ms, sum_ms / static_cast<double>(samples), max_ms,
                        dt_ * 1000.0, over_budget, samples);
                    report_begin = mpc_end;
                    sum_ms = max_ms = 0.0;
                    samples = over_budget = 0;
                }
            }
            mpc_force_P_ = force_feet_P;            // 缓存 MPC 求解得到的 P 系下的足底反力，用于下一周期的力分配和离地前卸载
        }
        if (force_solver_mode_ == ForceSolverMode::MPC) 
        {
            force_feet_P = mpc_force_P_;            // 在非MPC执行周期中，使用缓存的 MPC 求解得到的 P 系下的足底反力，避免频繁调用 MPC 求解器
            mpc_cycle_ = (mpc_cycle_ + 1) % 5;
            for (int leg = 0; leg < 4; ++leg)
            {
                auto& limits = relaxation_input_.force_limits[leg];
                limits.mu = convex_mpc_->frictionCoefficient();
                limits.f_z_min = convex_mpc_->normalForceMin();
                limits.f_z_max = std::min(convex_mpc_->normalForceMax(), liftoff_fz_limit(leg));
            }
        }

    }
    catch (...)     // 这是前面 try 代码抛出异常时的处理：
    {
        if (quadruped_debug::Csv_DebugMode) debug_frame_.meta[TrottingDebug::FLAGS] = 2;
        for (int k = 0; k < 12; ++k) 
            ctrl_interfaces_.joint_torque_command_interface_[k].get().set_value(0.0); 
        return; 
    }

    if (quadruped_debug::Csv_DebugMode)
        debug_frame_.ground_force_P = -force_feet_P; // 在摆腿 PD 覆盖之前记录规划接触力。把当前求解器输出保存到调试记录。取负号后，记录的是地面对机器人的支撑力。

    // 新增：此处的force_feet_P已取过负号；松弛模块要地面对机器人的反力，必须转回正号。
    // 在摆动腿PD覆盖前保存，且不改写原MPC缓存或下一次MPC的变化率参考。
    relaxation_input_.f_MPC_O = -force_feet_P;


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
    for (int i = 0; i < 4; i++) 
    {
        KDL::JntArray torque = robot_model_->getTorque(force_feet_B.col(i), i);  // 逆解
        for (int j = 0; j < 3; j++) 
        {
            double tau_cmd = tau_ff_scale * torque(j);

            if (j == 0) 
            {
                tau_cmd = saturation(tau_cmd, Vec2(-tau_ff_limit_hip, tau_ff_limit_hip));
            } 
            
            else if (j == 1) 
            {
                tau_cmd = saturation(tau_cmd, Vec2(-tau_ff_limit_thigh, tau_ff_limit_thigh));
            } 
            else 
            {
                tau_cmd = saturation(tau_cmd, Vec2(-tau_ff_limit_calf, tau_ff_limit_calf));
            }
            ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(tau_cmd);
            legacy_tau_(i * 3 + j) = tau_cmd;
        }
    }

    mpc_frame_valid_ = true;
}

/**
 * @brief 运动学计算总函数
 */
void StateTrotting::calcQQd() {
    wbc_output_ = sysu219::wbc::WbcOutput{}; // 每周期清除成功标志，禁止复用旧接触状态的解。
    // 逆运动学准备
    Vec12 q_goal;
    Vec12 qd_goal;
    q_goal.setZero();
    qd_goal.setZero();

    Vec34 pos_feet_target_B, vel_feet_target_B;
    const Vec3 omega_B = force_solver_mode_ == ForceSolverMode::MPC     // MPC模式下，使用滤波后的机身角速度作为角速度反馈，增强稳定性
        ? gyro_control_B_ : estimator_->getGyro();

    // 将足端目标位置和速度从G系转换为B系
    for (int i = 0; i < 4; ++i) {
        pos_feet_target_B.col(i) = P2B_RotMat * (pos_feet_goal_G.col(i) - pos_body_);
        // 对 r_B = R_G2B * (p_foot_G - p_body_G) 求导，须扣除机身转动项。
        vel_feet_target_B.col(i) = P2B_RotMat * (vel_feet_goal_G.col(i) - vel_body_)
                                 - omega_B.cross(pos_feet_target_B.col(i));
    }
    const bool debug = quadruped_debug::Csv_DebugMode;
    q_goal = robot_model_->getQ(pos_feet_target_B,
        debug ? &debug_frame_.ik_q : nullptr, debug ? &debug_frame_.fk_q : nullptr);
    const Vec12 debug_q_raw = q_goal;
    const Vec34 debug_vgoal_B = vel_feet_target_B;


    // 关节边界检查和速度限幅
    Vec12 lower = robot_model_->joint_lower_, upper = robot_model_->joint_upper_;
    Vec12 velocity_limit = robot_model_->joint_velocity_limit_;
    for (int leg = 0; leg < 4; ++leg) 
    {
        const int hip = 3 * leg;
        lower(hip) = std::max(lower(hip), -hip_q_range);
        upper(hip) = std::min(upper(hip), hip_q_range);
        velocity_limit(hip) = std::min(velocity_limit(hip), hip_qd_range);
        // 非有限逆解保持该腿当前角度，并取消该腿速度目标。
        if (!q_goal.segment<3>(hip).allFinite()) {
            q_goal.segment<3>(hip) = robot_model_->current_joint_pos_[leg].data;
            vel_feet_target_B.col(leg).setZero();
        }
        for (int j = hip; j < hip + 3; ++j) {
            if (!std::isfinite(q_goal(j))) q_goal(j) = 0.5 * (lower(j) + upper(j));
            q_goal(j) = std::clamp(q_goal(j), lower(j), upper(j));
        }
    }

    // 速度使用同一组限位后的角度，避免重复 IK 把膝关节推向伸直奇异点。
    qd_goal = robot_model_->getQd(q_goal, vel_feet_target_B,
        debug ? &debug_frame_.sigma_qd : nullptr);
    const Vec12 debug_qd_raw = qd_goal;

    // 速度限幅和关节边界检查
    for (int j = 0; j < 12; ++j) {
        qd_goal(j) = std::clamp(qd_goal(j), -velocity_limit(j), velocity_limit(j));
        if ((q_goal(j) <= lower(j) && qd_goal(j) < 0.0) ||
            (q_goal(j) >= upper(j) && qd_goal(j) > 0.0)) qd_goal(j) = 0.0;
    }
    q_goal_debug = q_goal;
    legacy_q_ = q_goal;
    legacy_qd_ = qd_goal;
    // 将关节目标位置和速度赋值给控制接口
    for (int i = 0; i < 12; i++) {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }

    if (debug) {
        debug_frame_.goal_B = pos_feet_target_B;
        debug_frame_.vgoal_B = debug_vgoal_B;
        debug_frame_.q_raw = debug_q_raw;
        debug_frame_.qd_raw = debug_qd_raw;
        // 保留 CSV 列名：此处不再有第二次 IK，记录实际用于速度求解的 q 的误差。
        debug_frame_.ik_qd = debug_frame_.ik_q;
        for (int leg = 0; leg < 4; ++leg) {
            KDL::JntArray leg_q(3);
            leg_q.data = q_goal.segment<3>(3 * leg);
            const auto foot = robot_model_->calcPEe2B_four_feet(leg, leg_q);
            debug_frame_.fk_qd(leg) = (Vec3(foot.p.data) - pos_feet_target_B.col(leg)).norm();
        }
        debug_frame_.q_cmd = q_goal;
        debug_frame_.qd_cmd = qd_goal;
    }

    // 新增：先保留上面的原IK输出作为回退，再在总运动学函数内计算完整WBC。
    // 新关节目标暂不写电机，等run的松弛优化成功后与新力矩一起应用。
    if (wbc_enabled_ && force_solver_mode_ == ForceSolverMode::MPC && mpc_frame_valid_ &&
        robot_model_->wbc_model_)
    {
        auto& input = wbc_input_;
        input.R_OB = B2P_RotMat;
        input.Theta_d = rotMatToRPY(Rd);
        input.p_com_O = pos_body_;
        input.p_com_d_O = pcd_;
        input.pdot_com_d_O = vel_target_;
        input.omega_d_O = w_cmd_global_;
        input.pddot_com_d_O.setZero();
        input.alpha_d_O.setZero();
        input.qdot.head<3>() = omega_B;
        input.x_f_d_O = pos_feet_goal_G;
        input.xdot_f_d_O = vel_feet_goal_G;
        input.xddot_f_d_O.setZero(); // 当前步态接口未输出加速度，沿用论文的零前馈入口。
        for (int leg = 0; leg < 4; ++leg)
        {
            input.q_j.segment<3>(3 * leg) = robot_model_->current_joint_pos_[leg].data;
            input.qdot.segment<3>(6 + 3 * leg) = robot_model_->current_joint_vel_[leg].data;
            input.contact[leg] = wave_generator_->contact_(leg);
        }
        const auto model_wbc_begin = quadruped_debug::Csv_DebugMode ?
            std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
        if (quadruped_debug::Csv_DebugMode) debug_frame_.wbc.flags |= TrottingDebug::WBC_ATTEMPT;
        try
        {
            // 仅修正WBC输入副本的微小正交误差，不修改估计器、主控制和MPC姿态。
            // 明显错误仍回退；归一化后同步欧拉角和B系线速度，供模型与任务共同使用。
            if (!input.R_OB.allFinite() ||
                (input.R_OB.transpose() * input.R_OB - Mat3::Identity()).norm() > 1e-5 ||
                std::abs(input.R_OB.determinant() - 1.0) > 1e-5)
                throw std::invalid_argument("WBC输入姿态不是有效旋转，误差超过微小修正范围");
            Eigen::Quaterniond orientation(input.R_OB);
            orientation.normalize();
            input.R_OB = orientation.toRotationMatrix();
            input.Theta = rotMatToRPY(input.R_OB);
            input.qdot.segment<3>(3) = input.R_OB.transpose() * vel_body_;
            robot_model_->wbc_model_->update(input, relaxation_input_.M, relaxation_input_.C);
            wbc_->updateInput(input);
            wbc_output_ = wbc_->solve();
        }
        catch (const std::exception& error)
        {
            wbc_output_.message = error.what();
        }
        if (quadruped_debug::Csv_DebugMode)
        {
            auto& sample = debug_frame_.wbc;
            sample.model_wbc_ms = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - model_wbc_begin).count();
            if (wbc_output_.success)
            {
                sample.flags |= TrottingDebug::WBC_OK;
                sample.qddot = wbc_output_.qddot_d.cast<float>();
                for (int task = 0; task < 4; ++task)
                {
                    const auto& diagnostic = wbc_output_.tasks[task];
                    sample.rank(task) = diagnostic.projected_rank;
                    sample.position_residual(task) = static_cast<float>(diagnostic.position_residual);
                    sample.velocity_residual(task) = static_cast<float>(diagnostic.velocity_residual);
                    sample.acceleration_residual(task) = static_cast<float>(diagnostic.acceleration_residual);
                }
            }
        }
        if (!wbc_output_.success)
        {
            static rclcpp::Clock wbc_clock(RCL_STEADY_TIME);
            RCLCPP_WARN_THROTTLE(ctrl_interfaces_.node->get_logger(), wbc_clock, 10000,
                "[WBC] 本帧沿用原控制输出：%s", wbc_output_.message.c_str());
        }
    }

}

/**
 * @brief 设置关节MIT kp kd增益
 */
void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) 
    {
        // 大腿和小腿
        for (int j = 1; j < 3; j++) 
        {
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
