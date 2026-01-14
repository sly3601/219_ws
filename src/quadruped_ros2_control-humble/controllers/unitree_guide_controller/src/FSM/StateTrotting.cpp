//
// Created by tlab-uav on 24-9-18.
//

// 包含Trotting状态类的头文件
#include "unitree_guide_controller/FSM/StateTrotting.h"

// 包含数学工具库（提供归一化、饱和函数等）
#include <unitree_guide_controller/common/mathTools.h>
// 包含控制组件头文件（提供机器人模型、平衡控制器等）
#include <unitree_guide_controller/control/CtrlComponent.h>
// 包含状态估计器头文件（提供机器人位置、姿态、足端状态等估计）
#include <unitree_guide_controller/control/Estimator.h>
// 包含步态波发生器头文件（管理支撑相/摆动相状态）
#include <unitree_guide_controller/gait/WaveGenerator.h>

/**
 * @brief Trotting状态类构造函数
 * @param ctrl_interfaces 控制接口（包含指令输入、关节控制接口等）
 * @param ctrl_component 控制组件（包含估计器、机器人模型等核心模块）
 */
StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces,
                             CtrlComponent &ctrl_component) : 
    // 初始化父类FSMState，指定状态名称为TROTTING，状态标识字符串为"trotting"，传入控制接口
    FSMState(FSMStateName::TROTTING, "trotting", ctrl_interfaces),
    // 初始化成员变量：状态估计器（从控制组件中获取）
    estimator_(ctrl_component.estimator_),
    // 初始化成员变量：机器人模型（从控制组件中获取，用于正逆运动学、力矩计算）
    robot_model_(ctrl_component.robot_model_),
    // 初始化成员变量：平衡控制器（从控制组件中获取，用于计算足端支撑力）
    balance_ctrl_(ctrl_component.balance_ctrl_),
    // 初始化成员变量：步态波发生器（从控制组件中获取，管理四足支撑/摆动状态）
    wave_generator_(ctrl_component.wave_generator_),
    // 初始化成员变量：步态生成器（从控制组件中初始化，用于生成足端目标轨迹）
    gait_generator_(ctrl_component) {
    // 提高摆动高度：避免足端落地过浅，增强整体支撑余量（适配1.5倍腿长）
    gait_height_ = 0.08;
    // 身体位置比例增益：大幅提高z轴抑制下沉，x/y提高增强平动控制（全局优化，无后腿单独补偿）
    Kpp = Vec3(130, 130, 180).asDiagonal();
    // 身体速度阻尼增益：增强z轴阻尼抗抖动，x/y提高抑制大惯性超调
    Kdp = Vec3(22, 22, 30).asDiagonal();
    // 姿态比例增益：大幅提高（作用于roll/pitch/yaw），增强整体姿态稳定性，防止侧倒/前后趴
    kp_w_ = 1900;
    // 姿态角速度阻尼增益：重点提高roll/pitch对应轴（x/y），加快姿态收敛，避免倾斜加剧
    Kd_w_ = Vec3(180, 180, 150).asDiagonal();
    // 摆动相位置增益：提高跟踪精度，确保足端精准落地，提供有效支撑
    Kp_swing_ = Vec3(650, 650, 650).asDiagonal();
    // 摆动相速度阻尼：提高避免摆动过快，减少落地冲击导致的支撑失效
    Kd_swing_ = Vec3(30, 30, 30).asDiagonal();

    // 降低速度限制：减少大重量机器人的运动惯性，降低失衡风险
    v_x_limit_ << -0.2, 0.2;    // 进一步降低前后速度
    v_y_limit_ << -0.1, 0.1;    // 大幅降低左右速度，避免侧倒
    w_yaw_limit_ << -0.2, 0.2;  // 降低转向速度，减少姿态扰动
    // 计算控制周期dt（1/控制频率，由控制接口传入）
    dt_ = 1.0 / ctrl_interfaces_.frequency_;
}

/**
 * @brief 进入Trotting状态时的初始化操作
 */
void StateTrotting::enter() {
    // 初始化期望身体位置pcd_为当前估计的身体位置
    pcd_ = estimator_->getPosition();
    // 修正期望身体z坐标：取所有足端z值的平均值，保证初始身体高度均衡（无后腿单独补偿）
    Eigen::Vector3d foot_z_avg = (estimator_->getFeetPos2Body().col(0) + estimator_->getFeetPos2Body().col(1) +
                                  estimator_->getFeetPos2Body().col(2) + estimator_->getFeetPos2Body().col(3)) / 4;
    pcd_(2) = -foot_z_avg(2) + 0.03;  // 整体抬高3cm，预留支撑余量
    // 初始化身体坐标系下的速度指令v_cmd_body_为零向量
    v_cmd_body_.setZero();
    // 初始化期望偏航角yaw_cmd_为当前估计的机器人偏航角
    yaw_cmd_ = estimator_->getYaw();
    // 初始化期望旋转矩阵Rd为绕z轴（yaw轴）旋转yaw_cmd_的矩阵
    Rd = rotz(yaw_cmd_);
    // 初始化全局坐标系下的角速度指令w_cmd_global_为零向量
    w_cmd_global_.setZero();

    // 将控制输入指令重置为0（避免残留指令影响）
    ctrl_interfaces_.control_inputs_.command = 0;
    // 重启步态生成器（重置步态相位，确保步态从初始状态开始）
    gait_generator_.restart();
}

/**
 * @brief Trotting状态的主运行逻辑（每个控制周期执行一次）
 * @param time 当前时间（未使用）
 * @param period 时间间隔（未使用）
 */
void StateTrotting::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    // 获取当前估计的身体位置（全局坐标系）
    pos_body_ = estimator_->getPosition();
    // 获取当前估计的身体速度（全局坐标系）
    vel_body_ = estimator_->getVelocity();

    // 获取当前身体坐标系到全局坐标系的旋转矩阵B2G_RotMat
    B2G_RotMat = estimator_->getRotation();
    // 计算全局坐标系到身体坐标系的旋转矩阵G2B_RotMat（为B2G_RotMat的转置，正交矩阵性质）
    G2B_RotMat = B2G_RotMat.transpose();

    // 解析用户输入指令（来自遥控器/上位机） 
    getUserCmd(); // 得到本体系下的 期望线速度 和 期望角速度
    // 计算全局坐标系下的期望控制指令（位置、速度、姿态）
    calcCmd();   // 并通过全局坐标系下的速度和角速度，计算出期望预期身体位置pcd_

    // 给步态生成器设置输入：全局坐标系下的目标平面速度（x/y）、（既是全局系又是本体系的）目标yaw角速度、足端摆动高度
    gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_global_(2), gait_height_);
    // 生成四足的足端目标位置（pos_feet_global_goal_）和目标速度（vel_feet_global_goal_，全局坐标系）
    gait_generator_.generate(pos_feet_global_goal_, vel_feet_global_goal_);

    // 计算关节力矩指令
    calcTau();
    // 计算关节位置和速度指令
    calcQQd();

    // 判断是否需要迈步：满足条件则切换为全步态波模式，否则切换为全支撑模式
    if (checkStepOrNot()) {
        wave_generator_->status_ = WaveStatus::WAVE_ALL;
    } else {
        wave_generator_->status_ = WaveStatus::STANCE_ALL;
    }

    // 计算并设置关节PID增益（支撑相/摆动相使用不同增益）
    calcGain();
}

/**
 * @brief 退出Trotting状态时的收尾操作
 */
void StateTrotting::exit() {
    // 将步态波发生器状态设置为全摆动模式（确保退出时四足抬起，避免卡顿）
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

/**
 * @brief 检查状态切换条件（判断是否需要从Trotting切换到其他状态）
 * @return 目标FSM状态名称
 */
FSMStateName StateTrotting::checkChange() {
    // 根据控制输入指令判断状态切换
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            // 指令1：切换到被动状态（PASSIVE，机器人关节解锁，随外力运动）
            return FSMStateName::PASSIVE;
        case 2:
            // 指令2：切换到固定站立状态（FIXEDSTAND，机器人保持站立姿态）
            return FSMStateName::FIXEDSTAND;
        default:
            // 默认：保持Trotting状态
            return FSMStateName::TROTTING;
    }
}

/**
 * @brief 解析用户输入指令（将遥控器摇杆值转换为机器人速度指令）
 */
void StateTrotting::getUserCmd() {
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
    /* 平移指令：身体坐标系转全局坐标系 */
    // 将身体坐标系下的速度指令v_cmd_body_通过旋转矩阵转换为全局坐标系下的目标速度vel_target_
    vel_target_ = B2G_RotMat * v_cmd_body_;

    // 进一步缩小速度饱和范围，减少大惯性下的姿态突变
    vel_target_(0) = saturation(vel_target_(0), Vec2(vel_body_(0) - 0.08, vel_body_(0) + 0.08));
    vel_target_(1) = saturation(vel_target_(1), Vec2(vel_body_(1) - 0.08, vel_body_(1) + 0.08));

    // 更新期望身体x/y位置：缩小调节范围，增强稳定性
    pcd_(0) = saturation(pcd_(0) + vel_target_(0) * dt_, Vec2(pos_body_(0) - 0.05, pos_body_(0) + 0.05));
    pcd_(1) = saturation(pcd_(1) + vel_target_(1) * dt_, Vec2(pos_body_(1) - 0.05, pos_body_(1) + 0.05));
    // 显式更新z轴期望位置：适配Kpp(z)的高增益，有效抑制身体下沉
    pcd_(2) = saturation(pcd_(2) + vel_target_(2) * dt_, Vec2(pos_body_(2) - 0.1, pos_body_(2) + 0.1));

    // 全局坐标系z方向目标速度设为0（Trotting步态保持身体高度稳定）
    vel_target_(2) = 0;

    /* 旋转指令：更新期望偏航角和角速度 */
    // 积分yaw轴角速度指令，得到期望偏航角（当前期望角+角速度×控制周期）
    yaw_cmd_ = yaw_cmd_ + d_yaw_cmd_ * dt_;
    // 更新期望旋转矩阵Rd为绕z轴旋转当前期望偏航角的矩阵
    Rd = rotz(yaw_cmd_);
    // 全局坐标系下的yaw轴角速度指令设为滤波后的d_yaw_cmd_
    w_cmd_global_(2) = d_yaw_cmd_;
}

/**
 * @brief 计算关节力矩指令（通过平衡控制和足端轨迹跟踪实现）
 */
void StateTrotting::calcTau() {
    // 计算身体位置误差：期望位置pcd_ - 实际位置pos_body_
    pos_error_ = pcd_ - pos_body_;
    // 计算身体速度误差：目标速度vel_target_ - 实际速度vel_body_
    vel_error_ = vel_target_ - vel_body_;

    // 计算期望身体加速度dd_pcd：位置误差×比例增益 + 速度误差×阻尼增益
    Vec3 dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
    // 计算期望身体角速度增量d_wbd：姿态误差×比例增益 + 角速度误差×阻尼增益
    // rotMatToExp(Rd * G2B_RotMat)：将期望姿态与实际姿态的偏差转换为欧拉角误差
    Vec3 d_wbd = kp_w_ * rotMatToExp(Rd * G2B_RotMat) + Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());

    // 调整加速度限制：缩小x/y范围减少前后左右失衡，放宽z轴增强垂直支撑力
    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-1.5, 1.5));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-1.5, 1.5));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-10, 10));

    // 调整角速度增量限制：缩小roll/pitch对应轴（x/y）范围，防止侧倒/前后趴
    d_wbd(0) = saturation(d_wbd(0), Vec2(-20, 20));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-20, 20));
    d_wbd(2) = saturation(d_wbd(2), Vec2(-8, 8));

    // 获取当前足端相对于身体的位置（全局坐标系，4个足端，3维坐标）
    const Vec34 pos_feet_body_global = estimator_->getFeetPos2Body();
    // 通过平衡控制器计算足端支撑力（全局坐标系）：依赖高增益参数实现均匀支撑，无后腿单独补偿
    Vec34 force_feet_global = -balance_ctrl_->calF(dd_pcd, d_wbd, B2G_RotMat, pos_feet_body_global, wave_generator_->contact_);

    // 获取当前足端实际位置（全局坐标系）
    Vec34 pos_feet_global = estimator_->getFeetPos();
    // 获取当前足端实际速度（全局坐标系）
    Vec34 vel_feet_global = estimator_->getFeetVel();

    // 遍历4个足端，修正摆动相足端力（支撑相使用平衡控制的力，摆动相使用轨迹跟踪的力）
    for (int i(0); i < 4; ++i) {
        // wave_generator_->contact_(i)==0 表示第i个足端处于摆动相
        if (wave_generator_->contact_(i) == 0) {
            // 摆动相足端力：位置跟踪误差×比例增益 + 速度跟踪误差×阻尼增益
            force_feet_global.col(i) = Kp_swing_ * (pos_feet_global_goal_.col(i) - pos_feet_global.col(i)) +
                                       Kd_swing_ * (vel_feet_global_goal_.col(i) - vel_feet_global.col(i));
        }
    }

    // 将足端力从全局坐标系转换为身体坐标系
    Vec34 force_feet_body_ = G2B_RotMat * force_feet_global;

    // 获取当前机器人关节位置（4条腿，每条腿3个关节）
    std::vector<KDL::JntArray> current_joints = robot_model_->current_joint_pos_;
    // 遍历4条腿，计算每条腿的关节力矩并赋值给控制接口
    for (int i = 0; i < 4; i++) {
        // 通过机器人模型的逆动力学计算第i条腿的关节力矩（输入足端力、腿索引）
        KDL::JntArray torque = robot_model_->getTorque(force_feet_body_.col(i), i);
        // 将力矩值赋值给关节力矩控制接口（每条腿3个关节，索引为i*3+j）
        for (int j = 0; j < 3; j++) {
            ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
        }
    }
}

/**
 * @brief 计算关节位置和速度指令（通过足端目标轨迹逆运动学求解）
 */
void StateTrotting::calcQQd() {
    // 获取当前足端相对于身体的位置（KDL Frame格式，4条腿）
    const std::vector<KDL::Frame> pos_feet_body = robot_model_->getFeet2BPositions();

    // 定义足端目标位置和速度（身体坐标系，4个足端，3维坐标）
    Vec34 pos_feet_target, vel_feet_target;
    // 遍历4个足端，转换足端目标轨迹到身体坐标系
    for (int i(0); i < 4; ++i) {
        // 足端目标位置（身体坐标系）：全局目标位置 - 身体实际位置，再转换到身体坐标系
        pos_feet_target.col(i) = G2B_RotMat * (pos_feet_global_goal_.col(i) - pos_body_);
        // 足端目标速度（身体坐标系）：全局目标速度 - 身体实际速度，再转换到身体坐标系
        vel_feet_target.col(i) = G2B_RotMat * (vel_feet_global_goal_.col(i) - vel_body_);
    }

    // 通过机器人模型逆运动学，根据足端目标位置求解关节目标位置q_goal（12个关节，4条腿×3个）
    Vec12 q_goal = robot_model_->getQ(pos_feet_target);
    // 通过机器人模型逆运动学，根据当前足端位置和目标速度求解关节目标速度qd_goal（12个关节）
    Vec12 qd_goal = robot_model_->getQd(pos_feet_body, vel_feet_target);
    // 将关节目标位置和速度赋值给控制接口
    for (int i = 0; i < 12; i++) {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }
}

/**
 * @brief 计算并设置关节PID增益（全局提高支撑刚度，无后腿单独补偿）
 */
void StateTrotting::calcGain() const {
    // 遍历4个足端，根据支撑/摆动状态设置对应关节增益
    for (int i(0); i < 4; ++i) {
        // 第i个足端处于摆动相（contact_==0）：提高全局摆动增益，确保落地精准
        if (wave_generator_->contact_(i) == 0) {
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(5.5);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(4.5);
            }
        } else {
            // 第i个足端处于支撑相（contact_==1）：大幅提高全局支撑增益，增强所有腿的支撑刚度
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(2.0);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(2.0);
            }
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