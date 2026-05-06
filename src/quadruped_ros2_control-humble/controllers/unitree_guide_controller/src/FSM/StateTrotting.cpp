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
    gait_generator_(ctrl_component, this){
    // 提高摆动高度：避免足端落地过浅，增强整体支撑余量（适配1.5倍腿长）
    gait_height_ = 0.04;
    // 身体位置比例增益：大幅提高z轴抑制下沉，x/y提高增强平动控制（全局优化，无后腿单独补偿）
    Kpp = Vec3(18, 18, 300.1).asDiagonal();
    // 身体速度阻尼增益：增强z轴阻尼抗抖动，x/y提高抑制大惯性超调
    Kdp = Vec3(2.2, 2.2, 5.0).asDiagonal();
    // 姿态比例增益：大幅提高（作用于roll/pitch/yaw），增强整体姿态稳定性，防止侧倒/前后趴
    kp_pitch_ = 420;    // 1900
    kp_roll_ = 370;    // 1900
    kp_yaw_ = 0.2;     // 1900
        // 姿态角速度阻尼增益：重点提高roll/pitch对应轴（x/y），加快姿态收敛，避免倾斜加剧
    Kd_w_ = Vec3(4.1, 5.1, 3.1).asDiagonal();
    // 摆动相位置增益：提高跟踪精度，确保足端精准落地，提供有效支撑
    Kp_swing_ = Vec3(0.3, 0.3, 0.3).asDiagonal();
    // 摆动相速度阻尼：提高避免摆动过快，减少落地冲击导致的支撑失效
    Kd_swing_ = Vec3(0.1, 0.1, 0.1).asDiagonal();

    // 降低速度限制：减少大重量机器人的运动惯性，降低失衡风险
    v_x_limit_ << -0.2, 0.2;    // 进一步降低前后速度
    v_y_limit_ << -0.1, 0.1;    // 大幅降低左右速度，避免侧倒
    w_yaw_limit_ << -0.2, 0.2;  // 降低转向速度，减少姿态扰动
    // 计算控制周期dt（1/控制频率，由控制接口传入）
    dt_ = 1.0 / ctrl_interfaces_.frequency_;
    //是否使用卡尔曼滤波位姿闭环
    troting_kalman = 2; //0 - 纯开环，1 - 卡尔曼滤波全闭环，2 - 卡尔曼滤波仅roll/pitch闭环

    // 新增：初始化足底可视化发布器
    foot_marker_pub_ = std::make_unique<quadruped_controller::FootMarkerPublisher>(this->ctrl_interfaces_.node);
}



/**
 * @brief 进入Trotting状态时的初始化操作
 */
void StateTrotting::enter() {
    if(troting_kalman == 1)
    {
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
        Rd = rotz(yaw_cmd_); // 期望的躯体姿态的B2G旋转矩阵
    }
    else if(troting_kalman == 0 || troting_kalman == 2)
    {
        // 得到四足当前实时的四个足底末端位置
        auto feet_2b = robot_model_->getFeet2BPositions();
        double foot_z_avg = (feet_2b[0].p.z() + feet_2b[1].p.z() + feet_2b[2].p.z() + feet_2b[3].p.z()) / 4.0;
        v_cmd_body_.setZero();
        
        if(troting_kalman == 2)
        {
            // 原地稳定踏步，世界系位置目标取进入时当前位置
            pcd_ = estimator_->getPosition();
            yaw_cmd_ = estimator_->getYaw(); // 这里非常对，把上电时初始yaw作为闭环目标
            const double roll_des = 0.0;
            const double pitch_des = 0.05; // 注意！pitch理想值不是0
            Rd = rotz(yaw_cmd_) * roty(pitch_des) * rotx(roll_des);
        }
        else
        {
            pcd_ << 0.0, 0.0, -foot_z_avg;
            yaw_cmd_ = 0;
            Rd.setIdentity();// 在开环中，期望姿态按B系与P系重合处理
        }
        wave_generator_->status_ = WaveStatus::STANCE_ALL;   // 先明确给全支撑
    }
    // 初始化G系下的角速度指令w_cmd_global_为零向量
    w_cmd_global_.setZero();
    // 初始化P系下的角速度指令w_cmd_parallel_为零向量
    w_cmd_parallel_.setZero();

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
    if(troting_kalman == 2)
    {
        // 获取当前估计的身体位置（全局坐标系）
        pos_body_ = estimator_->getPosition();
        // 获取当前估计的身体速度（全局坐标系）
        vel_body_ = estimator_->getVelocity();
        // 获取当前身体坐标系到定向身体坐标系的旋转矩阵B2P_RotMat
        // imu数据源内容就是B系到P系的旋转矩阵
        B2P_RotMat = estimator_->getRotation();
        // 计算P系到B系的旋转矩阵P2B_RotMat（为B2P_RotMat的转置，正交矩阵性质）
        P2B_RotMat = B2P_RotMat.transpose();
    }
    else if(troting_kalman == 0)
    {
        // P系 = B系，不考虑任何移动的情况。
        B2P_RotMat.setIdentity(); // 固定B系和P系一致，不受实际姿态影响
        // 计算P系到B系的旋转矩阵P2B_RotMat（为B2P_RotMat的转置，正交矩阵性质）
        P2B_RotMat = B2P_RotMat.transpose();
    }

    if (troting_kalman == 1) 
    {
        pos_body_ = estimator_->getPosition();
        vel_body_ = estimator_->getVelocity();
        B2G_RotMat = estimator_->getRotation();
        G2B_RotMat = B2G_RotMat.transpose(); // 保留世界系G
    } 

    // 解析用户输入指令（来自遥控器/上位机） 
    getUserCmd(); // 得到B系下的 期望线速度 和 期望角速度
    // 计算全局坐标系下的期望控制指令（位置、速度、姿态）
    calcCmd();   // 并通过全局坐标系下的速度和角速度，计算出期望预期身体位置pcd_

    if(troting_kalman == 1)
    {
        // 给步态生成器设置输入：全局坐标系下的目标平面速度（x/y）、（既是全局系又是本体系的）目标yaw角速度、足端摆动高度
        gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_global_(2), gait_height_);
        // 生成四足的足端目标位置（pos_feet_global_goal_）和目标速度（vel_feet_global_goal_，全局坐标系）
        gait_generator_.generate(pos_feet_global_goal_, vel_feet_global_goal_);
    }
    else if(troting_kalman == 2 || troting_kalman == 0)
    {
        // 给步态生成器设置输入：G系下的目标平面速度（x/y），P系下的目标yaw角速度，足端抬起高度
        gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_parallel_(2), gait_height_);
        // 生成四足的足端目标位置（pos_feet_parallel_goal_）和目标速度（vel_feet_parallel_goal_，是G系下的，变量名是错的待改正。
        gait_generator_.generate(pos_feet_parallel_goal_, vel_feet_parallel_goal_);
    }

    // 计算关节力矩指令
    calcTau();
    // 计算关节位置和速度指令
    calcQQd();

    if(troting_kalman == 1)
    {
        // 判断是否需要迈步：满足条件则切换为全步态波模式，否则切换为全支撑模式
        if (checkStepOrNot()) {
            wave_generator_->status_ = WaveStatus::WAVE_ALL;
        } else {
            wave_generator_->status_ = WaveStatus::STANCE_ALL;
        }
    }
    else if(troting_kalman == 0 || troting_kalman == 2)
    {
        // ========== 先切到全支撑，再切到迈步 ==========
        // 加一个静态变量，记录是不是第一次进入开环
        static bool first_time_open_loop = true;
        
        if(first_time_open_loop)
        {
            // 第一次：先强制切到 STANCE_ALL（全支撑），模拟闭环的初始状态
            wave_generator_->status_ = WaveStatus::STANCE_ALL;
            first_time_open_loop = false;
        }
        else
        {
            // 之后：再强制切到 WAVE_ALL（迈步）
            // 因为是从 STANCE_ALL 切过来的，作者的平滑逻辑能处理，不会死锁！
            wave_generator_->status_ = WaveStatus::WAVE_ALL;
        }
    }

    // 计算并设置关节PID增益（支撑相/摆动相使用不同增益）
    calcGain();

    // -------------------------------------------------------------------------
    // 新增：准备4个足底坐标（替换为你实际的足底坐标计算结果）
    // -------------------------------------------------------------------------

    std::array<geometry_msgs::msg::Point, 4> foot_positions;
    foot_positions[0].x = pos_feet_parallel_goal_(0,0);  
    foot_positions[0].y = pos_feet_parallel_goal_(1,0); 
    foot_positions[0].z = pos_feet_parallel_goal_(2,0); // FR
    foot_positions[1].x = pos_feet_parallel_goal_(0,1);  
    foot_positions[1].y = pos_feet_parallel_goal_(1,1); 
    foot_positions[1].z = pos_feet_parallel_goal_(2,1); // FL
    foot_positions[2].x = pos_feet_parallel_goal_(0,2); 
    foot_positions[2].y = pos_feet_parallel_goal_(1,2); 
    foot_positions[2].z = pos_feet_parallel_goal_(2,2); // RR
    foot_positions[3].x = pos_feet_parallel_goal_(0,3); 
    foot_positions[3].y = pos_feet_parallel_goal_(1,3); 
    foot_positions[3].z = pos_feet_parallel_goal_(2,3); // RL
    // 新增：更新并发布足底Marker1
    // 新增：降低发布频率到50Hz (500Hz / 10 = 50Hz)
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
void StateTrotting::getUserCmd() { // 该函数P/B/G坐标系混乱 还没改

    // ========== 新增 ==========
    // 读取原始值
    double ly_raw = ctrl_interfaces_.control_inputs_.ly;
    double lx_raw = ctrl_interfaces_.control_inputs_.lx;
    double rx_raw = ctrl_interfaces_.control_inputs_.rx;

    // ========== 新增：死区处理 (Deadzone) ==========
    const double deadzone = 0.05; // 阈值，小于这个值认为是0
    if (fabs(ly_raw) < deadzone) ly_raw = 0.0;
    if (fabs(lx_raw) < deadzone) lx_raw = 0.0;
    if (fabs(rx_raw) < deadzone) rx_raw = 0.0;
    // ===============================================



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
    // troting_kalman =1代表有卡尔曼滤波闭环
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
    else if(troting_kalman == 0)
    {
        /* 2026.04.04 无卡尔曼的开环步态*/
        // 无卡尔曼滤波的开环troting
        /* 平移指令：完全开环，不做位置积分，也不看实际位置 */
        // 直接把目标速度设为0（先试原地踏步），或者用很小的开环速度
        vel_target_.setZero(); // 强制目标速度为0，原地开环踏步
        vel_target_(0) = 0; // 加一个极小的X方向虚拟速度，强制步态生成器迈步
        vel_target_(2) = 0;

        // 期望位置也直接设为固定值，不用积分
        pcd_(0) = 0.0; // 固定期望X位置
        pcd_(1) = 0.0; // 固定期望Y位置
        // pcd_(2) 保持原来的高度就行，不用改

        /* 旋转指令：依然用IMU的直接姿态，不用改 */
        yaw_cmd_ = 0;
        Rd.setIdentity(); // 期望旋转矩阵固定为单位矩阵（完全水平）
        w_cmd_parallel_.setZero(); // 目标角速度固定为0
        
    }
    else if(troting_kalman == 2)
    {
        // v2.5修改：2模式只做原地稳定踏步，不接受平移/转向遥控
        const double roll_des  = 0.0;
        const double pitch_des = 0.05;

        vel_target_.setZero();          // 目标平移速度 = 0
        w_cmd_parallel_.setZero();      // 目标角速度 = 0

        // pcd_(0/1/2) 和 yaw_cmd_ 都保持 enter() 里初始化的目标，不在这里改
        Rd = rotz(yaw_cmd_) * roty(pitch_des) * rotx(roll_des);
    }
}

/**
 * @brief 计算关节力矩指令（通过平衡控制和足端轨迹跟踪实现）
 */

void StateTrotting::calcTau() {
    // ========== 函数公共变量 统一声明 ==========
    Vec3 dd_pcd; // 期望机身中心线加速度，dd = 二阶导数，也就是加速度。dd_pcd(0)是x轴加速度，dd_pcd(1)是y轴加速度，dd_pcd(2)是z轴加速度
    dd_pcd.setZero();

    Vec3 d_wbd; // 期望机身角运动控制量， d = 一阶导数。wb指机身角速度。d = desired，指期望的
    d_wbd.setZero();

    Vec34 pos_feet_body_global; // 全局坐标系下的足端位置（相对于身体坐标系）
    pos_feet_body_global.setZero();

    Vec34 force_feet_global;    // 全局坐标系下的期望足底反力
    force_feet_global.setZero();

    Vec34 pos_feet_global;      // 全局坐标系下，四个足端当前实际位置
    pos_feet_global.setZero();

    Vec34 vel_feet_global;      // 全局坐标系下，四个足端当前实际速度
    vel_feet_global.setZero();

    Vec34 force_feet_body_;     // 身体坐标系下的期望足底反力
    force_feet_body_.setZero();

    // ========== 新增：仅 troting_kalman == 2 会用到的中间变量，也统一放到判断外 ==========
    std::vector<KDL::Frame> feet_frames_body;
    Vec34 pos_feet_body;
    pos_feet_body.setZero();

    Vec3 rot_err;
    rot_err.setZero();

    Vec3 gyro_global;
    gyro_global.setZero();

    Vec3 zero_w_cmd;
    zero_w_cmd.setZero();

    // debug 用的变量，暂时放在这里
    double calf_tau_raw[4] = {0.0, 0.0, 0.0, 0.0};
    double calf_tau_cmd[4] = {0.0, 0.0, 0.0, 0.0};
    double raw_fz_right = 0.0;
    double raw_fz_left  = 0.0;
    double mx_qp_raw    = 0.0;

    double cmd_fz_right_body = 0.0;
    double cmd_fz_left_body  = 0.0;
    double mx_cmd_body       = 0.0;

    double hip_tau_raw_right = 0.0;   // FR+RR
    double hip_tau_raw_left  = 0.0;   // FL+RL
    double hip_tau_cmd_right = 0.0;
    double hip_tau_cmd_left  = 0.0;


    double roll_err_rpy  = 0;
    double pitch_err_rpy = 0;
    double yaw_err_rpy = 0;

    if(troting_kalman == 1)
    {
        // 计算身体位置误差：期望位置pcd_ - 实际位置pos_body_
        pos_error_ = pcd_ - pos_body_;
        // 计算身体速度误差：目标速度vel_target_ - 实际速度vel_body_
        vel_error_ = vel_target_ - vel_body_;
        // 计算期望身体加速度dd_pcd：位置误差×比例增益 + 速度误差×阻尼增益
        dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
        // 计算期望身体角速度增量d_wbd：姿态误差×比例增益 + 角速度误差×阻尼增益
        // Rd * G2B_RotMat就是误差旋转矩阵。rotMatToExp()函数能：将期望姿态与实际姿态的偏差转换为欧拉角误差
        d_wbd = kp_w_ * rotMatToExp(Rd * G2B_RotMat) + Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());

        // 调整加速度限制：缩小x/y范围减少前后左右失衡，放宽z轴增强垂直支撑力
        dd_pcd(0) = saturation(dd_pcd(0), Vec2(-1.5, 1.5));
        dd_pcd(1) = saturation(dd_pcd(1), Vec2(-1.5, 1.5));
        dd_pcd(2) = saturation(dd_pcd(2), Vec2(-20, 20));

        // 调整角速度增量限制：缩小roll/pitch对应轴（x/y）范围，防止侧倒/前后趴
        d_wbd(0) = saturation(d_wbd(0), Vec2(-25, 25));
        d_wbd(1) = saturation(d_wbd(1), Vec2(-30, 30));
        d_wbd(2) = saturation(d_wbd(2), Vec2(-8, 8));

        // 获取当前足端相对于身体的位置（全局坐标系，4个足端，3维坐标）
        pos_feet_body_global = estimator_->getFeetPos2Body();
        // 通过平衡控制器计算足端支撑力（全局坐标系）：依赖高增益参数实现均匀支撑，无后腿单独补偿
        force_feet_global = -balance_ctrl_->calF(dd_pcd, d_wbd, B2G_RotMat, pos_feet_body_global, wave_generator_->contact_);

        // 获取当前足端实际位置（全局坐标系）
        pos_feet_global = estimator_->getFeetPos();
        // 获取当前足端实际速度（全局坐标系）
        vel_feet_global = estimator_->getFeetVel();

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
        force_feet_body_ = G2B_RotMat * force_feet_global;

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

    // ========== troting_kalman == 2 基础支撑腿力分配/摆动腿轨迹闭环 ==========
    // 整段逻辑： 不做机身位置闭环，只利用当前接触腿，分配一组足底支撑力，让机器人一边托住自身重力，一边把 roll / pitch 姿态往期望方向拉回去。
    /* 
        姿态误差 / 角速度
            ↓
        机身想要的总力矩（以及重力支撑）
            ↓
        各支撑腿应该承担多少足底反力
            ↓
        每条腿对应的关节力矩
            ↓
        写入电机关节 torque
    */
    else if(troting_kalman == 2)
    {
        // 加入 x/y/z 平面闭环
        pos_error_ = pcd_ - pos_body_;
        vel_error_ = vel_target_ - vel_body_;

        dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;

        dd_pcd(0) = saturation(dd_pcd(0), Vec2(-3.2, 3.2));
        dd_pcd(1) = saturation(dd_pcd(1), Vec2(-3.2, 3.2));
        dd_pcd(2) = saturation(dd_pcd(2), Vec2(-10.5, 10.5));

        // rot_err = rotMatToExp(Rd * P2B_RotMat);
        // gyro_global = estimator_->getGyroGlobal();


        // // 角度误差乘以比例增益 + 角速度乘以阻尼增益，得到期望的机身角运动控制量（力矩）
        // // d_wbd = kp_w_ * rot_err + Kd_w_ * (w_cmd_parallel_ - gyro_global);
        // d_wbd(0) =  kp_w_ * rot_err(0) + Kd_w_(0,0) * (w_cmd_parallel_(0) - gyro_global(0));
        // d_wbd(1) =  kp_w_ * rot_err(1) + Kd_w_(1,1) * (w_cmd_parallel_(1) - gyro_global(1));
        // d_wbd(2) =  kp_w_ * rot_err(2) + Kd_w_(2,2) * (w_cmd_parallel_(2) - gyro_global(2));


        // 1) 用完整旋转矩阵计算姿态误差，不再手写 roll_des - roll / pitch_des - pitch
        rot_err = rotMatToExp(Rd * P2B_RotMat);
        // 2) 角速度仍然使用 estimator 给出的全局角速度
        gyro_global = estimator_->getGyroGlobal();
        // 3) debug 变量沿用原来的名字，但现在含义变成 rotMatToExp 的三轴误差
        roll_err_rpy  = rot_err(0);
        pitch_err_rpy = rot_err(1);
        yaw_err_rpy   = rot_err(2);
        // 4) 三轴分开的 kp / kd
        // 注意：这里不要再给 roll / pitch 外面额外加负号
        d_wbd(0) = kp_roll_  * rot_err(0) + Kd_w_(0,0) * (0.0 - gyro_global(0));
        d_wbd(1) = kp_pitch_ * rot_err(1) + Kd_w_(1,1) * (0.0 - gyro_global(1));
        d_wbd(2) = kp_yaw_   * rot_err(2) + Kd_w_(2,2) * (0.0 - gyro_global(2));


        // ========== 限制 roll， pitch，yaw ==========
        d_wbd(0) = saturation(d_wbd(0), Vec2(-40, 40));
        d_wbd(1) = saturation(d_wbd(1), Vec2(-50, 50));
        d_wbd(2) = saturation(d_wbd(2), Vec2(-26, 26));

        // 当前足端相对于身体的位置，直接用机器人模型正运动学，不依赖位置/速度估计
        feet_frames_body = robot_model_->getFeet2BPositions();
        for (int i = 0; i < 4; ++i) {
            pos_feet_body.col(i) = Vec3(feet_frames_body[i].p.data);
        }

        // 得到P系下的足端位置
        pos_feet_body_global = B2P_RotMat * pos_feet_body;

        // 通过平衡控制器计算足端支撑力（全局坐标系）
        // dd_pcd，期望机身加速度，原地踏步不需要，所以全设为0
        // d_wbd，期望机身角运动控制量（期望机身力矩），roll/pitch/yaw的姿态误差和角速度阻尼
        // B2P_RotMat，B系到P系的旋转矩阵
        // pos_feet_body_global，P系下的足端位置（相对于B系）
        // 输出force_feet_global，全局坐标系下的期望足底反力
        // force_feet_global = -balance_ctrl_->calF(dd_pcd, d_wbd, B2P_RotMat, pos_feet_body_global, wave_generator_->contact_);
        if (!B2P_RotMat.allFinite() || !pos_feet_body_global.allFinite()) 
        {
            for (int k = 0; k < 12; ++k)
                ctrl_interfaces_.joint_torque_command_interface_[k].get().set_value(0.0);
            return;
        }
        try 
        {
            // calF函数内部计算出来的是G系下的地面对机身的反作用力，我们需要的是足端对地面的力，所以加负号取反
            force_feet_global = -balance_ctrl_->calF(dd_pcd, d_wbd, B2P_RotMat, pos_feet_body_global, wave_generator_->contact_); 
        }
        catch (...) 
        {
            for (int k = 0; k < 12; ++k) 
                ctrl_interfaces_.joint_torque_command_interface_[k].get().set_value(0.0); 
            return; 
        }

        raw_fz_right = -(force_feet_global(2, 0) + force_feet_global(2, 2)); // FR + RR
        raw_fz_left  = -(force_feet_global(2, 1) + force_feet_global(2, 3)); // FL + RL

        for (int i = 0; i < 4; ++i) {
            mx_qp_raw += pos_feet_body_global(1, i) * (-force_feet_global(2, i))
                    - pos_feet_body_global(2, i) * (-force_feet_global(1, i));
        }

        // 当前四个足端在全局系下的实际位置
        pos_feet_global = estimator_->getFeetPos();

        // 当前四个足端在全局系下的实际速度
        vel_feet_global = estimator_->getFeetVel();

        const double swing_force_limit_xy = 5.0;
        const double swing_force_limit_z  = 10.0;

        for (int i = 0; i < 4; ++i)
        {
            // contact == 0 表示摆动腿
            if (wave_generator_->contact_(i) == 0)
            {
                Vec3 swing_force =
                    Kp_swing_ * (pos_feet_parallel_goal_.col(i) - pos_feet_global.col(i)) +
                    Kd_swing_ * (vel_feet_parallel_goal_.col(i) - vel_feet_global.col(i));

                swing_force(0) = saturation(swing_force(0), Vec2(-swing_force_limit_xy, swing_force_limit_xy));
                swing_force(1) = saturation(swing_force(1), Vec2(-swing_force_limit_xy, swing_force_limit_xy));
                swing_force(2) = saturation(swing_force(2), Vec2(-swing_force_limit_z,  swing_force_limit_z));

                force_feet_global.col(i) = swing_force;
            }
        }


        // 将足端力从P系转换为B系
        force_feet_body_ = P2B_RotMat * force_feet_global;

        // debug经过 -calF 和 P2B 之后，命令到 body 里的总滚转力矩方向还对不对。
        cmd_fz_right_body = force_feet_body_(2, 0) + force_feet_body_(2, 2); // FR + RR
        cmd_fz_left_body  = force_feet_body_(2, 1) + force_feet_body_(2, 3); // FL + RL

        for (int i = 0; i < 4; ++i) {
            mx_cmd_body += pos_feet_body(1, i) * force_feet_body_(2, i)
                        - pos_feet_body(2, i) * force_feet_body_(1, i);
        }

        // ========== 新增：MIT模式下，这里更适合作为前馈/偏置力矩，而不是直接满量目标力矩 ==========
        // 足底反力没有精确控制，而是变成偏置力矩*小于1的比例系数进行修正控制
        const double tau_ff_scale = 0.93;   // 衰减系数 先从0.25开始，后面可再调大
        const double tau_ff_limit_hip = 10.0;
        const double tau_ff_limit_thigh = 82.0;
        const double tau_ff_limit_calf = 100.0;

        // 遍历4条腿，计算每条腿的关节力矩并赋值给控制接口
        for (int i = 0; i < 4; i++) {
            KDL::JntArray torque = robot_model_->getTorque(force_feet_body_.col(i), i);
            for (int j = 0; j < 3; j++) {
                if (j == 0) { // hip raw/cmd
                    if (i == 0 || i == 2) hip_tau_raw_right += torque(j);
                    else                   hip_tau_raw_left  += torque(j);
                }
                if (j == 2) // debug小腿偏置力矩
                {
                    calf_tau_raw[i] = torque(j);   // 小腿原始力矩
                }
                // ========== 新增：先缩放，再按关节类型限幅 ==========
                double tau_cmd = tau_ff_scale * torque(j);

                if (j == 0) {
                    tau_cmd = saturation(tau_cmd, Vec2(-tau_ff_limit_hip, tau_ff_limit_hip));
                } else if (j == 1) {
                    tau_cmd = saturation(tau_cmd, Vec2(-tau_ff_limit_thigh, tau_ff_limit_thigh));
                } else {
                    tau_cmd = saturation(tau_cmd, Vec2(-tau_ff_limit_calf, tau_ff_limit_calf));
                    calf_tau_cmd[i] = tau_cmd;     // debug小腿最终命令力矩
                }

                if (j == 0) {
                    if (i == 0 || i == 2) hip_tau_cmd_right += tau_cmd;
                    else                   hip_tau_cmd_left  += tau_cmd;
                }

                ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(tau_cmd);
            }
        }

    }

    else if(troting_kalman == 0)
    {
        /* 开环troting不计算力矩，直接位置控制 */
        for (int i = 0; i < 12; i++) {
            ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
        }
    }

    if (ctrl_interfaces_.debug_pub) { // 检查指针是否存在
        std_msgs::msg::Float64MultiArray msg;

        // 0: 目标高度 pcd_z
        msg.data.push_back(pcd_(2));

        // 1: 高度误差 pos_error_z
        msg.data.push_back(pos_error_(2));

        // 2: 高度速度误差 vel_error_z
        msg.data.push_back(vel_error_(2));

        // 3-5: xyz方向期望控制输出机身加速度 dd_pcd
        msg.data.push_back(dd_pcd(0));
        msg.data.push_back(dd_pcd(1));
        msg.data.push_back(dd_pcd(2));

        // 6-9: 四条腿 Fz FR FL RR RL
        msg.data.push_back(force_feet_global(2, 0));
        msg.data.push_back(force_feet_global(2, 1));
        msg.data.push_back(force_feet_global(2, 2));
        msg.data.push_back(force_feet_global(2, 3));

        // 10-13: 四条腿小腿原始力矩 FR FL RR RL
        msg.data.push_back(calf_tau_raw[0]);
        msg.data.push_back(calf_tau_raw[1]);
        msg.data.push_back(calf_tau_raw[2]);
        msg.data.push_back(calf_tau_raw[3]);

        // 14-17: 四条腿小腿最终命令力矩 FR FL RR RL
        msg.data.push_back(calf_tau_cmd[0]);
        msg.data.push_back(calf_tau_cmd[1]);
        msg.data.push_back(calf_tau_cmd[2]);
        msg.data.push_back(calf_tau_cmd[3]);

        // 18: roll_err，19: pitch_err，20: yaw_err
        msg.data.push_back(roll_err_rpy);
        msg.data.push_back(pitch_err_rpy);
        msg.data.push_back(yaw_err_rpy);

        msg.data.push_back(d_wbd(0));     // 21: roll方向目标力矩项
        msg.data.push_back(d_wbd(1));     // 22: pitch方向目标力矩项
        msg.data.push_back(d_wbd(2));     // 23: yaw方向目标力矩项

        msg.data.push_back(raw_fz_right); // 24: calF原始输出右侧总Fz
        msg.data.push_back(raw_fz_left);  // 25: calF原始输出左侧总Fz
        msg.data.push_back(mx_qp_raw);    // 26: calF原始输出总Mx

        msg.data.push_back(cmd_fz_right_body); // 27: 经过 -calF、转到body系、并清零摆动腿后，右侧(FR+RR)总Fz
        msg.data.push_back(cmd_fz_left_body);  // 28: 经过 -calF、转到body系、并清零摆动腿后，左侧(FL+RL)总Fz
        msg.data.push_back(mx_cmd_body);       // 29: 经过 -calF、转到body系、并清零摆动腿后，由当前命令足底力产生的总滚转力矩Mx（body系）

        msg.data.push_back(hip_tau_raw_right); // 30: 右侧hip原始力矩和(FR+RR)，来自 getTorque()，尚未乘 tau_ff_scale/尚未限幅
        msg.data.push_back(hip_tau_raw_left);  // 31: 左侧hip原始力矩和(FL+RL)，来自 getTorque()，尚未乘 tau_ff_scale/尚未限幅
        msg.data.push_back(hip_tau_cmd_right); // 32: 右侧hip最终命令力矩和(FR+RR)，已经过 tau_ff_scale 和限幅
        msg.data.push_back(hip_tau_cmd_left);  // 33: 左侧hip最终命令力矩和(FL+RL)，已经过 tau_ff_scale 和限幅


        msg.data.push_back(dd_pcd(0));  // 34: x方向加速度控制输出
        msg.data.push_back(dd_pcd(1)); // 35: y方向加速度控制输出
        msg.data.push_back(dd_pcd(2));  // 36: z方向加速度控制输出

        msg.data.push_back(gyro_global(0));  // 37: x方向加速度控制输出
        msg.data.push_back(gyro_global(1)); // 38: y方向加速度控制输出
        msg.data.push_back(gyro_global(2));  // 39: z方向加速度控制输出
        ctrl_interfaces_.debug_pub->publish(msg);
    }
}

/**
 * @brief 计算关节位置和速度指令（通过足端目标轨迹逆运动学求解）
 * 这函数只管目标足底位置的逆解，别的不管
 */
void StateTrotting::calcQQd() {
    Vec3 pos_body_to_use;
    Vec3 vel_body_to_use;
    if(troting_kalman == 1)
    {
        pos_body_to_use = pos_body_;
        vel_body_to_use = vel_body_;
    }
    else if(troting_kalman == 0 || troting_kalman == 2)
    {
        pos_body_to_use = pcd_; // 开环时用固定的期望位置
        vel_body_to_use = vel_target_; // 开环时用固定的期望速度
    }
    // 获取当前足端在B系下的位置（KDL Frame格式，4条腿）
    const std::vector<KDL::Frame> pos_feet_body = robot_model_->getFeet2BPositions();

    // 定义足端目标位置和速度（身体坐标系，4个足端，3维坐标）
    Vec34 pos_feet_target, vel_feet_target;
    if(troting_kalman == 1)
    {
        // 遍历4个足端，转换足端目标轨迹到身体坐标系
        for (int i(0); i < 4; ++i) {
            // 足端目标位置（身体坐标系）：全局目标位置 - 身体实际位置，再转换到身体坐标系
            pos_feet_target.col(i) = G2B_RotMat * (pos_feet_global_goal_.col(i) - pos_body_to_use);
            // 足端目标速度（身体坐标系）：全局目标速度 - 身体实际速度，再转换到身体坐标系
            vel_feet_target.col(i) = G2B_RotMat * (vel_feet_global_goal_.col(i) - vel_body_to_use);
        }
    }
    else if(troting_kalman == 0)
    {
        // 开环0：GaitGenerator 输出本来就是B系，直接用
        pos_feet_target = pos_feet_parallel_goal_;
        vel_feet_target = vel_feet_parallel_goal_;
    }
    else if(troting_kalman == 2)
    {
        // GaitGenerator 内部现在存的是世界系脚点，这里统一转回B系再做IK
        for (int i = 0; i < 4; ++i) {
            pos_feet_target.col(i) = P2B_RotMat * (pos_feet_parallel_goal_.col(i) - pos_body_);
            vel_feet_target.col(i) = P2B_RotMat * (vel_feet_parallel_goal_.col(i) - vel_body_);
        }
    }

    // 通过机器人模型逆运动学，根据足端目标位置求解关节目标位置q_goal（12个关节，4条腿×3个）
    Vec12 q_goal;
    Vec12 qd_goal;
    q_goal.setZero();
    qd_goal.setZero();

    // v2.3修改：仅roll/pitch闭环框架，只修正支撑腿目标足端z
    // 闭环思路： 机身姿态误差 → 支撑腿脚高修正（pos_feet_target）
    if (troting_kalman == 2)
    {
        // roll 和 pitch 的 PD 增益
        const double kp_roll = 0.70;
        const double kd_roll = 0.07;
        const double kp_pitch = 0.60;
        const double kd_pitch = 0.04;

        const double k_roll_to_z = 0.1; // roll 方向的纠正量，映射到支撑腿 z 修正时，放大/缩小多少
        const double k_pitch_to_z = 0.08;// pitch 方向的纠正量，映射到支撑腿 z 修正时，放大/缩小多少
        const double dz_limit = 0;   // roll/pitch闭环已暂时禁用！单条腿这次最多只允许修正这么高，防止闭环一上来把腿拉太狠。

        Vec3 attitude_error = rotMatToExp(Rd * P2B_RotMat); // Rd 期望姿态旋转矩阵，是单位阵，也就是“希望机身保持水平”
        Vec3 gyro = estimator_->getGyroGlobal(); // 预期：gyro(0)：roll 方向角速度 gyro(1)：pitch 方向角速度

        double roll_cmd = kp_roll * attitude_error(0) + kd_roll * (0.0 - gyro(0));
        double pitch_cmd = kp_pitch * attitude_error(1) + kd_pitch * (0.0 - gyro(1));

        for (int i = 0; i < 4; ++i)
        {
            if (wave_generator_->contact_(i) == 1)
            {
                bool is_front = (i == 0 || i == 1);  // FR / FL
                bool is_rear  = (i == 2 || i == 3);  // RR / RL
                bool is_left  = (i == 1 || i == 3);  // FL / RL
                bool is_right = (i == 0 || i == 2);  // FR / RR

                double roll_correction = 0.0;
                double pitch_correction = 0.0;

                // v2.3修改：roll 纠正左右倾斜
                // 如果方向反了，只交换左右两边的正负号
                if (is_left) {
                    roll_correction = -k_roll_to_z * roll_cmd;
                } else if (is_right) {
                    roll_correction = +k_roll_to_z * roll_cmd;
                }

                // v2.3修改：pitch 纠正前后俯仰
                // 如果方向反了，只交换前后两边的正负号
                if (is_front) {
                    pitch_correction = -k_pitch_to_z * pitch_cmd;
                } else if (is_rear) {
                    pitch_correction = +k_pitch_to_z * pitch_cmd;
                }

                double dz_correction = roll_correction + pitch_correction;
                dz_correction = saturation(dz_correction, Vec2(-dz_limit, dz_limit));
                debug_z_[i] = dz_correction; // 用于调试，发布到ROS2话题
                pos_feet_target(2, i) += dz_correction;
            }
        }
    }


    // =====================================================
    // 1) 统一恢复为正常三轴逆解：不再区分闭环/开环使用不同IK
    //    开环和闭环都统一走 robot_model_->getQ(pos_feet_target)
    // =====================================================
    q_goal = robot_model_->getQ(pos_feet_target);

    std::vector<KDL::Frame> pos_feet_target_frame(4);
    for (int i = 0; i < 4; ++i) {
        pos_feet_target_frame[i].p = KDL::Vector(
            pos_feet_target(0, i),
            pos_feet_target(1, i),
            pos_feet_target(2, i)
        );
        pos_feet_target_frame[i].M = KDL::Rotation::Identity();
    }
    qd_goal = robot_model_->getQd(pos_feet_target_frame, vel_feet_target);

    // =====================================================
    // 2) 关节命令最终限幅（重点限制髋关节）
    //    这是“命令层限幅”，为了安全应急的，但足够适合作为当前版本
    // =====================================================

    // ---------- 髋关节（第0关节）小范围限制 ----------
    const double hip_center = 0.0;    // 希望髋关节稳定在0附近
    const double hip_range  = 0.08;   // 先用 ±0.08 rad（约 ±4.6°）

    const double hip_min = hip_center - hip_range;
    const double hip_max = hip_center + hip_range;



    // ---------- 逐条腿做髋关节限幅 ----------
    for (int leg_idx = 0; leg_idx < 4; ++leg_idx) 
    {
        const int hip_idx   = leg_idx * 3 + 0;
        const int thigh_idx = leg_idx * 3 + 1;
        const int calf_idx  = leg_idx * 3 + 2;

        // // 位置限幅
        q_goal(hip_idx)  = saturation(q_goal(hip_idx),   Vec2(hip_min,   hip_max));
        // 髋关节速度也顺手限一下，防止突然抽动
        qd_goal(hip_idx) = saturation(qd_goal(hip_idx), Vec2(-1.0, 1.0));

    }

    // 将关节目标位置和速度赋值给控制接口
    for (int i = 0; i < 12; i++) {
        q_goal_debug = q_goal; // 用于调试，发布到ROS2话题
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }
}

/**
 * @brief 计算并设置关节PID增益（全局提高支撑刚度，无后腿单独补偿）
 */
void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) {
        // 先设置大腿和小腿 (j=1, 2)
        for (int j = 1; j < 3; j++) {
            if (wave_generator_->contact_(i) == 0) {
                // ================= 摆动相（脚在空中） =================
                // 增益中等，跟轨迹但不僵硬
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(220.0); // 从65
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(2.8);  
            } else {
                // ================= 支撑相（脚踩地） =================
                // 增益拉高，抗干扰、站稳
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(300.0); // 从68
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(4.5);  
            }
        }

        // 【单独设置髋关节】
        int hip_idx = i * 3 + 0;
        if (wave_generator_->contact_(i) == 0) 
        {
            // 摆动相：hip负责帮助足端横向回正，但不能太硬
            ctrl_interfaces_.joint_kp_command_interface_[hip_idx].get().set_value(220.0);
            ctrl_interfaces_.joint_kd_command_interface_[hip_idx].get().set_value(3.0);
        } 
        else 
        {
            // 支撑相：稍硬一点，提供侧向支撑，但仍低于原来的700
            ctrl_interfaces_.joint_kp_command_interface_[hip_idx].get().set_value(300.0);
            ctrl_interfaces_.joint_kd_command_interface_[hip_idx].get().set_value(5.0);
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