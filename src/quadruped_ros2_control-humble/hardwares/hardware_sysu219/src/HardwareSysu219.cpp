//
// Created by biao on 24-9-9.
//

#include "hardware_sysu219/HardwareSysu219.h"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "crc32.h"

#include <cmath>
#include <unordered_map>


using hardware_interface::return_type;


// ========== 新增：全局函数和变量，用于电机状态/指令修正（转向+偏置） ==========
namespace
{
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float TWO_PI_F = 2.0f * PI_F;

    // 达妙驱动默认位置量程：±12.5 rad
    constexpr float DM_PMAX_F = 12.5f;
    // 靠近边界时报警阈值，你可以自己改
    constexpr float EDGE_WARN_THRESHOLD_F = 11.5f;

    // 当前这一拍驱动直接返回的原始位置（不累计，不展开）
    std::unordered_map<std::string, float> g_current_raw_pos;

    // 防止报警刷屏
    std::unordered_map<std::string, bool> g_edge_warned;
    // 记录上一次的底层原始位置
    std::unordered_map<std::string, float> g_last_good_raw_pos;
    // 底层原始位置忍受的最大跳变（单位：rad），超过这个值认为是底层原始异常读数
    constexpr float RAW_JUMP_HARD_STOP_F = 2.5f;

}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareSysu219::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    joint_torque_command_.assign(12, 0);
    joint_position_command_.assign(12, 0);
    joint_velocities_command_.assign(12, 0);
    joint_kp_command_.assign(12, 0);
    joint_kd_command_.assign(12, 0);

    joint_position_.assign(12, 0);
    joint_velocities_.assign(12, 0);
    joint_effort_.assign(12, 0);

    imu_states_.assign(10, 0);
    foot_force_.assign(4, 0);
    high_states_.assign(6, 0);

    g_current_raw_pos.clear();
    g_edge_warned.clear();
    g_last_good_raw_pos.clear();

    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }
    /*
        double pos_min;  // 位置最小值
        double pos_max;  // 位置最大值
        double vel_min;  // 速度最小值（负值）
        double vel_max;  // 速度最大值
        double torque_min;  // 扭矩最小值（负值）
        double torque_max;  // 扭矩最大值
    */
     // 修改限幅2026.02.06
        // 按类型定义限位（匹配xacro的const.xacro值，同一类共用）
        joint_type_limits_["hip"] = {-0.638, 0.748, -30.1, 30.1, -100.0, 100.0};
        joint_type_limits_["thigh"] = {-0.623, 1.65, -30.1, 30.1, -100.0, 100.0};
        joint_type_limits_["calf"] = {-2.322, -0.672, -20.06, 20.06, -100.0, 100.0};

    // ========== imu改动 ==========
    // 获取参数
    imu_serial_port_ = info_.hardware_parameters.at("imu_serial_port");
    imu_serial_baud_ = std::stoi(info_.hardware_parameters.at("imu_serial_baud"));
    if_debug_ = info_.hardware_parameters.at("debug") == "true";
    // ========== imu改动 ==========
    try {
        // 初始化 IMU 驱动
        imu_driver_ = std::make_unique<FDILink::imu>(); // FDILink是自定义命名空间，imu是自定义类
        
        // 设置参数（可以通过参数服务器传递）
        // 注意：您可能需要修改您的 imu 类以支持参数设置
        
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("imu_driver"), "Failed to configure IMU driver: %s", e.what());
        return CallbackReturn::ERROR;
    }


    // ========== 新增：初始化TF广播器 ==========
    node_ptr_ = rclcpp::Node::make_shared("hardware_sysu219_tf_node");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_ptr_);




        // ========== 核心改动2：新增达妙电机串口初始化逻辑 ==========
    // 1. 从hardware_parameters中读取达妙电机YAML配置文件路径
    std::string yaml_file_path = "";
    if (info_.hardware_parameters.find("yaml_file_path") != info_.hardware_parameters.end()) {
        yaml_file_path = info_.hardware_parameters.at("yaml_file_path");
    } else {
        RCLCPP_FATAL(rclcpp::get_logger("sysu219_hardware"), "未配置达妙电机YAML文件路径（yaml_file_path）！");
        return CallbackReturn::ERROR;
    }

    // 2. 解析达妙电机配置（串口、CAN ID、电机类型等）
    if (!parseDmActData(yaml_file_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("sysu219_hardware"), 
            "解析达妙电机配置失败：%s", yaml_file_path.c_str());
        return CallbackReturn::ERROR;
    }




    // return SystemInterface::on_init(info);
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HardwareSysu219::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    // export imu sensor state interface
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]);
    }

    // export foot force sensor state interface
    if (info_.sensors.size() > 1)
    {
        for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_force_[i]);
        }
    }

    // export odometer state interface
    if (info_.sensors.size() > 2)
    {
        // export high state interface
        for (uint i = 0; i < info_.sensors[2].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[2].name, info_.sensors[2].state_interfaces[i].name, &high_states_[i]);
        }
    }


    return
        state_interfaces;
}




std::vector<hardware_interface::CommandInterface> HardwareSysu219::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
        ind++;
    }
    return command_interfaces;
}

return_type HardwareSysu219::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // ========== 核心改动：删除宇树网络读取逻辑 ==========
    /*
    // joint states
    for (int i(0); i < 12; ++i)
    {
        joint_position_[i] = low_state_.motor_state()[i].q();
        joint_velocities_[i] = low_state_.motor_state()[i].dq();
        joint_effort_[i] = low_state_.motor_state()[i].tau_est();
    }

    // imu states
    imu_states_[0] = low_state_.imu_state().quaternion()[0]; // w
    imu_states_[1] = low_state_.imu_state().quaternion()[1]; // x
    imu_states_[2] = low_state_.imu_state().quaternion()[2]; // y
    imu_states_[3] = low_state_.imu_state().quaternion()[3]; // z
    imu_states_[4] = low_state_.imu_state().gyroscope()[0];
    imu_states_[5] = low_state_.imu_state().gyroscope()[1];
    imu_states_[6] = low_state_.imu_state().gyroscope()[2];
    imu_states_[7] = low_state_.imu_state().accelerometer()[0];
    imu_states_[8] = low_state_.imu_state().accelerometer()[1];
    imu_states_[9] = low_state_.imu_state().accelerometer()[2];

    // contact states
    foot_force_[0] = low_state_.foot_force()[0];
    foot_force_[1] = low_state_.foot_force()[1];
    foot_force_[2] = low_state_.foot_force()[2];
    foot_force_[3] = low_state_.foot_force()[3];

    if (show_foot_force_)
    {
        RCLCPP_INFO(rclcpp::get_logger("sysu219_hardware"), "foot_force(): %f, %f, %f, %f", foot_force_[0], foot_force_[1], foot_force_[2],
                    foot_force_[3]);
    }

    // high states
    high_states_[0] = high_state_.position()[0];
    high_states_[1] = high_state_.position()[1];
    high_states_[2] = high_state_.position()[2];
    high_states_[3] = high_state_.velocity()[0];
    high_states_[4] = high_state_.velocity()[1];
    high_states_[5] = high_state_.velocity()[2];
    */

    // ========== 核心改动：新增达妙串口读取状态逻辑 ==========
    // 1. 调用每个串口的Motor_Control::read()，从硬件读取电机状态
    for (auto& motor_control : motor_ports_) {
        motor_control->read();
    }

    for (auto& port_entry : port_id2dm_data_) 
    {
        for (auto& can_entry : port_entry.second) 
        {
            auto& dm_data = can_entry.second;
            const std::string& name = dm_data.name;

            // 1. 先记录驱动这一拍直接返回的原始位置（多圈值，可能到 ±12.5）
            float raw_now = dm_data.pos;
            if (!std::isfinite(raw_now)) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("sysu219_hardware"),
                    "硬停：电机[%s] raw_now 非法！",
                    name.c_str());

                for (auto& port_entry_stop : port_id2dm_data_) {
                    for (auto& can_entry_stop : port_entry_stop.second) {
                        can_entry_stop.second.cmd_effort = 0.0f;
                        can_entry_stop.second.kp = 0.0f;
                        can_entry_stop.second.kd = 0.0f;
                        can_entry_stop.second.cmd_vel = 0.0f;
                    }
                }
                for (auto& motor_control_stop : motor_ports_) {
                    motor_control_stop->write();
                }
                while (1) {}
            }
            // 第一拍：只记录，不判断跳变
            if (g_last_good_raw_pos.find(name) == g_last_good_raw_pos.end()) {
                g_last_good_raw_pos[name] = raw_now;
            }
            else {
                float last_raw_now = g_last_good_raw_pos[name];

                if (std::fabs(raw_now - last_raw_now) > RAW_JUMP_HARD_STOP_F) {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("sysu219_hardware"),
                        "硬停：电机[%s] raw位置异常跳变！last=%.4f now=%.4f diff=%.4f",
                        name.c_str(),
                        last_raw_now,
                        raw_now,
                        std::fabs(raw_now - last_raw_now));

                    // 先强制切断所有电机关节力矩
                    for (auto& port_entry_stop : port_id2dm_data_) {
                        for (auto& can_entry_stop : port_entry_stop.second) {
                            can_entry_stop.second.cmd_effort = 0.0f;
                            can_entry_stop.second.kp = 0.0f;
                            can_entry_stop.second.kd = 0.0f;
                            can_entry_stop.second.cmd_vel = 0.0f;
                        }
                    }

                    for (auto& motor_control_stop : motor_ports_) {
                        motor_control_stop->write();
                    }

                    while (1) {}
                }
            }

            g_current_raw_pos[name] = raw_now;
            g_last_good_raw_pos[name] = raw_now;

            // 2. 如果靠近 ±12.5，报警
            if (std::fabs(raw_now) > EDGE_WARN_THRESHOLD_F) {
                if (!g_edge_warned[name]) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("sysu219_hardware"),
                        "电机[%s] 原始位置 raw=%.4f 接近 ±12.5rad 边界，当前位置控制存在分支风险！",
                        name.c_str(), raw_now);
                    g_edge_warned[name] = true;
                }
            } else {
                g_edge_warned[name] = false;
            }

            // 3. 再做状态映射
            correctMotorState(dm_data);
        }
    }

    // 2. 把达妙电机的状态赋值给ROS2的状态数组（joint_position_/joint_velocities_/joint_effort_）
    // 2.1 读取位置状态
    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"]) {
        bool found = false;
        for (auto& port_entry : port_id2dm_data_) {
            for (auto& can_entry : port_entry.second) {
                if (can_entry.second.name == joint_name) {
                    joint_position_[ind] = can_entry.second.pos;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        if (!found) {
            RCLCPP_WARN(rclcpp::get_logger("sysu219_hardware"), 
                "关节%s未找到对应达妙电机！", joint_name.c_str());
        }
        ind++;
    }

    // 2.2 读取速度状态
    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"]) {
        bool found = false;
        for (auto& port_entry : port_id2dm_data_) {
            for (auto& can_entry : port_entry.second) {
                if (can_entry.second.name == joint_name) {
                    joint_velocities_[ind] = can_entry.second.vel;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        ind++;
    }

    // 2.3 读取扭矩状态
    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"]) {
        bool found = false;
        for (auto& port_entry : port_id2dm_data_) {
            for (auto& can_entry : port_entry.second) {
                if (can_entry.second.name == joint_name) {
                    joint_effort_[ind] = can_entry.second.effort;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        ind++;
    }

    // ========== imu改动 ==========
    if (imu_driver_ && imu_driver_->check_newdata()) {
    auto imu_data = imu_driver_->return_latest_imu_data();
    
    imu_states_[0] = imu_data.orientation.w;
    imu_states_[1] = imu_data.orientation.x;
    imu_states_[2] = imu_data.orientation.y;
    imu_states_[3] = imu_data.orientation.z;

    imu_states_[4] = imu_data.angular_velocity.x;
    imu_states_[5] = imu_data.angular_velocity.y;
    imu_states_[6] = imu_data.angular_velocity.z;
    imu_states_[7] = imu_data.linear_acceleration.x;
    imu_states_[8] = imu_data.linear_acceleration.y;
    imu_states_[9] = imu_data.linear_acceleration.z;


    // ========== 修正：发布base→trunk的TF（用Node的时钟） ==========
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = node_ptr_->get_clock()->now(); // 改用Node的时钟
    tf_msg.header.frame_id = "base";       
    tf_msg.child_frame_id = "trunk";      
    // 平移永远为0（只转不移）
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    // 旋转用IMU的四元数（注意顺序匹配）
    tf_msg.transform.rotation.w = imu_data.orientation.w;
    tf_msg.transform.rotation.x = imu_data.orientation.x;
    tf_msg.transform.rotation.y = imu_data.orientation.y;
    tf_msg.transform.rotation.z = imu_data.orientation.z;
    // 发布TF
    // 2026.04.12 我把这一行注释了！用于开环调试！闭环的时候要调回来并且还要改开闭环控制参数的逻辑！
    // tf_broadcaster_->sendTransform(tf_msg);
    
  }
    // ========== 可选：保留足端力逻辑（如果需要） ==========
    // 如果你仍需要足端力数据，可保留原有逻辑
    foot_force_.assign(4, 0);   // 临时赋值0，可按需修改
    high_states_.assign(6, 0);  // 临时赋值0，可按需修改

    return return_type::OK;
}

return_type HardwareSysu219::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // ========== 核心改动：新增达妙串口下发指令逻辑 ==========
    // 1. 把ROS2的指令（joint_xxx_command_）赋值给达妙电机的DmActData
    // 1.1 处理位置指令
    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"]) {
        bool found = false;

        // 修改限幅2026.02.06：按关节名判断类型，应用对应限位
        if (joint_name.find("hip") != std::string::npos) {
            auto& limit = joint_type_limits_["hip"];
            joint_position_command_[ind] = std::clamp(joint_position_command_[ind], limit.pos_min, limit.pos_max);
        } else if (joint_name.find("thigh") != std::string::npos) {
            auto& limit = joint_type_limits_["thigh"];
            joint_position_command_[ind] = std::clamp(joint_position_command_[ind], limit.pos_min, limit.pos_max);
        } else if (joint_name.find("calf") != std::string::npos) {
            auto& limit = joint_type_limits_["calf"];
            joint_position_command_[ind] = std::clamp(joint_position_command_[ind], limit.pos_min, limit.pos_max);
        }


        // 遍历所有串口→所有电机，匹配关节名
        for (auto& port_entry : port_id2dm_data_) {
            for (auto& can_entry : port_entry.second) {
                if (can_entry.second.name == joint_name) {
                    can_entry.second.cmd_pos = static_cast<float>(joint_position_command_[ind]);
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        if (!found) {
            RCLCPP_WARN(rclcpp::get_logger("sysu219_hardware"), 
                "关节%s未找到对应达妙电机！", joint_name.c_str());
        }
        ind++;
    }

    // 1.2 处理速度指令
    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"]) {
        bool found = false;

        // 修改限幅2026.02.06：按关节名判断类型，应用对应速度限位
        if (joint_name.find("hip") != std::string::npos) {
            auto& limit = joint_type_limits_["hip"];
            joint_velocities_command_[ind] = std::clamp(joint_velocities_command_[ind], limit.vel_min, limit.vel_max);
        } else if (joint_name.find("thigh") != std::string::npos) {
            auto& limit = joint_type_limits_["thigh"];
            joint_velocities_command_[ind] = std::clamp(joint_velocities_command_[ind], limit.vel_min, limit.vel_max);
        } else if (joint_name.find("calf") != std::string::npos) {
            auto& limit = joint_type_limits_["calf"];
            joint_velocities_command_[ind] = std::clamp(joint_velocities_command_[ind], limit.vel_min, limit.vel_max);
        }


        for (auto& port_entry : port_id2dm_data_) {
            for (auto& can_entry : port_entry.second) {
                if (can_entry.second.name == joint_name) {
                    can_entry.second.cmd_vel = static_cast<float>(joint_velocities_command_[ind]);
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        ind++;
    }

    // 1.3 处理扭矩+KP/KD指令
    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"]) {
        bool found = false;
        // 修改限幅2026.02.06：按关节名判断类型，应用对应扭矩限位
        if (joint_name.find("hip") != std::string::npos) {
            // RCLCPP_INFO(rclcpp::get_logger("sysu219_hardware1111"),"");
            auto& limit = joint_type_limits_["hip"];
            joint_torque_command_[ind] = std::clamp(joint_torque_command_[ind], limit.torque_min, limit.torque_max);
        } else if (joint_name.find("thigh") != std::string::npos) {
            auto& limit = joint_type_limits_["thigh"];
            joint_torque_command_[ind] = std::clamp(joint_torque_command_[ind], limit.torque_min, limit.torque_max);
        } else if (joint_name.find("calf") != std::string::npos) {
            auto& limit = joint_type_limits_["calf"];
            joint_torque_command_[ind] = std::clamp(joint_torque_command_[ind], limit.torque_min, limit.torque_max);
        }

        const double max_pos_error = 1.5;  // 极简统一阈值，先用1.5rad

        if (joint_kp_command_[ind] > 1e-3 &&
            std::fabs(joint_position_command_[ind] - joint_position_[ind]) > max_pos_error) {
            RCLCPP_FATAL(
                rclcpp::get_logger("sysu219_hardware"),
                "硬停：关节[%s] 位置命令超差！cmd=%.4f current=%.4f diff=%.4f",
                joint_name.c_str(),
                joint_position_command_[ind],
                joint_position_[ind],
                std::fabs(joint_position_command_[ind] - joint_position_[ind]));
            
            // 先强制切断所有电机关节力矩
            for (auto& port_entry_stop : port_id2dm_data_) {
                for (auto& can_entry_stop : port_entry_stop.second) {
                    can_entry_stop.second.cmd_effort = 0.0f;
                    can_entry_stop.second.kp = 0.0f;
                    can_entry_stop.second.kd = 0.0f;
                    can_entry_stop.second.cmd_vel = 0.0f;
                }
            }

            for (auto& motor_control_stop : motor_ports_) {
                motor_control_stop->write();
            }
            while (1) {}
        }


        for (auto& port_entry : port_id2dm_data_) {
            for (auto& can_entry : port_entry.second) {
                if (can_entry.second.name == joint_name) {
                    can_entry.second.cmd_effort = static_cast<float>(joint_torque_command_[ind]);
                    can_entry.second.kp = static_cast<float>(joint_kp_command_[ind]);
                    can_entry.second.kd = static_cast<float>(joint_kd_command_[ind]);
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        ind++;
    }

    // ========== 新增：修正所有电机的下发指令 ==========
    for (auto& port_entry : port_id2dm_data_) {
        for (auto& can_entry : port_entry.second) {
            correctMotorCommand(can_entry.second);
        }
    }

    // 2. 调用每个串口的Motor_Control::write()，下发指令到电机
    for (auto& motor_control : motor_ports_) {
        motor_control->write();
    }

    return return_type::OK;
}





// 新增：解析达妙电机YAML配置的函数
// 函数作用：将12个电机的基础信息，ID等参数放在port_id2dm_data_里面
// 函数作用：创建Motor_Control实例
bool HardwareSysu219::parseDmActData(const std::string& yaml_file_path)
{
    try {
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        
        if (!config["motor_ports"] || !config["motors"]) {
            RCLCPP_ERROR(rclcpp::get_logger("sysu219_hardware"), 
                "YAML文件缺少motor_ports或motors节点！");
            return false;
        }

        // 1. 解析motors配置（电机参数）
        const auto& motors = config["motors"];
        if (motors.IsSequence()) { // 判断当前 YAML 节点是否是「序列类型」（对应 YAML 中的数组 / 列表，用短横线-定义的结构）。
            for (const auto& motor : motors) {
                std::string port = motor["port"].as<std::string>();
                int can_id = motor["can_id"].as<int>();
                int master_id = motor["master_id"].as<int>();
                std::string name = motor["name"].as<std::string>();
                std::string type_str = motor["type"].as<std::string>();
                
                // 转换电机类型字符串为枚举
                damiao::DM_Motor_Type motor_type;
                if (type_str == "DM10010") motor_type = damiao::DM10010;
                // 其他电机类型可按需添加（如DM4310等）
                else {
                    RCLCPP_ERROR(rclcpp::get_logger("sysu219_hardware"), 
                        "未知电机类型：%s", type_str.c_str());
                    return false;
                }
                
                // 创建DmActData并存储
                damiao::DmActData dm_data;
                dm_data.name = name;
                dm_data.motorType = motor_type;
                dm_data.can_id = can_id;
                dm_data.mst_id = master_id;
                // ========== 新增：读取转向+偏置参数 ==========
                dm_data.direction = motor["direction"].as<int>(1); // 默认正向
                dm_data.offset = motor["offset"].as<float>(0.0f);  // 默认无偏置

                port_id2dm_data_[port][can_id] = dm_data;

                RCLCPP_INFO(rclcpp::get_logger("sysu219_hardware"), 
                    "添加电机：%s (串口：%s, CAN ID：%d, Master ID：%d)", 
                    name.c_str(), port.c_str(), can_id, master_id);
            }
        }

        // 2. 解析motor_ports配置（串口参数），创建Motor_Control实例
        const auto& ports = config["motor_ports"];
        if (ports.IsSequence()) {
            motor_ports_.clear();
            for (const auto& port : ports) {
                std::string port_name = port["port"].as<std::string>();
                int baudrate = port["baudrate"].as<int>();

                // 创建该串口的Motor_Control实例（负责串口通信）
                auto motor_control = std::make_shared<damiao::Motor_Control>(
                    port_name, baudrate, &port_id2dm_data_[port_name]);
                motor_ports_.push_back(motor_control);

                RCLCPP_INFO(rclcpp::get_logger("sysu219_hardware"), 
                    "初始化串口：%s (波特率：%d)", port_name.c_str(), baudrate);
            }
        }

        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("sysu219_hardware"), 
            "解析YAML失败：%s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("sysu219_hardware"), 
            "初始化达妙电机失败：%s", e.what());
        return false;
    }
}



void HardwareSysu219::correctMotorState(damiao::DmActData& dm_data) {
    // 1. 先做方向和 offset 映射
    float q_model = dm_data.pos * dm_data.direction + dm_data.offset;

    // 2. 只放宽“换算窗口”，不改真实关节限位
    const float read_margin = 1.0f;   // 你可以先试 1.0，再看情况改成 0.8 或 1.2

    if (dm_data.name.find("hip") != std::string::npos) {
        auto& limit = joint_type_limits_["hip"];
        float read_min = limit.pos_min - read_margin;
        float read_max = limit.pos_max + read_margin;

        while (q_model < read_min) q_model += TWO_PI_F;
        while (q_model > read_max) q_model -= TWO_PI_F;
    } else if (dm_data.name.find("thigh") != std::string::npos) {
        auto& limit = joint_type_limits_["thigh"];
        float read_min = limit.pos_min - read_margin;
        float read_max = limit.pos_max + read_margin;

        while (q_model < read_min) q_model += TWO_PI_F;
        while (q_model > read_max) q_model -= TWO_PI_F;
    } else if (dm_data.name.find("calf") != std::string::npos) {
        auto& limit = joint_type_limits_["calf"];
        float read_min = limit.pos_min - read_margin;
        float read_max = limit.pos_max + read_margin;

        while (q_model < read_min) q_model += TWO_PI_F;
        while (q_model > read_max) q_model -= TWO_PI_F;
    }

    dm_data.pos = q_model;

    // 3. 速度、力矩照旧
    dm_data.vel = dm_data.vel * dm_data.direction;
    dm_data.effort = dm_data.effort * dm_data.direction;
}


void HardwareSysu219::correctMotorCommand(damiao::DmActData& dm_data) {
    auto it = g_current_raw_pos.find(dm_data.name);
    if (it == g_current_raw_pos.end()) return;

    float raw_now = it->second;

    // 1. 上层目标角先反算成 raw 空间基础解
    float raw_cmd_base = (dm_data.cmd_pos - dm_data.offset) / dm_data.direction;

    // 2. 用“当前这一拍的 raw_now”选最近的等价 2π 分支
    float k = std::round((raw_now - raw_cmd_base) / TWO_PI_F);
    float raw_cmd_candidate = raw_cmd_base + k * TWO_PI_F;

    // 3. 如果当前 raw 或目标 raw 已经靠近 ±12.5，报警
    if (std::fabs(raw_now) > EDGE_WARN_THRESHOLD_F ||
        std::fabs(raw_cmd_candidate) > EDGE_WARN_THRESHOLD_F) {
        RCLCPP_WARN(
            rclcpp::get_logger("sysu219_hardware"),
            "电机[%s] 当前位置 raw=%.4f 或目标 raw=%.4f 接近 ±12.5rad 边界，分支选择风险较高！",
            dm_data.name.c_str(), raw_now, raw_cmd_candidate);
    }

    // 4. 如果目标已经超出驱动默认量程，也报警
    if (std::fabs(raw_cmd_candidate) > DM_PMAX_F) {
        RCLCPP_ERROR(
            rclcpp::get_logger("sysu219_hardware"),
            "电机[%s] 目标 raw=%.4f 超出驱动默认 ±12.5rad 量程！",
            dm_data.name.c_str(), raw_cmd_candidate);

        // 先强制切断所有电机关节力矩
        for (auto& port_entry_stop : port_id2dm_data_) {
            for (auto& can_entry_stop : port_entry_stop.second) {
                can_entry_stop.second.cmd_effort = 0.0f;
                can_entry_stop.second.kp = 0.0f;
                can_entry_stop.second.kd = 0.0f;
                can_entry_stop.second.cmd_vel = 0.0f;
            }
        }

        for (auto& motor_control_stop : motor_ports_) {
            motor_control_stop->write();
        }
        while (1) {}
    }

    dm_data.cmd_pos = raw_cmd_candidate;

    // 5. 速度、力矩照旧反向映射
    dm_data.cmd_vel = dm_data.cmd_vel / dm_data.direction;
    dm_data.cmd_effort = dm_data.cmd_effort / dm_data.direction;
}


// 新增析构函数：释放达妙电机串口和线程资源
HardwareSysu219::~HardwareSysu219() {
    // 清空motor_ports_，自动调用Motor_Control的析构函数（关闭串口、停止线程）
    motor_ports_.clear();
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    HardwareSysu219, hardware_interface::SystemInterface)
