//
// Created by biao on 24-9-9.
//


#ifndef HARDWAREUNITREE_H
#define HARDWAREUNITREE_H

#include "hardware_interface/system_interface.hpp"


// 新增：达妙电机相关头文件（必须引入）
#include "hardware_unitree_mujoco/motor_hw.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <memory>
#include "hardware_unitree_mujoco/imu.hpp" 


// 1. 头文件新增（加在文件顶部）
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/node.hpp"


// 新增：达妙电机数据映射类型定义（和cpp文件保持一致）
using DmActDataMap = std::unordered_map<std::string, std::unordered_map<int, damiao::DmActData>>;



class HardwareUnitree final : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    
    virtual ~HardwareUnitree();

protected:
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    std::vector<double> imu_states_;    // IMU 数据存储
    std::vector<double> foot_force_;    // no used
    std::vector<double> high_states_;   // no used

    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}}
    };

        // ========== 新增：达妙电机串口通信相关成员（仅新增，不修改原有内容） ==========
    // 串口→CAN ID→电机数据映射（存储所有达妙电机的配置和实时数据）
    DmActDataMap port_id2dm_data_;
    // 每个串口对应一个Motor_Control实例（负责该串口的底层通信）
    std::vector<std::shared_ptr<damiao::Motor_Control>> motor_ports_;

    // 新增：解析达妙电机YAML配置文件的函数声明
    bool parseDmActData(const std::string& yaml_file_path);

private:
    // IMU 实例
    std::unique_ptr<FDILink::imu> imu_driver_;
    // 参数
    std::string imu_serial_port_;
    int imu_serial_baud_;
    bool if_debug_;

    // useless possible
    int domain_ = 1;
    bool show_foot_force_ = false;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // TF广播器
    rclcpp::Node::SharedPtr node_ptr_; // 新增：关联ROS2节点

};

#endif //HARDWAREUNITREE_H
