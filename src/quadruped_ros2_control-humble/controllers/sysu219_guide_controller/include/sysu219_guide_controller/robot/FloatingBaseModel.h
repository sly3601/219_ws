#pragma once

#include "sysu219_guide_controller/control/WbcController.h"
#include <array>
#include <string>
#include <vector>

namespace sysu219::wbc {

// WBC的模型数据来源：从实际URDF建立18维浮动基模型，不使用MPC的单刚体近似。
// [接入约定] 基座原点使用base_name，与估计器一致，模型与任务均取此原点。
// 速度顺序为B系角速度、B系线速度、12关节速度；不把估计器位置当作整机质心。
// 包含URDF刚体惯性、关节B/Fc摩擦，以及identified_dynamics中的输出轴额外惯量Ja。
class FloatingBaseModel {
 public:
  FloatingBaseModel(const std::string& urdf, const std::string& base_name,
                    const std::vector<std::string>& feet_names);

  // 读取input的姿态、基座位置、关节位置和实测速度。
  // 输出M含关节Ja，C含重力、科氏/离心和克服当前滑动摩擦所需的力矩。
  // 同时填写四足位置、完整雅可比和加速度偏置；不修改反馈力矩或硬件方向。
  // URDF解析和缓存分配只在构造时进行，update正常路径不分配堆内存。
  void update(WbcInput& input, Mat18& M, Vec18& C);

 private:
  struct JointDynamics {
    double B = 0.0;   // 黏性摩擦，N*m*s/rad，来自joint/dynamics/@damping。
    double Fc = 0.0;  // 库仑摩擦幅值，N*m，来自joint/dynamics/@friction。
    double Ja = 0.0;  // 输出轴额外惯量，kg*m^2，来自joint/identified_dynamics/@Ja。
  };

  struct Link {
    int parent = -1;
    int joint = -1; // -1为固定连接，0～11与FR、FL、RR、RL关节顺序一致。
    Vec3 origin = Vec3::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Vec3 axis = Vec3::Zero();
    double mass = 0.0;
    Vec3 com = Vec3::Zero();
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero(); // 连杆系下、绕该连杆质心的惯量。
  };

  struct State {
    Vec3 position = Vec3::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    FootJacobian J_v = FootJacobian::Zero();
    FootJacobian J_w = FootJacobian::Zero();
    Vec3 omega = Vec3::Zero();
    Vec3 acceleration_bias = Vec3::Zero();
    Vec3 angular_acceleration_bias = Vec3::Zero();
  };

  std::vector<Link> links_;       // 父节点始终排在子节点之前，运行时只顺序遍历。
  std::vector<State> states_;     // 长度在初始化时确定，控制周期内不resize。
  std::array<int, 4> feet_{};      // 足端对应的连杆编号。
  std::array<JointDynamics, 12> joint_dynamics_{}; // 初始化按完整关节名映射，不按XML顺序。
};

}  // namespace sysu219::wbc
