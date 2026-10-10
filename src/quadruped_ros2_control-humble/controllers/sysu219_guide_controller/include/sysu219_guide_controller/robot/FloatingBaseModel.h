#pragma once

#include "sysu219_guide_controller/control/WbcController.h"
#include <string>
#include <vector>

namespace sysu219::wbc {

// WBC的模型数据来源：从实际URDF建立18维浮动基模型，不使用MPC的单刚体近似。
// [接入约定] 基座原点使用base_name，与估计器一致，模型与任务均取此原点。
// 速度顺序为B系角速度、B系线速度、12关节速度；不把估计器位置当作整机质心。
// 包含URDF中腿、足和固定附件的惯性；未建模传动摩擦或折算电机转子惯量。
class FloatingBaseModel {
 public:
  FloatingBaseModel(const std::string& urdf, const std::string& base_name,
                    const std::vector<std::string>& feet_names);

  // 读取input的姿态、基座位置、关节位置和实测速度。
  // 填写四足位置、完整雅可比和加速度偏置，同时输出同一约定下的M、C。
  // URDF解析和缓存分配只在构造时进行，update正常路径不分配堆内存。
  void update(WbcInput& input, Mat18& M, Vec18& C);

 private:
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
};

}  // namespace sysu219::wbc
