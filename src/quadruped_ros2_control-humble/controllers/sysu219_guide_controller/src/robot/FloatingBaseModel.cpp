#include "sysu219_guide_controller/robot/FloatingBaseModel.h"

#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <stdexcept>

namespace sysu219::wbc {
namespace {

Eigen::Matrix3d skew(const Vec3& v)
{
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}

Eigen::Matrix3d rotation(const urdf::Rotation& value)
{
  return Eigen::Quaterniond(value.w, value.x, value.y, value.z).toRotationMatrix();
}

Vec3 vector(const urdf::Vector3& value)
{
  return Vec3(value.x, value.y, value.z);
}

}  // namespace


FloatingBaseModel::FloatingBaseModel(const std::string& urdf, const std::string& base_name,
                                   const std::vector<std::string>& feet_names)
{
  urdf::Model model;
  if (!model.initString(urdf) || !model.getLink(base_name) || feet_names.size() != 4)
    throw std::invalid_argument("Invalid floating-base URDF or foot names");

  // 沿四条足端路径确定关节编号，保证与已有KDL单腿模型的排列完全一致。
  std::map<std::string, int> joint_indices;
  for (int leg = 0; leg < 4; ++leg)
  {
    std::vector<std::string> joints;
    auto link = model.getLink(feet_names[leg]);
    while (link && link->name != base_name)
    {
      if (!link->parent_joint) break;
      if (link->parent_joint->type != urdf::Joint::FIXED)
        joints.push_back(link->parent_joint->name);
      link = link->getParent();
    }
    if (!link || link->name != base_name || joints.size() != 3)
      throw std::invalid_argument("Each WBC foot must have three joints below base_name");
    std::reverse(joints.begin(), joints.end());
    for (int joint = 0; joint < 3; ++joint)
      if (!joint_indices.emplace(joints[joint], 3 * leg + joint).second)
        throw std::invalid_argument("WBC leg paths must have independent joints");
  }

  // 保存所有连杆，包括不在足端路径上的固定转子外壳、IMU等附件，避免漏算质量。
  // std::map/std::function只在初始化使用，不进入控制周期。
  links_.reserve(model.links_.size());
  std::map<std::string, int> link_indices;
  std::function<void(urdf::LinkConstSharedPtr, int)> append =
      [&](urdf::LinkConstSharedPtr source, int parent)
  {
    Link link;
    link.parent = parent;
    if (parent >= 0)
    {
      const auto& joint = *source->parent_joint;
      link.origin = vector(joint.parent_to_joint_origin_transform.position);
      link.rotation = rotation(joint.parent_to_joint_origin_transform.rotation);
      if (joint.type != urdf::Joint::FIXED)
      {
        const auto index = joint_indices.find(joint.name);
        if (index == joint_indices.end() || joint.mimic ||
            (joint.type != urdf::Joint::REVOLUTE && joint.type != urdf::Joint::CONTINUOUS))
          throw std::invalid_argument("Unsupported joint in floating-base model: " + joint.name);
        link.joint = index->second;
        link.axis = vector(joint.axis);
        if (!link.axis.allFinite() || link.axis.norm() < 1e-12)
          throw std::invalid_argument("Invalid WBC joint axis");
        link.axis.normalize();
      }
    }
    if (source->inertial)
    {
      const auto& inertia = *source->inertial;
      link.mass = inertia.mass;
      link.com = vector(inertia.origin.position);
      Eigen::Matrix3d I;
      I << inertia.ixx, inertia.ixy, inertia.ixz,
           inertia.ixy, inertia.iyy, inertia.iyz,
           inertia.ixz, inertia.iyz, inertia.izz;
      const auto R = rotation(inertia.origin.rotation);
      link.inertia.noalias() = R * I * R.transpose();
      if (!std::isfinite(link.mass) || link.mass < 0.0 || !link.com.allFinite() ||
          !link.inertia.allFinite() ||
          (link.mass > 0.0 && Eigen::LLT<Eigen::Matrix3d>(link.inertia).info() != Eigen::Success))
        throw std::invalid_argument("Invalid WBC link inertia: " + source->name);
    }
    const int index = static_cast<int>(links_.size());
    links_.push_back(link);
    link_indices.emplace(source->name, index);
    for (const auto& child : source->child_links) append(child, index);
  };
  append(model.getLink(base_name), -1);
  for (int leg = 0; leg < 4; ++leg) feet_[leg] = link_indices.at(feet_names[leg]);
  states_.resize(links_.size());
}


void FloatingBaseModel::update(WbcInput& input, Mat18& M, Vec18& C)
{
  if (!input.q_j.allFinite() || !input.qdot.allFinite() || !input.R_OB.allFinite() ||
      !input.p_com_O.allFinite())
    throw std::invalid_argument("Nonfinite floating-base state");

  M.setZero();
  C.setZero();
  const Vec3 gravity_B = input.R_OB.transpose() * Vec3(0.0, 0.0, -9.81);

  // 1. 从基座向各连杆递推位姿、雅可比和零广义加速度时的实际运动加速度。
  // 所有中间量用当前B系表达，最后统一把足端量转到世界系。
  for (std::size_t i = 0; i < links_.size(); ++i)
  {
    const auto& link = links_[i];
    auto& state = states_[i];
    if (link.parent < 0)
    {
      state.position.setZero();
      state.rotation.setIdentity();
      state.J_v.setZero();
      state.J_w.setZero();
      state.J_v.block<3, 3>(0, 3).setIdentity();
      state.J_w.leftCols<3>().setIdentity();
      state.omega = input.qdot.head<3>();
      state.angular_acceleration_bias.setZero();
      // qddot的线性部分是B系速度的导数，因此实际线加速度还包含这个叉乘。
      state.acceleration_bias = state.omega.cross(input.qdot.segment<3>(3));
    }
    else
    {
      const auto& parent = states_[link.parent];
      const Vec3 r = parent.rotation * link.origin;
      state.position = parent.position + r;
      state.rotation.noalias() = parent.rotation * link.rotation;
      state.J_v.noalias() = parent.J_v - skew(r) * parent.J_w;
      state.J_w = parent.J_w;
      state.omega = parent.omega;
      state.angular_acceleration_bias = parent.angular_acceleration_bias;
      state.acceleration_bias = parent.acceleration_bias +
          parent.angular_acceleration_bias.cross(r) + parent.omega.cross(parent.omega.cross(r));

      if (link.joint >= 0)
      {
        const Vec3 axis_B = state.rotation * link.axis;
        const Vec3 relative_omega = axis_B * input.qdot(6 + link.joint);
        state.J_w.col(6 + link.joint) += axis_B;
        state.omega += relative_omega;
        state.angular_acceleration_bias += parent.omega.cross(relative_omega);
        state.rotation = (state.rotation *
            Eigen::AngleAxisd(input.q_j(link.joint), link.axis).toRotationMatrix()).eval();
      }
    }

    // 2. 汇总每个刚体的动能系数与零广义加速度所需的惯性/重力作用，得到完整M、C。
    // 这里保留所有基座与关节耦合；质心偏移同时影响质量矩阵和动力学偏置。
    if (link.mass > 0.0)
    {
      const Vec3 r_com = state.rotation * link.com;
      const FootJacobian J_com = state.J_v - skew(r_com) * state.J_w;
      const Eigen::Matrix3d I_B = state.rotation * link.inertia * state.rotation.transpose();
      const Vec3 a_com_bias = state.acceleration_bias +
          state.angular_acceleration_bias.cross(r_com) + state.omega.cross(state.omega.cross(r_com));
      const Vec3 force_bias = link.mass * (a_com_bias - gravity_B);
      const Vec3 torque_bias = I_B * state.angular_acceleration_bias +
          state.omega.cross(I_B * state.omega);
      M.noalias() += link.mass * J_com.transpose() * J_com;
      M.noalias() += state.J_w.transpose() * I_B * state.J_w;
      C.noalias() += J_com.transpose() * force_bias + state.J_w.transpose() * torque_bias;
    }
  }

  // 3. 同一模型生成WBC足端任务数据，避免拼接来自不同模型的M、J和偏置。
  for (int leg = 0; leg < 4; ++leg)
  {
    const auto& foot = states_[feet_[leg]];
    input.x_f_O.col(leg) = input.p_com_O + input.R_OB * foot.position;
    input.J_f_O[leg].noalias() = input.R_OB * foot.J_v;
    input.Jdot_f_qdot_O.col(leg).noalias() = input.R_OB * foot.acceleration_bias;
  }
  input.Jdot_orientation_qdot_O.setZero();
  input.Jdot_position_qdot_O = input.R_OB *
      input.qdot.head<3>().cross(input.qdot.segment<3>(3));
  if (!M.allFinite() || !C.allFinite())
    throw std::invalid_argument("Nonfinite floating-base dynamics");
}

}  // namespace sysu219::wbc
