//
// Created by biao on 24-9-12.
//

#include <iostream>
#include "controller_common/CtrlInterfaces.h"
#include "sysu219_guide_controller/robot/QuadrupedRobot.h"
#include <limits>
#include <cmath>
#include <stdexcept>
#include <urdf/model.h>

QuadrupedRobot::QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                               const std::vector<std::string> &feet_names,
                               const std::string &base_name) : ctrl_interfaces_(ctrl_interfaces) {
    KDL::Tree robot_tree;
    kdl_parser::treeFromString(robot_description, robot_tree);

    robot_tree.getChain(base_name, feet_names[0], fr_chain_);
    robot_tree.getChain(base_name, feet_names[1], fl_chain_);
    robot_tree.getChain(base_name, feet_names[2], rr_chain_);
    robot_tree.getChain(base_name, feet_names[3], rl_chain_);

    urdf::Model model;
    if (!model.initString(robot_description))
        throw std::runtime_error("Cannot read joint limits from robot_description");
    const KDL::Chain* chains[] = {&fr_chain_, &fl_chain_, &rr_chain_, &rl_chain_};
    for (int leg = 0; leg < 4; ++leg) {
        int joint_index = 0;
        for (const auto& segment : chains[leg]->segments) {
            const auto& kdl_joint = segment.getJoint();
            if (kdl_joint.getType() == KDL::Joint::Fixed) continue;
            const auto joint = model.getJoint(kdl_joint.getName());
            if (joint_index >= 3 || !joint || !joint->limits ||
                !std::isfinite(joint->limits->lower) || !std::isfinite(joint->limits->upper) ||
                !std::isfinite(joint->limits->velocity) ||
                joint->limits->lower >= joint->limits->upper || joint->limits->velocity <= 0.0)
                throw std::runtime_error("Missing/invalid limits for joint " + kdl_joint.getName());
            const int index = 3 * leg + joint_index++;
            joint_lower_(index) = joint->limits->lower;
            joint_upper_(index) = joint->limits->upper;
            joint_velocity_limit_(index) = joint->limits->velocity;
        }
        if (joint_index != 3) throw std::runtime_error("Expected three joints per leg");
    }


    robot_legs_.resize(4);
    robot_legs_[0] = std::make_shared<RobotLeg>(fr_chain_);
    robot_legs_[1] = std::make_shared<RobotLeg>(fl_chain_);
    robot_legs_[2] = std::make_shared<RobotLeg>(rr_chain_);
    robot_legs_[3] = std::make_shared<RobotLeg>(rl_chain_);

    current_joint_pos_.resize(4);
    current_joint_vel_.resize(4);

    std::cout << "robot_legs_.size(): " << robot_legs_.size() << std::endl;

    // calculate total mass from urdf
    mass_ = 0;
    for (const auto &[fst, snd]: robot_tree.getSegments()) {
        mass_ += snd.segment.getInertia().getMass();
    }
    // 下面数据不可用！下面是宇树go2的参数！因为是足底位置！
    feet_pos_normal_stand_ << 0.1881, 0.1881, -0.1881, -0.1881, -0.1300, 0.1300, 
            -0.1300, 0.1300, -0.3200, -0.3200, -0.3200, -0.3200;  // 此处证明X 轴正方向：向前。
}

std::vector<KDL::JntArray> QuadrupedRobot::getQ(const std::vector<KDL::Frame> &pEe_list,
        VecInt4* status, Vec4* error) const {
    std::vector<KDL::JntArray> result;
    result.resize(4);
    for (int i(0); i < 4; ++i) {
        result[i] = robot_legs_[i]->calcQ(pEe_list[i], current_joint_pos_[i], status ? &(*status)(i) : nullptr);
        if (error) (*error)(i) = (robot_legs_[i]->calcPEe2B(result[i]).p - pEe_list[i].p).Norm();
    }
    return result;
}

// 注意这个函数是强旋转约束
Vec12 QuadrupedRobot::getQ(const Vec34 &vecP, VecInt4* status, Vec4* error) const {
    Vec12 q;
    for (int i(0); i < 4; ++i) {
        KDL::Frame frame;
        frame.p = KDL::Vector(vecP.col(i)[0], vecP.col(i)[1], vecP.col(i)[2]);
        frame.M = KDL::Rotation::Identity();
        const auto leg_q = robot_legs_[i]->calcQ(frame, current_joint_pos_[i], status ? &(*status)(i) : nullptr);
        q.segment(3 * i, 3) = leg_q.data;
        if (error) (*error)(i) = (robot_legs_[i]->calcPEe2B(leg_q).p - frame.p).Norm();
    }
    return q;
}

Vec12 QuadrupedRobot::getQd(const Vec12 &q, const Vec34 &vel, Vec4* sigma_min) {
    Vec12 qd = Vec12::Zero();
    for (int i(0); i < 4; ++i) {
        KDL::JntArray leg_q(3);
        leg_q.data = q.segment<3>(3 * i);
        const Mat3 jacobian = robot_legs_[i]->calcJaco(leg_q).data.topRows(3);
        if (sigma_min) (*sigma_min)(i) = jacobian.allFinite()
            ? jacobian.jacobiSvd().singularValues().minCoeff()
            : std::numeric_limits<double>::quiet_NaN();
        if (!jacobian.allFinite() || !vel.col(i).allFinite()) continue;
        Eigen::FullPivLU<Mat3> inverse(jacobian);
        inverse.setThreshold(1e-4);
        if (!inverse.isInvertible()) continue;
        const Vec3 leg_qd = inverse.solve(vel.col(i)); // 原公式 J * qd = v。
        if (leg_qd.allFinite()) qd.segment<3>(3 * i) = leg_qd;
    }
    return qd;
}

/* 运动学逆解，算出足底位姿 */
std::vector<KDL::Frame> QuadrupedRobot::getFeet2BPositions() const {
    std::vector<KDL::Frame> result;
    result.resize(4);
    for (int i = 0; i < 4; i++) {
        result[i] = robot_legs_[i]->calcPEe2B(current_joint_pos_[i]);// 运动学正解，计算身体坐标系下足底位置
    }
    return result;
}

KDL::Frame QuadrupedRobot::getFeet2BPositions(const int index) const {
    return robot_legs_[index]->calcPEe2B(current_joint_pos_[index]);
}

KDL::Jacobian QuadrupedRobot::getJacobian(const int index) const {// 这一定是身体坐标系下，计算雅可比矩阵需要输入当前关节位置
    return robot_legs_[index]->calcJaco(current_joint_pos_[index]);
}

KDL::JntArray QuadrupedRobot::getTorque(
    const Vec3 &force, const int index) const {
    return robot_legs_[index]->calcTorque(current_joint_pos_[index], force);
}

KDL::JntArray QuadrupedRobot::getTorque(const KDL::Vector &force, int index) const {
    return robot_legs_[index]->calcTorque(current_joint_pos_[index], Vec3(force.data));
}

KDL::Vector QuadrupedRobot::getFeet2BVelocities(const int index) const {// 
    const Mat3 jacobian = getJacobian(index).data.topRows(3);// 只取前三行线速度部分的雅可比矩阵
    Vec3 foot_velocity = jacobian * current_joint_vel_[index].data; // v = J * qd，速度的正解
    return {foot_velocity(0), foot_velocity(1), foot_velocity(2)};
}

std::vector<KDL::Vector> QuadrupedRobot::getFeet2BVelocities() const { // 也是身体坐标系下的足底速度
    std::vector<KDL::Vector> result;
    result.resize(4);
    for (int i = 0; i < 4; i++) {
        result[i] = getFeet2BVelocities(i);
    }
    return result;
}

void QuadrupedRobot::update() {
    if (mass_ == 0) return;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray pos_array(3);
        pos_array(0) = ctrl_interfaces_.joint_position_state_interface_[i * 3].get().get_value();
        pos_array(1) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 1].get().get_value();
        pos_array(2) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 2].get().get_value();
        current_joint_pos_[i] = pos_array;

        KDL::JntArray vel_array(3);
        vel_array(0) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3].get().get_value();
        vel_array(1) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 1].get().get_value();
        vel_array(2) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 2].get().get_value();
        current_joint_vel_[i] = vel_array;
    }
}


// 【新增实现】同时运动学正解所有四个足底坐标
KDL::Frame QuadrupedRobot::calcPEe2B_four_feet(const int index, const KDL::JntArray& q) const {
    // calcPEe2B函数：运动学正解，计算身体坐标系下足底位置
    KDL::Frame foot_frame = robot_legs_[index]->calcPEe2B(q);
    return foot_frame;
}
