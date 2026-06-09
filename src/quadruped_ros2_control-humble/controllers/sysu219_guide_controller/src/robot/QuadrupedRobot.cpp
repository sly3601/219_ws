//
// Created by biao on 24-9-12.
//

#include <iostream>
#include "controller_common/CtrlInterfaces.h"
#include "sysu219_guide_controller/robot/QuadrupedRobot.h"

QuadrupedRobot::QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                               const std::vector<std::string> &feet_names,
                               const std::string &base_name) : ctrl_interfaces_(ctrl_interfaces) {
    KDL::Tree robot_tree;
    kdl_parser::treeFromString(robot_description, robot_tree);

    robot_tree.getChain(base_name, feet_names[0], fr_chain_);
    robot_tree.getChain(base_name, feet_names[1], fl_chain_);
    robot_tree.getChain(base_name, feet_names[2], rr_chain_);
    robot_tree.getChain(base_name, feet_names[3], rl_chain_);


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

std::vector<KDL::JntArray> QuadrupedRobot::getQ(const std::vector<KDL::Frame> &pEe_list) const {
    std::vector<KDL::JntArray> result;
    result.resize(4);
    for (int i(0); i < 4; ++i) {
        result[i] = robot_legs_[i]->calcQ(pEe_list[i], current_joint_pos_[i]);
    }
    return result;
}

// 注意这个函数是强旋转约束
Vec12 QuadrupedRobot::getQ(const Vec34 &vecP) const {
    Vec12 q;
    for (int i(0); i < 4; ++i) {
        KDL::Frame frame;
        frame.p = KDL::Vector(vecP.col(i)[0], vecP.col(i)[1], vecP.col(i)[2]);
        frame.M = KDL::Rotation::Identity();
        q.segment(3 * i, 3) = robot_legs_[i]->calcQ(frame, current_joint_pos_[i]).data;
    }
    return q;
}

Vec12 QuadrupedRobot::getQd(const std::vector<KDL::Frame> &pos, const Vec34 &vel) {
    Vec12 qd;
    const std::vector<KDL::JntArray> q = getQ(pos);
    for (int i(0); i < 4; ++i) {
        Mat3 jacobian = robot_legs_[i]->calcJaco(q[i]).data.topRows(3);
        qd.segment(3 * i, 3) = jacobian.inverse() * vel.col(i); // 逆解速度：qd = J^-1 * v，只取前三行线速度
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