// Generated from the checkpoint's SAVED env.yaml and agent.yaml. Do not edit by hand.
#pragma once
#include <array>
#include <cstddef>

namespace sysu219_trot {
inline constexpr std::size_t kObservationDim = 45;
inline constexpr std::size_t kActionDim = 12;
inline constexpr const char* kContractSha256 = "2c115bc9c503a576eb5b4681b5440dacbf987e2502dc940eea2dfedd3b722b48";
inline constexpr const char* kModelFile = "policy_trot_ros_dr_7cm.onnx";
inline constexpr std::array<const char*, 12> kJointNames = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
inline constexpr std::array<double, 12> kDefaultPosition = {0, 0.90000000000000002, -1.53, 0, 0.90000000000000002, -1.53, 0, 0.90000000000000002, -1.3, 0, 0.90000000000000002, -1.3};
inline constexpr std::array<double, 12> kDefaultVelocity = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
inline constexpr std::array<double, 12> kActionScale = {0.080000000000000002, 0.20000000000000001, 0.25, 0.080000000000000002, 0.20000000000000001, 0.25, 0.080000000000000002, 0.20000000000000001, 0.25, 0.080000000000000002, 0.20000000000000001, 0.25};
inline constexpr std::array<double, 12> kTargetMin = {-0.16, 0.5, -2.0300000000000002, -0.16, 0.5, -2.0300000000000002, -0.16, 0.5, -1.8, -0.16, 0.5, -1.8};
inline constexpr std::array<double, 12> kTargetMax = {0.16, 1.3, -1.03, 0.16, 1.3, -1.03, 0.16, 1.3, -0.88800000000000001, 0.16, 1.3, -0.88800000000000001};
inline constexpr std::array<double, 12> kKp = {260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260, 260};
inline constexpr std::array<double, 12> kKd = {3.7999999999999998, 3.7999999999999998, 3.7999999999999998, 3.7999999999999998, 3.7999999999999998, 3.7999999999999998, 3.7999999999999998, 3.7999999999999998, 3.7999999999999998, 3.7999999999999998, 3.7999999999999998, 3.7999999999999998};
inline constexpr std::array<double, 12> kEffortLimit = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
inline constexpr double kActionClip = 2;
inline constexpr double kPolicyDt = 0.02;
inline constexpr double kPeriod = 0.97499999999999998;
inline constexpr double kSettle = 0.5;
inline constexpr double kRamp = 1;
inline constexpr double kAngularVelocityClipMin = -100;
inline constexpr double kAngularVelocityClipMax = 100;
inline constexpr double kAngularVelocityScale = 0.25;
inline constexpr double kGravityClipMin = -1;
inline constexpr double kGravityClipMax = 1;
inline constexpr double kGravityScale = 1;
inline constexpr double kJointPositionClipMin = -100;
inline constexpr double kJointPositionClipMax = 100;
inline constexpr double kJointPositionScale = 1;
inline constexpr double kJointVelocityClipMin = -100;
inline constexpr double kJointVelocityClipMax = 100;
inline constexpr double kJointVelocityScale = 0.050000000000000003;
inline constexpr double kLastActionClipMin = -2;
inline constexpr double kLastActionClipMax = 2;
inline constexpr double kLastActionScale = 1;
}  // namespace sysu219_trot
