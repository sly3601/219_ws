#ifndef QUADRUPED_CONTROLLER__FOOT_MARKER_PUBLISHER_HPP_
#define QUADRUPED_CONTROLLER__FOOT_MARKER_PUBLISHER_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <array>
#include <string>
#include <memory>
#include <deque> // 新增：用于存储历史轨迹点

namespace quadruped_controller
{

class FootMarkerPublisher
{
public:
  // 构造函数：传入你的控制器节点指针
  explicit FootMarkerPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  
  // 更新4个足底的位置（输入：按顺序FL/FR/RL/RR的坐标）
  void update(const std::array<geometry_msgs::msg::Point, 4> & foot_positions);
  
  // 发布MarkerArray到RViz2
  void publish();

private:
  // 初始化4个足底Marker的基础属性（颜色、大小、类型等）
  void initMarkers();

  // 成员变量
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  visualization_msgs::msg::MarkerArray marker_array_;

  // ========== 新增：历史轨迹配置 ==========
  static constexpr int HISTORY_LENGTH = 100; // 保留最近100帧
  std::array<std::deque<geometry_msgs::msg::Point>, 4> foot_history_; // 每个足一个历史轨迹队列

  // 常量配置：4个足的名称和颜色（FR红/FL绿/RR蓝/RL黄）
  static constexpr std::array<const char *, 4> FOOT_NAMES = {"FR", "FL", "RR", "RL"};
  static constexpr std::array<std::array<float, 4>, 4> FOOT_COLORS = {{
    {1.0, 0.0, 0.0, 1.0}, // FR: Red
    {0.0, 1.0, 0.0, 1.0}, // FL: Green
    {0.0, 0.0, 1.0, 1.0}, // RR: Blue
    {1.0, 1.0, 0.0, 1.0}  // RL: Yellow
  }};
};

} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER__FOOT_MARKER_PUBLISHER_HPP_