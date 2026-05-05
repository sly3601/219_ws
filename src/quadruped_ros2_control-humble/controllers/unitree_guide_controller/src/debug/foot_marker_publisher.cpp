#include "unitree_guide_controller/debug/foot_marker_publisher.hpp"

namespace quadruped_controller
{

FootMarkerPublisher::FootMarkerPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: node_(std::move(node))
{
  // 创建发布者，话题名为 /foot_markers，QoS适合实时数据
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/foot_markers", 10);
  
  // 初始化Marker基础属性
  initMarkers();
  
  RCLCPP_INFO(node_->get_logger(), "足底Marker发布器已启动");
}

void FootMarkerPublisher::initMarkers()
{
  marker_array_.markers.clear();
  
  // 循环创建4个足底Marker（球体）
  for (size_t i = 0; i < 4; ++i) {
    visualization_msgs::msg::Marker marker;
    
    // 1. 基础配置
    marker.header.frame_id = "world"; // 【关键】足底坐标的参考系（根据你的工程修改）
    marker.ns = "foot_markers";           // 命名空间，避免与其他Marker冲突
    marker.id = static_cast<int32_t>(i);  // 唯一ID：0-FL, 1-FR, 2-RL, 3-RR
    marker.type = visualization_msgs::msg::Marker::SPHERE; // 用球体表示足底
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // 2. 位姿配置（初始位置设为原点，后续通过update更新）
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0; // 四元数必须初始化
    
    // 3. 大小配置（球体直径5cm，根据你的机器人大小调整）
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    
    // 4. 颜色配置
    marker.color.r = FOOT_COLORS[i][0];
    marker.color.g = FOOT_COLORS[i][1];
    marker.color.b = FOOT_COLORS[i][2];
    marker.color.a = FOOT_COLORS[i][3]; // 【关键】透明度必须>0，否则不可见
    
    // 5. 生命周期配置
    marker.lifetime = rclcpp::Duration(0, 0); // 永久显示
    marker.frame_locked = true; // 随参考系移动
    
    marker_array_.markers.push_back(marker);
  }

  // ========== 2. 新增：4条历史轨迹线Marker (ID 10-13) ==========
  for (size_t i = 0; i < 4; ++i) {
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "world";
    line_marker.ns = "foot_trajectory";
    line_marker.id = static_cast<int32_t>(10 + i);
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.005; // 线宽5mm
    
    line_marker.color.r = FOOT_COLORS[i][0];
    line_marker.color.g = FOOT_COLORS[i][1];
    line_marker.color.b = FOOT_COLORS[i][2];
    line_marker.color.a = 0.6; // 轨迹线半透明
    
    line_marker.lifetime = rclcpp::Duration(0, 0);
    line_marker.frame_locked = true;
    
    marker_array_.markers.push_back(line_marker);
  }
}

void FootMarkerPublisher::update(const std::array<geometry_msgs::msg::Point, 4> & foot_positions)
{
  // 先检查markers是否初始化
  if (marker_array_.markers.size() < 8) {
    return;
  }

  for (size_t i = 0; i < 4; ++i) {
    // ========== 1. 更新你原有的当前足底位置 (索引 0-3) ==========
    marker_array_.markers[i].pose.position = foot_positions[i];
    marker_array_.markers[i].header.stamp = rclcpp::Time(0);

    // ========== 2. 新增：更新历史轨迹 ==========
    foot_history_[i].push_back(foot_positions[i]);
    if (foot_history_[i].size() > HISTORY_LENGTH) {
      foot_history_[i].pop_front();
    }

    // ========== 关键修复：这里改成 4 + i (索引 4-7) ==========
    marker_array_.markers[4 + i].points.clear();
    for (const auto& point : foot_history_[i]) {
      marker_array_.markers[4 + i].points.push_back(point);
    }
    marker_array_.markers[4 + i].header.stamp = rclcpp::Time(0);
  }
}

void FootMarkerPublisher::publish()
{
  if (marker_array_.markers.empty()) {
    initMarkers();
  }
  marker_pub_->publish(marker_array_);
}

} // namespace quadruped_controller