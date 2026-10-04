#!/usr/bin/env python3
"""Gazebo 专用：比较真实机身位置和当前 MPC 的估计质心近似。"""

from collections import deque
import math

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray


class GazeboComVisualizer(Node):
    def __init__(self):
        super().__init__('gazebo_com_visualizer')
        self.history_seconds = float(self.declare_parameter('history_seconds', 10.0).value)
        if not math.isfinite(self.history_seconds) or self.history_seconds <= 0.0:
            raise ValueError('history_seconds must be finite and positive')
        self.histories = {name: deque(maxlen=2000) for name in ('gazebo', 'estimated')}
        self.last_clock = None
        self.publisher = self.create_publisher(MarkerArray, '/com_markers', 1)
        self.truth_sub = self.create_subscription(
            Odometry, '/gazebo/body_ground_truth', self.on_truth, qos_profile_sensor_data)
        self.estimate_sub = self.create_subscription(
            PointStamped, '/com_estimated', self.on_estimate, qos_profile_sensor_data)
        self.timer = self.create_timer(0.04, self.publish_markers)

    def append(self, name, stamp, frame, point):
        if frame.lstrip('/') != 'world':
            return
        if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
            return
        seconds = stamp.sec + stamp.nanosec * 1e-9
        history = self.histories[name]
        if history and seconds < history[-1][0] - 0.2:
            # Gazebo reset：两种数据都清空，避免连接到上一次仿真轨迹。
            for samples in self.histories.values():
                samples.clear()
        if history and seconds <= history[-1][0]:
            return
        history.append((seconds, point))

    def on_truth(self, msg):
        self.append('gazebo', msg.header.stamp, msg.header.frame_id, msg.pose.pose.position)

    def on_estimate(self, msg):
        self.append('estimated', msg.header.stamp, msg.header.frame_id, msg.point)

    def publish_markers(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock:
            for samples in self.histories.values():
                samples.clear()
        self.last_clock = now
        markers = MarkerArray()
        colors = {'gazebo': (1.0, 0.45, 0.0), 'estimated': (0.0, 1.0, 1.0)}
        for name, samples in self.histories.items():
            while samples and samples[0][0] < now - self.history_seconds:
                samples.popleft()
            for marker_id, marker_type in ((0, Marker.SPHERE), (1, Marker.LINE_STRIP)):
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.ns = 'com_' + name
                marker.id = marker_id
                marker.type = marker_type
                marker.action = Marker.ADD if samples else Marker.DELETE
                marker.pose.orientation.w = 1.0
                marker.color.r, marker.color.g, marker.color.b = colors[name]
                marker.color.a = 1.0
                marker.lifetime.sec = 1
                if marker_type == Marker.SPHERE:
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.04
                    if samples:
                        marker.pose.position = samples[-1][1]
                else:
                    marker.scale.x = 0.005
                    marker.points = [point for _, point in samples]
                markers.markers.append(marker)
        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = GazeboComVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
