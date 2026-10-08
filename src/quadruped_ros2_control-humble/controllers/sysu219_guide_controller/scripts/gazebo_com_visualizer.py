#!/usr/bin/env python3
"""Gazebo 专用：比较真实机身位置和当前 MPC 的估计质心近似。"""

from collections import deque
import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray
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
        self.debug_estimates = deque(maxlen=64)
        self.debug_truth = deque(maxlen=64)
        self.debug_file = self.debug_writer = None
        self.debug_last_log = -math.inf
        self.debug_last_state = None
        self.debug_last_truth_s = None
        self.debug_sub = self.create_subscription(
            Float64MultiArray, '/estimator_debug', self.on_debug, qos_profile_sensor_data)
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
        if not self.debug_estimates:
            return  # Csv_DebugMode 关闭时没有估计器消息，跳过诊断处理。
        if msg.header.frame_id.lstrip('/') != 'world':
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        # 当前 p3d 插件在 frame_name=world 时直接输出 WorldLinearVel；无需再旋转。
        values = (p.x, p.y, p.z, v.x, v.y, v.z)
        if not all(math.isfinite(x) for x in values):
            return
        if self.debug_last_truth_s is not None and stamp < self.debug_last_truth_s:
            self.reset_debug_pairing()
        self.debug_last_truth_s = stamp
        self.debug_truth.append((stamp, values))
        self.write_debug_pairs()

    def on_estimate(self, msg):
        self.append('estimated', msg.header.stamp, msg.header.frame_id, msg.point)

    def reset_debug_pairing(self):
        self.debug_estimates.clear()
        self.debug_truth.clear()
        self.debug_last_log = -math.inf
        self.debug_last_state = None
        self.debug_last_truth_s = None

    def on_debug(self, msg):
        # C++ 数据顺序见 TrottingDebug.md；保留异常数值用于诊断。
        if len(msg.data) != 28 or not math.isfinite(msg.data[0]):
            return
        if self.debug_estimates and msg.data[0] < self.debug_estimates[-1][0]:
            self.reset_debug_pairing()
        self.debug_estimates.append(tuple(msg.data))
        self.write_debug_pairs()

    def write_debug_pairs(self):
        # 等估计数据推进到真值时刻，再取时间最近的样本；记录实际时间差。
        while self.debug_truth and self.debug_estimates:
            stamp, truth = self.debug_truth[0]
            if self.debug_estimates[-1][0] < stamp:
                break
            self.debug_truth.popleft()
            estimate = min(self.debug_estimates, key=lambda sample: abs(sample[0] - stamp))
            if abs(estimate[0] - stamp) > 0.025:
                continue  # 不把丢帧后的旧数据当成同一时刻的真值。
            if self.debug_file is None:
                path = f'/tmp/estimator_debug_{os.getpid()}_{time.time_ns() // 1000}.csv'
                self.debug_file = open(path, 'w', newline='', buffering=65536)
                self.debug_writer = csv.writer(self.debug_file)
                header = ['truth_s']
                for name in ('truth_p', 'truth_v'):
                    header.extend(f'{name}_{axis}' for axis in 'xyz')
                header.extend(('est_s', 'state', 'period_s'))
                for name in ('est_p', 'v_filtered', 'v_predicted', 'v_raw', 'acc_G'):
                    header.extend(f'{name}_{axis}' for axis in 'xyz')
                header.extend(f'contact_{leg}' for leg in ('FR', 'FL', 'RR', 'RL'))
                header.extend(f'phase_{leg}' for leg in ('FR', 'FL', 'RR', 'RL'))
                header.extend(('estimator_dt_s', 'fsm_mode', 'pair_dt_ms'))
                self.debug_writer.writerow(header)
                self.get_logger().info(f'[EST_DEBUG] saved={path}')
            pair_dt_ms = (estimate[0] - stamp) * 1000.0
            self.debug_writer.writerow((stamp, *truth, *estimate, pair_dt_ms))
            if stamp - self.debug_last_log >= 1.0 or estimate[1] != self.debug_last_state:
                state = {4: 'FIXEDSTAND', 6: 'TROTTING'}.get(estimate[1], str(estimate[1]))
                self.get_logger().info(
                    f'[EST_DEBUG] state={state} z[true,est]=[{truth[2]:.4f},{estimate[5]:.4f}] '
                    f'vz[true,pred,raw,filt]=[{truth[5]:.4f},{estimate[11]:.4f},'
                    f'{estimate[14]:.4f},{estimate[8]:.4f}] pair_dt_ms={pair_dt_ms:.1f}')
                self.debug_file.flush()
                self.debug_last_log, self.debug_last_state = stamp, estimate[1]

    def close_debug(self):
        if self.debug_file is not None:
            self.debug_file.close()
            self.debug_file = self.debug_writer = None

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
        node.close_debug()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
