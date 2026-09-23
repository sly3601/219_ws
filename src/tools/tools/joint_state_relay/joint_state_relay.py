"""joint_state_relay：把 GUI 的 12 个目标关节角映射为 /joint_states 与 TF。

用法：
    ros2 run tools joint_state_relay_node

订阅：
    /joint_tuner/joint_targets (std_msgs/Float64MultiArray)
        12 个目标角，按 sysu219 FR/FL/RR/RL × hip/thigh/calf 顺序
        （tools 包私有话题，与控制器侧的全局命令话题刻意区分开）

发布：
    /joint_states (sensor_msgs/JointState)
        robot_state_publisher 会用它生成 TF；RViz RobotModel/TF 即可看到机器人。

实现要点：
- 不接 Gazebo、不接 ros2_control，只是把 GUI 拖滑块的目标角直接当机器人当前角用，
  配合 robot_state_publisher 就能在 RViz 里观察姿态变化。
- 启动时按 DEFAULT_STAND_POS 初始化一次 /joint_states，避免 RViz 一开始关节全 0。
- 默认 50Hz 发布，保证 RViz 跟随丝滑。
- 注意：本节点只在"独立预览"场景使用。若实机/仿真的 joint_state_broadcaster
  也在发 /joint_states，两者会同时写同一话题，需要先停掉其中一个。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from tools.common import (
    JOINT_NAMES,
    DEFAULT_STAND_POS,
    JOINT_STATES_TOPIC,
    JOINT_TARGETS_TOPIC,
)


class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.declare_parameter('publish_rate_hz', 50.0)
        rate = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)

        self._latest_target = list(DEFAULT_STAND_POS)
        self._have_target = False

        # /joint_states 发布
        self._js_pub = self.create_publisher(
            JointState, JOINT_STATES_TOPIC, 10)

        # /joint_tuner/joint_targets 订阅
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(
            Float64MultiArray, JOINT_TARGETS_TOPIC, self._on_cmd, qos)

        # 周期性发布
        period = 1.0 / max(1.0, rate)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'joint_state_relay 启动，订阅 {JOINT_TARGETS_TOPIC}，发布 {JOINT_STATES_TOPIC}'
            f' @ {rate:.1f} Hz')

    def _on_cmd(self, msg: Float64MultiArray):
        if len(msg.data) != len(JOINT_NAMES):
            self.get_logger().warn_throttle(
                1000,
                f'joint_state_relay: 收到 {len(msg.data)} 个关节，应为 {len(JOINT_NAMES)}，忽略')
            return
        self._latest_target = [float(v) for v in msg.data]
        self._have_target = True

    def _on_timer(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = ''
        js.name = list(JOINT_NAMES)
        js.position = [float(v) for v in self._latest_target]
        js.velocity = [0.0] * len(JOINT_NAMES)
        js.effort = [0.0] * len(JOINT_NAMES)
        self._js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
