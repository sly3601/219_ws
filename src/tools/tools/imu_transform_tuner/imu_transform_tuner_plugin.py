"""IMU Transform Tuner RQt 插件：用「轴映射」调整 IMU 相对机身的朝向，实时发布 TF 到 RViz。

用途
----
IMU 通常按 90° 倍数安装在机身上，所以传感器系 -> 机身系只需做「轴的重排 + 符号」，
不需要任意角度的 roll/pitch/yaw。所有满足右手系（det=+1）的轴对齐映射一共有 24 种，
本插件把它们预先枚举好，用「一个按钮」循环切换，在 RViz 里实时预览四足模型
（发布 TF world -> base，让模型随 IMU 姿态旋转），对齐机身参考系后，
读出对应的轴映射矩阵和等效四元数，写回 imu.cpp。

数据流
------
    /imu/data_raw (sensor_msgs/Imu, 可配置)
        ↓ 取 orientation 作为原始朝向 q_raw（传感器系 -> 世界）
    按钮切换 -> 24 种右手系轴映射矩阵 M（机身 = M @ 传感器）
        ↓ q_M = M 的四元数；q_body = q_raw ⊗ conj(q_M)
    TF: world -> base  (q_body)
        ↓
    RViz RobotModel 显示四足模型并随 IMU 姿态旋转

无 IMU 数据时用单位四元数 (0,0,0,1) 预览，工具仍可离线调整方向。
"""

import itertools
import math
import os
import time

from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from rqt_gui_py.plugin import Plugin
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

import tf2_ros

# ---------- 环境变量可覆盖的默认配置 ----------
IMU_TOPIC = os.environ.get('IMU_TUNER_IMU_TOPIC', '/imu/data_raw')
PARENT_FRAME = os.environ.get('IMU_TUNER_PARENT_FRAME', 'world')
CHILD_FRAME = os.environ.get('IMU_TUNER_CHILD_FRAME', 'base')
BASE_HEIGHT = float(os.environ.get('IMU_TUNER_BASE_HEIGHT', '0.35'))


def _quat_multiply(p, q):
    px, py, pz, pw = p
    qx, qy, qz, qw = q
    return (
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
        pw * qw - px * qx - py * qy - pz * qz,
    )


def _quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q) if n > 1e-12 else (0.0, 0.0, 0.0, 1.0)


def _quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _det3(m):
    (a, b, c), (d, e, f), (g, h, i) = m
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def _matrix_to_quat(m):
    """把 3x3 旋转矩阵（det=+1）转成四元数 (x, y, z, w)。"""
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return (x, y, z, w)


def _build_right_handed_mappings():
    """枚举所有 24 种右手系（det=+1）的轴对齐映射，返回 [(描述, 3x3 矩阵), ...]。"""
    axes = ('X', 'Y', 'Z')
    result = []
    for perm in itertools.permutations(axes):
        for signs in itertools.product((1, -1), repeat=3):
            m = [[0.0] * 3 for _ in range(3)]
            for i, (axis, sign) in enumerate(zip(perm, signs)):
                m[i][axes.index(axis)] = float(sign)
            if _det3(m) > 0.5:
                desc = (
                    f"X←{'+' if signs[0] > 0 else '-'}{perm[0]}, "
                    f"Y←{'+' if signs[1] > 0 else '-'}{perm[1]}, "
                    f"Z←{'+' if signs[2] > 0 else '-'}{perm[2]}"
                )
                result.append((desc, m))
    return result


RIGHT_HANDED_MAPPINGS = _build_right_handed_mappings()


class ImuTransformTunerPlugin(Plugin):
    """RQt 插件入口。"""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('ImuTransformTunerPlugin')
        self._node = context.node
        self._widget = ImuTransformTunerWidget(self._node)
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass


class ImuTransformTunerWidget(QWidget):
    """实际 UI 控件。"""

    _imu_arrived = Signal(object)

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self._node = node
        self._raw_orientation = None  # (x, y, z, w)
        self._mapping_index = 0
        self._imu_rate = 0.0
        self._last_imu_t = None

        self._build_ui()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self._node.create_subscription(
            Imu, IMU_TOPIC, self._on_imu, qos)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self._node)

        self._imu_arrived.connect(self._apply_orientation)

        self._timer = QTimer(self)
        self._timer.setInterval(20)
        self._timer.timeout.connect(self._publish_tf)
        self._timer.start()
        self._publish_tf()

    # ---------- UI ----------
    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        title = QLabel('IMU 安装朝向调试（右手系轴映射，模型随 IMU 旋转）')
        f = QFont()
        f.setBold(True)
        f.setPointSize(11)
        title.setFont(f)
        outer.addWidget(title)

        switch_box = QGroupBox('切换轴映射（遍历全部 24 种右手系）')
        switch_layout = QVBoxLayout(switch_box)
        btn_row = QGridLayout()
        self._switch_btn = QPushButton('下一个')
        self._switch_btn.clicked.connect(self._on_switch)
        btn_row.addWidget(self._switch_btn, 0, 0)
        self._prev_btn = QPushButton('上一个')
        self._prev_btn.clicked.connect(self._on_prev)
        btn_row.addWidget(self._prev_btn, 0, 1)
        self._index_label = QLabel('1 / 24')
        btn_row.addWidget(self._index_label, 0, 2)
        switch_layout.addLayout(btn_row)
        self._desc_label = QLabel(RIGHT_HANDED_MAPPINGS[0][0])
        f2 = QFont()
        f2.setPointSize(12)
        self._desc_label.setFont(f2)
        switch_layout.addWidget(self._desc_label)
        outer.addWidget(switch_box)

        result_box = QGroupBox('结果（写回 imu.cpp 的轴映射 / 等效四元数）')
        res = QVBoxLayout(result_box)
        self._matrix_label = QLabel('映射矩阵 M (机身 = M @ 传感器):\n[1 0 0]\n[0 1 0]\n[0 0 1]')
        self._quat_label = QLabel('等效四元数 q_M (w, x, y, z): 1.000, 0.000, 0.000, 0.000')
        self._det_label = QLabel('右手系 (det=+1) · 轴映射有效')
        self._det_label.setStyleSheet('color: #0a0;')
        self._src_label = QLabel('IMU 源：等待中…')
        self._rate_label = QLabel('IMU 频率：0.0 Hz')
        res.addWidget(self._matrix_label)
        res.addWidget(self._quat_label)
        res.addWidget(self._det_label)
        res.addWidget(self._src_label)
        res.addWidget(self._rate_label)
        outer.addWidget(result_box)

        self._status_label = QLabel(
            f'就绪 · 订阅 {IMU_TOPIC} · 发布 TF {PARENT_FRAME} -> {CHILD_FRAME}')
        self._status_label.setStyleSheet('color: #0a0;')
        outer.addWidget(self._status_label)
        outer.addStretch(1)

    # ---------- rclpy 回调 ----------
    def _on_imu(self, msg):
        now = time.monotonic()
        if self._last_imu_t is not None:
            dt = now - self._last_imu_t
            if dt > 1e-6:
                self._imu_rate = self._imu_rate * 0.8 + (1.0 / dt) * 0.2
        self._last_imu_t = now
        q = (msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w)
        self._imu_arrived.emit(q)

    def _apply_orientation(self, q):
        self._raw_orientation = _quat_normalize(q)
        self._src_label.setText('IMU 源：已接收，实时跟随 /imu 方向')
        self._publish_tf()

    # ---------- 交互 ----------
    def _on_switch(self):
        self._mapping_index = (self._mapping_index + 1) % len(RIGHT_HANDED_MAPPINGS)
        self._publish_tf()

    def _on_prev(self):
        self._mapping_index = (self._mapping_index - 1) % len(RIGHT_HANDED_MAPPINGS)
        self._publish_tf()

    def _publish_tf(self):
        desc, m = RIGHT_HANDED_MAPPINGS[self._mapping_index]

        q_raw = self._raw_orientation if self._raw_orientation is not None \
            else (0.0, 0.0, 0.0, 1.0)

        q_m = _matrix_to_quat(m)
        q_body = _quat_normalize(_quat_multiply(q_raw, _quat_conj(q_m)))

        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = PARENT_FRAME
        t.child_frame_id = CHILD_FRAME
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = BASE_HEIGHT
        t.transform.rotation.x = q_body[0]
        t.transform.rotation.y = q_body[1]
        t.transform.rotation.z = q_body[2]
        t.transform.rotation.w = q_body[3]
        self._tf_broadcaster.sendTransform(t)

        self._index_label.setText(
            f'{self._mapping_index + 1} / {len(RIGHT_HANDED_MAPPINGS)}')
        self._desc_label.setText(desc)
        self._matrix_label.setText(
            '映射矩阵 M (机身 = M @ 传感器):\n'
            f'[{m[0][0]:+.0f} {m[0][1]:+.0f} {m[0][2]:+.0f}]\n'
            f'[{m[1][0]:+.0f} {m[1][1]:+.0f} {m[1][2]:+.0f}]\n'
            f'[{m[2][0]:+.0f} {m[2][1]:+.0f} {m[2][2]:+.0f}]')
        self._quat_label.setText(
            f'等效四元数 q_M (w, x, y, z): {q_m[3]:.4f}, {q_m[0]:.4f}, '
            f'{q_m[1]:.4f}, {q_m[2]:.4f}')
        self._rate_label.setText(f'IMU 频率：{self._imu_rate:.1f} Hz')
