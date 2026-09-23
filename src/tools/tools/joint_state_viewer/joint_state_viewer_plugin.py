"""JointStateViewer RQt 插件：实时显示 12 个关节的实际位置 + 目标位置。

设计：
- 同时订阅 /joint_states（joint_state_broadcaster 发布的实测角度）
  和 /joint_cmd_states（sysu219_guide_controller 内部目标位置镜像，250 Hz）。
- 每行一个关节：名字 + 实际 (rad) + 目标 (rad)。
- 4 条腿 (FR/FL/RR/RL) × 3 段位 (hip/thigh/calf) 排成 4×3 网格。
- 20Hz 刷新。
- "保存快照"按钮把当前 12 个目标位置（cmd_pos）写到
  ~/219_ws/src/tools/config/joint_states/joint_snapshot_YYYYMMDD_HHMMSS.yaml。
- 不向 ROS 话题发布（只读 + 本地文件）。

RQt 插件接口约束：
- 继承 rqt_gui_py.plugin.Plugin
- 通过 context.node 获取 rclpy 共享节点
- 通过 context.add_widget 注册控件
"""

import datetime
import os

from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from rqt_gui_py.plugin import Plugin
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from tools.common import (
    JOINT_NAMES,
    JOINT_STATES_TOPIC,
)


# 订阅 sysu219_guide_controller 内部发出的目标位置镜像
JOINT_CMD_STATES_TOPIC = '/joint_cmd_states'

# 一键保存默认目录（仓库内，方便 git 跟踪）
SNAPSHOT_DEFAULT_DIR = os.path.expanduser('~/219_ws/src/tools/config/joint_states')


class JointStateViewerPlugin(Plugin):
    """RQt 插件入口。"""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('JointStateViewerPlugin')

        self._node = context.node
        self._widget = JointStateViewerWidget(self._node)
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass


class JointStateViewerWidget(QWidget):
    """实际 UI 控件。"""

    # Signal: 由 rclpy 回调线程发出新状态；UI 线程接收后更新 labels
    _state_arrived = Signal(object, object)  # (actual_dict, cmd_dict)

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self._node = node
        self._actual = {name: 0.0 for name in JOINT_NAMES}
        self._cmd = {name: 0.0 for name in JOINT_NAMES}
        self._have_actual = False
        self._have_cmd = False
        self._msg_count = 0

        self._build_ui()

        # 订阅 /joint_states 和 /joint_cmd_states
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub_actual = self._node.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_actual_state, qos)
        self._sub_cmd = self._node.create_subscription(
            JointState, JOINT_CMD_STATES_TOPIC, self._on_cmd_state, qos)

        # 把 rclpy 回调线程的信号接到主线程槽函数
        self._state_arrived.connect(self._apply_joint_state)

        # 20Hz 刷新"已收消息数"等元信息
        self._timer = QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._refresh_meta)
        self._timer.start()

    # ---------- UI 构建 ----------
    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        # 标题
        title = QLabel('sysu219 关节实时显示 (订阅 /joint_states + /joint_cmd_states)')
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(11)
        title.setFont(title_font)
        outer.addWidget(title)

        # 元信息行
        meta_box = QHBoxLayout()
        self._status_label = QLabel('等待 /joint_states 和 /joint_cmd_states...')
        self._status_label.setStyleSheet('color: #888;')
        meta_box.addWidget(self._status_label)
        meta_box.addStretch(1)
        self._meta_label = QLabel('msgs: 0')
        self._meta_label.setStyleSheet('color: #555;')
        meta_box.addWidget(self._meta_label)
        outer.addLayout(meta_box)

        # 4 条腿 × 3 段位 网格
        grid_box = QGroupBox('关节实际位置 + 目标位置 (rad)')
        grid = QGridLayout(grid_box)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)

        leg_names = ['FR', 'FL', 'RR', 'RL']
        seg_names = ['hip', 'thigh', 'calf']

        # 表头
        header_font = QFont()
        header_font.setBold(True)
        for col, txt in enumerate(['关节', '实际 (rad)', '目标 (rad)']):
            lbl = QLabel(txt)
            lbl.setFont(header_font)
            grid.addWidget(lbl, 0, col)

        self._actual_labels = {}  # name -> QLabel 实际
        self._cmd_labels = {}    # name -> QLabel 目标

        for leg_row, leg in enumerate(leg_names):
            for seg_col, seg in enumerate(seg_names):
                joint_name = f'{leg}_{seg}_joint'
                row = leg_row * 4 + seg_col + 1  # +1 因为第 0 行是表头

                name_label = QLabel(joint_name)
                name_label.setMinimumWidth(110)
                grid.addWidget(name_label, row, 0)

                actual_label = QLabel('+0.000')
                actual_label.setMinimumWidth(90)
                actual_label.setStyleSheet('color: #0066cc; font-family: monospace;')
                grid.addWidget(actual_label, row, 1)
                self._actual_labels[joint_name] = actual_label

                cmd_label = QLabel('+0.000')
                cmd_label.setMinimumWidth(90)
                cmd_label.setStyleSheet('color: #cc6600; font-family: monospace;')
                grid.addWidget(cmd_label, row, 2)
                self._cmd_labels[joint_name] = cmd_label

        outer.addWidget(grid_box)
        outer.addStretch(1)

        # 一键保存快照行
        save_box = QHBoxLayout()
        self._save_button = QPushButton('保存当前 12 关节实测位置到 YAML')
        self._save_button.setMinimumHeight(32)
        self._save_button.clicked.connect(self._on_save_clicked)
        save_box.addWidget(self._save_button)
        save_box.addStretch(1)
        self._save_status = QLabel('')
        self._save_status.setStyleSheet('color: #555;')
        save_box.addWidget(self._save_status)
        outer.addLayout(save_box)

        # 说明
        tip = QLabel(
            '本工具只读 /joint_states（实测，joint_state_broadcaster 发）和 '
            '/joint_cmd_states（目标，sysu219_guide_controller 内部镜像），不会向任何 ROS 话题发布。\n'
            '蓝色 = 实际位置，橙色 = 目标位置。两值接近表示 PD 已收敛。\n'
            '点上面"保存"按钮把当前 12 个实测位置写到 '
            '~/219_ws/src/tools/config/joint_states/joint_snapshot_*.yaml。\n'
            '（保存的是 /joint_states 的实测角度，不是 cmd_pos。准备当 stand_pos_ 用时，'
            '先把机器人摆到目标姿态再保存。）'
        )
        tip.setStyleSheet('color: #666; font-size: 10pt;')
        tip.setWordWrap(True)
        outer.addWidget(tip)

    # ---------- rclpy 回调（spinner 线程） ----------
    def _on_actual_state(self, msg: JointState) -> None:
        state = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                state[name] = float(pos)
        if state:
            self._msg_count += 1
            self._have_actual = True
            self._state_arrived.emit(state, None)

    def _on_cmd_state(self, msg: JointState) -> None:
        state = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                state[name] = float(pos)
        if state:
            self._have_cmd = True
            self._state_arrived.emit(None, state)

    # ---------- 主线程槽 ----------
    def _apply_joint_state(self, actual, cmd) -> None:
        if actual is not None:
            self._actual.update(actual)
        if cmd is not None:
            self._cmd.update(cmd)
        for name in JOINT_NAMES:
            self._actual_labels[name].setText(f'{self._actual[name]:+.3f}')
            self._cmd_labels[name].setText(f'{self._cmd[name]:+.3f}')

    def _refresh_meta(self) -> None:
        if self._have_actual and self._have_cmd:
            self._status_label.setText('✓ 已订阅 /joint_states + /joint_cmd_states')
            self._status_label.setStyleSheet('color: #0a0;')
        elif self._have_actual:
            self._status_label.setText('✓ /joint_states  (等待 /joint_cmd_states...)')
            self._status_label.setStyleSheet('color: #aa0;')
        elif self._have_cmd:
            self._status_label.setText('✓ /joint_cmd_states  (等待 /joint_states...)')
            self._status_label.setStyleSheet('color: #aa0;')
        self._meta_label.setText(f'msgs: {self._msg_count}    rate: ~20 Hz')

    # ---------- 一键保存快照 ----------
    def _on_save_clicked(self) -> None:
        """保存当前 12 个关节实测位置到 YAML。"""
        if not self._have_actual:
            QMessageBox.warning(self, '无法保存', '尚未订阅到 /joint_states。')
            return

        values = [self._actual[name] for name in JOINT_NAMES]
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(SNAPSHOT_DEFAULT_DIR, f'joint_snapshot_{stamp}.yaml')
        try:
            os.makedirs(SNAPSHOT_DEFAULT_DIR, exist_ok=True)
            self._write_yaml(path, values)
        except Exception as e:
            QMessageBox.critical(self, '保存失败', str(e))
            return

        self._save_status.setText(f'✓ 已保存到 {path}')
        self._save_status.setStyleSheet('color: #0a0;')

    @staticmethod
    def _write_yaml(path: str, values) -> None:
        """手写 yaml（避免 PyYAML 依赖），字段名与 JointTuner 保存的快照保持一致。

        保存字段叫 joint_targets_rad，但内容是 /joint_states 中的"实测位置"（不是目标位置）。
        """
        with open(path, 'w', encoding='utf-8') as f:
            f.write('# sysu219 关节快照（JointStateViewer 一键保存）\n')
            f.write(f'# timestamp: {datetime.datetime.now().isoformat()}\n')
            f.write(f'# source_topic: /joint_states (joint_state_broadcaster 实测)\n')
            f.write('# 说明：这是机器人当前的实测关节角（实际位置），不是目标位置。\n')
            f.write('# 准备当 stand_pos_ 用时，先把机器人摆到目标姿态，再点保存。\n')
            f.write('joint_targets_rad:\n')
            for name, v in zip(JOINT_NAMES, values):
                f.write(f'  - {{name: {name}, position: {v:.6f}}}\n')
