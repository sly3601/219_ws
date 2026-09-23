"""Joint Tuner RQt 插件：拖动 12 个滑条把目标关节角写到 /joint_tuner/joint_targets。

设计：
- 12 个 QSlider，按 FR/FL/RR/RL × hip/thigh/calf 排成 4×3 网格。
- 每个 slider 旁边显示当前 GUI 目标值（rad）和从 /joint_states 读回来的实际值。
- 50Hz 把 12 个目标角发到 /joint_tuner/joint_targets（tools 包私有话题）；
  joint_state_relay 节点订阅后转成 /joint_states；robot_state_publisher 生成 TF；
  RViz 中看到机器人姿态。

RQt 插件接口约束：
- 继承 rqt_gui_py.plugin.Plugin
- 通过 context.node 获取 rclpy 共享节点
- 通过 context.add_widget 注册控件
"""

import os
from datetime import datetime

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from rqt_gui_py.plugin import Plugin
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from tools.common import (
    JOINT_NAMES,
    JOINT_LIMITS,
    DEFAULT_STAND_POS,
    STATE_PRESETS,
    POSE_CONFIG_SOURCE,
    load_state_presets,
    JOINT_STATES_TOPIC,
    JOINT_TARGETS_TOPIC,
)


# slider 整数 [0, SLIDER_RES) <-> [low, high] rad
SLIDER_RES = 1000


def _rad_to_slider(value, low, high):
    return int(round((value - low) / (high - low) * (SLIDER_RES - 1)))


def _slider_to_rad(slider_value, low, high):
    return low + slider_value * (high - low) / (SLIDER_RES - 1)


# 批量分组：每组两条腿是左右镜像关系
#   ref_leg    : 组内基准腿（取左腿），批量框里的数值就是这条腿的值
#   mirror_leg : 组内另一条腿
# hip 绕 x 轴，左右腿反号（如 FL +0.417 / FR -0.417）；
# thigh / calf 左右同号。
BATCH_GROUPS = [
    ('front', '前腿', 'FL', 'FR'),
    ('rear',  '后腿', 'RL', 'RR'),
]
# 需要随左右镜像反号的段位
MIRRORED_SEGS = {'hip'}


def _batch_leg_values(ref_leg, mirror_leg, seg, rad):
    """把批量框里的 rad 折算到组内两条腿，返回 [(joint_name, rad), ...]。"""
    mirrored = seg in MIRRORED_SEGS
    return [
        (f'{ref_leg}_{seg}_joint', rad),
        (f'{mirror_leg}_{seg}_joint', -rad if mirrored else rad),
    ]


class JointTunerPlugin(Plugin):
    """RQt 插件入口。"""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('JointTunerPlugin')

        self._node = context.node
        self._widget = JointTunerWidget(self._node)
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass


class JointTunerWidget(QWidget):
    """实际 UI 控件。"""

    _state_arrived = Signal(object)

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self._node = node
        self._actual = {name: 0.0 for name in JOINT_NAMES}
        self._suppress_slider_signal = False
        self._suppress_spin_signal = False

        self._sliders = []        # (name, slider, low, high)
        self._target_spins = []   # (name, spin)
        self._actual_labels = []  # (name, label)

        self._build_ui()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self._node.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_state, qos)
        self._pub = self._node.create_publisher(
            Float64MultiArray, JOINT_TARGETS_TOPIC, 10)

        self._state_arrived.connect(self._apply_joint_state)

        # 50Hz 发布目标
        self._timer = QTimer(self)
        self._timer.setInterval(20)
        self._timer.timeout.connect(self._publish_targets)
        self._timer.start()
        self._publish_targets()

    # ---------- UI 构建 ----------
    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        title = QLabel('sysu219 四足关节实时调姿 (RViz preview)')
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(11)
        title.setFont(title_font)
        outer.addWidget(title)

        grid_box = QGroupBox('关节控制 (rad)')
        grid = QGridLayout(grid_box)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)

        leg_names = ['FR', 'FL', 'RR', 'RL']
        seg_names = ['hip', 'thigh', 'calf']

        for leg_row, leg in enumerate(leg_names):
            for seg_col, seg in enumerate(seg_names):
                joint_name = f'{leg}_{seg}_joint'
                low, high = JOINT_LIMITS[joint_name]
                default = DEFAULT_STAND_POS[JOINT_NAMES.index(joint_name)]

                name_label = QLabel(joint_name)
                name_label.setMinimumWidth(110)
                grid.addWidget(name_label, leg_row * 4 + seg_col, 0)

                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(0)
                slider.setMaximum(SLIDER_RES - 1)
                slider.setValue(_rad_to_slider(default, low, high))
                slider.setMinimumWidth(220)
                slider.valueChanged.connect(self._make_slider_callback(joint_name))
                grid.addWidget(slider, leg_row * 4 + seg_col, 1)
                self._sliders.append((joint_name, slider, low, high))

                spin = QDoubleSpinBox()
                spin.setRange(low, high)
                spin.setDecimals(3)
                spin.setSingleStep(0.01)
                spin.setValue(default)
                spin.setMinimumWidth(80)
                # 关键：spinner 用 editingFinished，避免输入过程中频繁 setValue
                # 干扰 spin 的编辑状态（光标跳走、丢字符、卡住）。
                spin.editingFinished.connect(self._make_spin_callback(joint_name))
                grid.addWidget(spin, leg_row * 4 + seg_col, 2)
                self._target_spins.append((joint_name, spin))

                actual = QLabel('actual: 0.000')
                actual.setMinimumWidth(110)
                actual.setStyleSheet('color: #555;')
                grid.addWidget(actual, leg_row * 4 + seg_col, 3)
                self._actual_labels.append((joint_name, actual))

        outer.addWidget(grid_box)

        # 批量设置：前腿 (FL+FR) / 后腿 (RL+RR) × hip/thigh/calf，拖动即生效
        # 6 个 master slider：实时跟随 4×3 表里该组两条腿的「当前值」，
        # 拖动时把 4×3 表里对应 leg+seg 的两个 slider/spin 同步到 master 当前值。
        # 不影响另两条腿。hip 在同一组内左右反号，thigh/calf 同号。
        self._batch_segments = {}  # (group, seg) -> (slider, spin, low, high, ref_leg, mirror_leg)
        batch_box = QGroupBox(
            '批量：前/后两条腿 × hip/thigh/calf (拖动即应用, 显示当前值)'
            '；hip 左右反号, 数值填左腿的')
        batch_layout = QGridLayout(batch_box)
        batch_layout.setHorizontalSpacing(8)
        batch_layout.setVerticalSpacing(4)
        batch_layout.addWidget(QLabel('段位'), 0, 0)
        batch_layout.addWidget(QLabel('前腿 (FL+FR) 目标 (rad)'), 0, 1)
        batch_layout.addWidget(QLabel('数值 (前)'), 0, 2)
        batch_layout.addWidget(QLabel('后腿 (RL+RR) 目标 (rad)'), 0, 3)
        batch_layout.addWidget(QLabel('数值 (后)'), 0, 4)

        for col, seg in enumerate(['hip', 'thigh', 'calf']):
            row = col + 1
            low, high = JOINT_LIMITS[f'FR_{seg}_joint']

            batch_layout.addWidget(QLabel(seg), row, 0)

            for group_idx, (group_label, _cn, ref_leg, mirror_leg) in enumerate(
                    BATCH_GROUPS):
                col_off = 1 + group_idx * 2
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(0)
                slider.setMaximum(SLIDER_RES - 1)
                # 初值：取组内基准腿（左腿）在 DEFAULT_STAND_POS 中的角度，
                # 之后随 4×3 表的改动（点「切换」/「恢复」/拖单腿 slider）同步。
                seed_idx = JOINT_NAMES.index(f'{ref_leg}_{seg}_joint')
                slider.setValue(_rad_to_slider(DEFAULT_STAND_POS[seed_idx], low, high))
                slider.setMinimumWidth(180)
                batch_layout.addWidget(slider, row, col_off)

                spin = QDoubleSpinBox()
                spin.setRange(low, high)
                spin.setDecimals(3)
                spin.setSingleStep(0.01)
                spin.setValue(_slider_to_rad(slider.value(), low, high))
                spin.setMinimumWidth(70)
                batch_layout.addWidget(spin, row, col_off + 1)

                # 拖动 slider → 立即同步 4×3 表里该组两条腿并发布
                slider.valueChanged.connect(
                    self._make_batch_slider_callback(
                        ref_leg, mirror_leg, seg, low, high, spin))
                # 关键：批量 master spin 用 editingFinished，避免输入过程被打断
                # editingFinished 触发时把 spin 当前值应用到该组两条腿。
                spin.editingFinished.connect(
                    lambda v=spin, rl=ref_leg, ml=mirror_leg, sg=seg, sp=spin:
                    self._apply_batch_to_legs(
                        _batch_leg_values(rl, ml, sg, v.value()), sp))

                self._batch_segments[(group_label, seg)] = (
                    slider, spin, low, high, ref_leg, mirror_leg)

        outer.addWidget(batch_box)

        # 状态切换（对应 Sysu219GuideController FSM 中的几个状态）
        state_box = QGroupBox('状态切换 (Sysu219GuideController FSM)')
        state_layout = QHBoxLayout(state_box)
        self._state_combo = QComboBox()
        for name in STATE_PRESETS.keys():
            self._state_combo.addItem(name)
        self._state_combo.setCurrentText('FixedStand')
        state_layout.addWidget(QLabel('状态:'))
        state_layout.addWidget(self._state_combo, 1)
        switch_btn = QPushButton('切换')
        switch_btn.clicked.connect(self._on_switch_state)
        state_layout.addWidget(switch_btn)
        outer.addWidget(state_box)

        btn_box = QGroupBox('操作')
        btn_layout = QHBoxLayout(btn_box)
        reset_btn = QPushButton('恢复 default stand')
        reset_btn.clicked.connect(self._on_reset)
        zero_btn = QPushButton('全部归零')
        zero_btn.clicked.connect(self._on_zero)
        save_btn = QPushButton('一键保存 (snapshot)')
        save_btn.clicked.connect(self._on_save)
        btn_layout.addWidget(reset_btn)
        btn_layout.addWidget(zero_btn)
        btn_layout.addWidget(save_btn)
        btn_layout.addStretch(1)
        outer.addWidget(btn_box)

        self._status_label = QLabel(f'就绪 · 姿态参数：{POSE_CONFIG_SOURCE}')
        self._status_label.setStyleSheet('color: #0a0;')
        outer.addWidget(self._status_label)
        outer.addStretch(1)

    # ---------- rclpy 回调（spinner 线程） ----------
    def _on_joint_state(self, msg):
        state = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                state[name] = float(pos)
        if state:
            self._state_arrived.emit(state)

    # ---------- 主线程槽 ----------
    def _apply_joint_state(self, state):
        self._actual.update(state)
        for name, label in self._actual_labels:
            label.setText(f'actual: {self._actual[name]:+.3f}')

    # ---------- 交互回调 ----------
    def _make_batch_slider_callback(self, ref_leg, mirror_leg, seg, low, high, spin):
        """构造批量 master slider 的回调：拖动即更新 4×3 表里该组两条腿 + 发布。"""
        def _cb(slider_value):
            rad = _slider_to_rad(slider_value, low, high)
            self._apply_batch_to_legs(
                _batch_leg_values(ref_leg, mirror_leg, seg, rad), spin)

        return _cb

    def _apply_batch_to_legs(self, joint_values, spin):
        """把批量 master 当前值同步到 4×3 表里该组两条腿的 slider/spin + 发布。

        joint_values 是 [(joint_name, rad), ...]，hip 的左右反号由
        _batch_leg_values 折算好；第一个元素是基准腿，用来回写 master 的 spin。
        """
        # 同步 batch master 自己的 spin（不触发 editingFinished）
        spin.blockSignals(True)
        spin.setValue(joint_values[0][1])
        spin.blockSignals(False)

        # 同步 4×3 表里对应 leg+seg 的 slider/spin
        self._suppress_slider_signal = True
        self._suppress_spin_signal = True
        try:
            for name, rad in joint_values:
                for n, slider, lo, hi in self._sliders:
                    if n == name:
                        slider.setValue(_rad_to_slider(rad, lo, hi))
                        break
                for n, s in self._target_spins:
                    if n == name:
                        s.setValue(rad)
                        break
        finally:
            self._suppress_slider_signal = False
            self._suppress_spin_signal = False

        self._publish_targets()

    def _set_targets(self, targets):
        """把 12 个关节角同时写到所有 slider/spin。

        不会再发布到 /joint_tuner/joint_targets，由调用方决定是否触发
        _publish_targets()。
        """
        if len(targets) != len(JOINT_NAMES):
            raise ValueError(
                f'期望 {len(JOINT_NAMES)} 个关节角，得到 {len(targets)}')
        self._suppress_slider_signal = True
        self._suppress_spin_signal = True
        try:
            for i, name in enumerate(JOINT_NAMES):
                rad = float(targets[i])
                low, high = next((lo, hi) for n, _, lo, hi in self._sliders if n == name)
                for n, slider, _, _ in self._sliders:
                    if n == name:
                        slider.setValue(_rad_to_slider(rad, low, high))
                        break
                for n, spin in self._target_spins:
                    if n == name:
                        spin.setValue(rad)
                        break
        finally:
            self._suppress_slider_signal = False
            self._suppress_spin_signal = False

        # 同步批量 master：取组内基准腿（左腿）的当前值，与 4×3 表里那条腿一致
        self._sync_batch_masters(targets)

    def _sync_batch_masters(self, targets):
        """把 6 个批量 master 同步到 targets 中对应基准腿（左腿）的当前值。

        基准腿：前腿组取 FL，后腿组取 RL，与初始化时一致。
        """
        if not self._batch_segments:
            return
        name_to_value = dict(zip(JOINT_NAMES, targets))
        for (group, seg), (slider, spin, low, high, ref_leg, _ml) in \
                self._batch_segments.items():
            rad = name_to_value.get(f'{ref_leg}_{seg}_joint')
            if rad is None:
                continue
            slider.blockSignals(True)
            spin.blockSignals(True)
            try:
                slider.setValue(_rad_to_slider(rad, low, high))
                spin.setValue(rad)
            finally:
                slider.blockSignals(False)
                spin.blockSignals(False)

    def _on_switch_state(self):
        name = self._state_combo.currentText()
        # 每次切换都重新读一遍参数文件，改完 robot_control.yaml 不用重启 rqt
        presets, source = load_state_presets()
        targets = presets.get(name)
        if targets is None:
            # FreeStand：保留当前 GUI 上的关节角，不强切。
            self._set_status(f'切换到 {name}：保持当前关节角，拖滑块自由调整')
            self._publish_targets()
            return
        self._set_targets(targets)
        self._publish_targets()
        self._set_status(f'已切换到 {name} · 姿态参数：{source}')

    def _make_slider_callback(self, joint_name):
        def _cb(value):
            if self._suppress_slider_signal:
                return
            _, _, low, high = next(e for e in self._sliders if e[0] == joint_name)
            rad = _slider_to_rad(value, low, high)
            self._suppress_spin_signal = True
            for n, spin in self._target_spins:
                if n == joint_name:
                    spin.setValue(rad)
                    break
            self._suppress_spin_signal = False
            # 同步批量 master：让 (side, seg) 的 master 与这条腿对齐
            self._sync_batch_master_for_leg(joint_name, rad)
        return _cb

    def _make_spin_callback(self, joint_name):
        def _cb():
            # editingFinished 回调里没传 value，从 spin 自己读
            for n, spin in self._target_spins:
                if n == joint_name:
                    value = spin.value()
                    break
            else:
                return
            for n, slider, low, high in self._sliders:
                if n == joint_name:
                    self._suppress_slider_signal = True
                    slider.setValue(_rad_to_slider(value, low, high))
                    self._suppress_slider_signal = False
                    break
            self._sync_batch_master_for_leg(joint_name, value)
            self._publish_targets()
        return _cb

    def _sync_batch_master_for_leg(self, joint_name, rad):
        """把 batch master 同步到 4×3 表里指定 leg 的 rad。

        leg 在 batch master 的组里才更新（避免 user 拖单条非组内腿
        的 slider 时，master 显示「该 leg 自身」的值）。
        组内另一条腿（右腿）的 hip 与基准腿（左腿）反号，换算回基准腿坐标系。
        """
        if not self._batch_segments:
            return
        # 解析 (leg, seg)
        try:
            leg, seg = joint_name.split('_', 1)  # e.g. FL_thigh_joint -> ('FL', 'thigh_joint')
            seg = seg.rsplit('_', 1)[0]            # -> 'thigh'
        except ValueError:
            return
        for (group, seg_key), (slider, spin, low, high, ref_leg, mirror_leg) in \
                self._batch_segments.items():
            if seg_key != seg or leg not in (ref_leg, mirror_leg):
                continue
            value = -rad if (leg == mirror_leg and seg in MIRRORED_SEGS) else rad
            slider.blockSignals(True)
            spin.blockSignals(True)
            try:
                slider.setValue(_rad_to_slider(value, low, high))
                spin.setValue(value)
            finally:
                slider.blockSignals(False)
                spin.blockSignals(False)

    def _on_reset(self):
        self._suppress_slider_signal = True
        self._suppress_spin_signal = True
        for i, name in enumerate(JOINT_NAMES):
            rad = DEFAULT_STAND_POS[i]
            low, high = JOINT_LIMITS[name]
            for n, slider, _, _ in self._sliders:
                if n == name:
                    slider.setValue(_rad_to_slider(rad, low, high))
            for n, spin in self._target_spins:
                if n == name:
                    spin.setValue(rad)
        self._suppress_slider_signal = False
        self._suppress_spin_signal = False
        self._set_status('已恢复 default stand')

    def _on_zero(self):
        self._suppress_slider_signal = True
        self._suppress_spin_signal = True
        for _, slider, _, _ in self._sliders:
            slider.setValue(0)
        for _, spin in self._target_spins:
            spin.setValue(0.0)
        self._suppress_slider_signal = False
        self._suppress_spin_signal = False
        self._set_status('已全部归零')

    def _on_save(self):
        targets = self._current_targets()
        path = self._save_snapshot(targets)
        self._set_status(f'已保存到 {path}')

    # ---------- 内部工具 ----------
    def _current_targets(self):
        result = []
        for name in JOINT_NAMES:
            for n, spin in self._target_spins:
                if n == name:
                    result.append(spin.value())
                    break
        return result

    def _publish_targets(self):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self._current_targets()]
        self._pub.publish(msg)

    def _save_snapshot(self, targets):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        # 优先写入 ~/219_ws/src/tools/config/joint_states（仓库内可写，便于把快照提交进 git），
        # 同时在 tools 包的 data_files 中安装这个目录到 share/tools/config/joint_states，
        # 这样 install 后虽然 share 目录是只读的，但开发期直接用 --symlink-install 时是软链。
        # 也可显式传参覆盖。
        out_dir = os.path.expanduser('~/219_ws/src/tools/config/joint_states')
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f'joint_snapshot_{stamp}.yaml')

        with open(path, 'w', encoding='utf-8') as f:
            f.write('# sysu219 关节目标快照\n')
            f.write(f'# timestamp: {datetime.now().isoformat()}\n')
            f.write('joint_targets_rad:\n')
            for name, v in zip(JOINT_NAMES, targets):
                f.write(f'  - {{name: {name}, position: {v:.6f}}}\n')

        line = ', '.join(f'{v:.4f}' for v in targets)
        self._node.get_logger().info(f'快照已保存：{path}')
        self._node.get_logger().info('====== 可粘贴到 Sysu219GuideController 的 stand_pos_ ======')
        self._node.get_logger().info(f'    {line}')
        self._node.get_logger().info('========================================================')
        return path

    def _set_status(self, text, color='#0a0'):
        self._status_label.setText(text)
        self._status_label.setStyleSheet(f'color: {color};')