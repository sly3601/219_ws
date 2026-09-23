"""tools 包各插件 / 节点共用的关节列表与话题常量。

关节顺序与 sysu219_guide_controller 的 controller 配置一致：
    FR_hip_joint, FR_thigh_joint, FR_calf_joint,
    FL_hip_joint, FL_thigh_joint, FL_calf_joint,
    RR_hip_joint, RR_thigh_joint, RR_calf_joint,
    RL_hip_joint, RL_thigh_joint, RL_calf_joint

每个关节的 limit 与 descriptions/sysu219/.../xacro/const.xacro 中的 joint limit 一致。
"""

import os
from pathlib import Path

import yaml

JOINT_NAMES = [
    'FR_hip_joint',
    'FR_thigh_joint',
    'FR_calf_joint',
    'FL_hip_joint',
    'FL_thigh_joint',
    'FL_calf_joint',
    'RR_hip_joint',
    'RR_thigh_joint',
    'RR_calf_joint',
    'RL_hip_joint',
    'RL_thigh_joint',
    'RL_calf_joint',
]

# 各关节的默认角度范围 (rad)，参考 const.xacro 里的 position_max/min
JOINT_LIMITS = {
    'FR_hip_joint':   (-0.863,  0.863),
    'FR_thigh_joint': (-2.10,   4.501),
    'FR_calf_joint':  (-2.818, -0.888),
    'FL_hip_joint':   (-0.863,  0.863),
    'FL_thigh_joint': (-2.10,   4.501),
    'FL_calf_joint':  (-2.818, -0.888),
    'RR_hip_joint':   (-0.863,  0.863),
    'RR_thigh_joint': (-2.10,   4.501),
    'RR_calf_joint':  (-2.818, -0.888),
    'RL_hip_joint':   (-0.863,  0.863),
    'RL_thigh_joint': (-2.10,   4.501),
    'RL_calf_joint':  (-2.818, -0.888),
}

# ---------- 姿态参数：直接读 sysu219_guide_controller 的参数文件 ----------
# tuner / relay 里的三个姿态预设不在这里写死，而是读控制器实际使用的那份配置，
# 保证工具和控制器看到的姿态永远一致（改完参数文件重开或重切即可生效）。
#
# 查找顺序：
#   1. 环境变量 JOINT_TUNER_POSE_CONFIG 指定的文件
#   2. sysu219_description 包 share/config/robot_control.yaml
#   3. 源码树相对路径 <ws>/src/quadruped_ros2_control-humble/descriptions/sysu219/
#      sysu219_description/config/robot_control.yaml
# 三条都读不到时退回下面的内置默认值，保证工具仍能启动。

POSE_CONFIG_ENV = 'JOINT_TUNER_POSE_CONFIG'
POSE_CONFIG_BASENAME = 'robot_control.yaml'
POSE_PARAM_KEYS = ('stand_pos', 'down_pos', 'prone_pos')

# 兜底默认值，与 Sysu219GuideController.h 中的同名成员一致
_FALLBACK_POSES = {
    # 站立姿态（FIXEDSTAND）
    'stand_pos': [
        0.0, 0.9, -1.53,   # FR
        0.0, 0.9, -1.53,   # FL
        0.0, 0.9, -1.30,   # RR
        0.0, 0.9, -1.30,   # RL
    ],
    # 半趴姿态（FIXEDDOWN）
    'down_pos': [
        -0.054, 1.111, -2.155,  # FR
        0.054, 1.111, -2.155,   # FL
        -0.163, 0.999, -2.094,  # RR
        0.163, 0.999, -2.094,   # RL
    ],
    # 全趴姿态（FIXEDPRONE）
    'prone_pos': [
        -0.394, 1.599, -2.519,  # FR
        0.394, 1.599, -2.519,   # FL
        -0.435, 1.587, -2.490,  # RR
        0.435, 1.587, -2.490,   # RL
    ],
}

_NO_CONFIG_SOURCE = '<内置默认值>'


def _pose_config_candidates():
    """按优先级产出候选参数文件路径。"""
    override = os.environ.get(POSE_CONFIG_ENV)
    if override:
        yield Path(override)

    try:
        from ament_index_python.packages import get_package_share_directory
        yield (Path(get_package_share_directory('sysu219_description'))
               / 'config' / POSE_CONFIG_BASENAME)
    except Exception:
        pass

    # <ws>/src/tools/tools/common.py -> <ws>/src/quadruped_ros2_control-humble/...
    yield (Path(__file__).resolve().parents[2]
           / 'quadruped_ros2_control-humble' / 'descriptions' / 'sysu219'
           / 'sysu219_description' / 'config' / POSE_CONFIG_BASENAME)


def load_pose_config():
    """读三个姿态参数，返回 (poses, source)。

    poses 是 {'stand_pos': [...], 'down_pos': [...], 'prone_pos': [...]}，
    每个列表 12 个关节角。全部候选路径都读不到、或长度不是 12 时，
    返回内置默认值且 source 为 '<内置默认值>'。
    """
    for path in _pose_config_candidates():
        try:
            if not path.is_file():
                continue
            with open(path, encoding='utf-8') as f:
                params = yaml.safe_load(f)['sysu219_guide_controller']['ros__parameters']
            poses = {}
            for key in POSE_PARAM_KEYS:
                values = [float(v) for v in params[key]]
                if len(values) != len(JOINT_NAMES):
                    raise ValueError(
                        f'{key} 需要 {len(JOINT_NAMES)} 个值，实际 {len(values)} 个')
                poses[key] = values
            return poses, str(path)
        except Exception:
            continue
    return {k: list(v) for k, v in _FALLBACK_POSES.items()}, _NO_CONFIG_SOURCE


def load_state_presets():
    """重新读参数文件并返回 (预设表, source)。

    GUI 每次点“切换”时调用，这样改完参数文件不用重启 rqt。
    """
    poses, source = load_pose_config()
    return {
        "FixedProne": poses['prone_pos'],
        "FixedDown": poses['down_pos'],
        "FixedStand": poses['stand_pos'],
        "FreeStand": None,  # 保留当前 slider 值，让 GUI 自由微调机身位姿
    }, source


POSES, POSE_CONFIG_SOURCE = load_pose_config()

# 站姿（对应 StateFixedStand）
DEFAULT_STAND_POS = POSES['stand_pos']
# 半趴姿态（对应 StateFixedDown）
DEFAULT_DOWN_POS = POSES['down_pos']
# 全趴姿态（对应 StateFixedProne）
DEFAULT_PRONE_POS = POSES['prone_pos']

# JointTuner 状态切换表：键与 GUI 中下拉框显示名一致。
# 顺序与 2 键往返顺序一致：全趴 -> 半趴 -> 站立。
# None 表示“用 GUI 现状”，不是具体姿态。
STATE_PRESETS = {
    "FixedProne": DEFAULT_PRONE_POS,
    "FixedDown": DEFAULT_DOWN_POS,
    "FixedStand": DEFAULT_STAND_POS,
    "FreeStand": None,
}

# GUI / relay 默认使用的 ROS 话题
#
# JOINT_TARGETS_TOPIC 是 tools 包内部私有通道（JointTuner -> joint_state_relay），
# 刻意放在 /joint_tuner/ 命名空间下：它与控制器侧可能存在的全局命令话题
# （例如 /joint_position_command）语义完全不同，用私有名字可以彻底避免撞名。
# 需要真正驱动控制器时，请自行把控制器接到这个私有话题上，而不是改回全局名。
JOINT_STATES_TOPIC = '/joint_states'
JOINT_TARGETS_TOPIC = '/joint_tuner/joint_targets'
