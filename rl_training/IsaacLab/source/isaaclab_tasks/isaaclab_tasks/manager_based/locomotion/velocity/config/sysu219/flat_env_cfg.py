# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from .rough_env_cfg import SYSU219RoughEnvCfg

# 新增
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp


@configclass
class SYSU219FlatEnvCfg(SYSU219RoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # override rewards
        self.rewards.flat_orientation_l2.weight = -2.5
        self.rewards.feet_air_time.weight = 0.25

        # 原地踏步
        # self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        # self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        # self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        # 目标速度范围
        self.commands.base_velocity.ranges.lin_vel_x = (-0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.05, 0.05)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.5, 0.5)

        # 惩罚足端接触地面时的水平滑动
        self.rewards.feet_slide = RewTerm(
            func=mdp.feet_slide,
            weight=-0.5,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
                "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            },
        )

        # 电机扭矩过大惩罚
        self.rewards.dof_torques_l2.weight = -0.0001

        self.rewards.track_lin_vel_xy_exp.weight = 2.5  # xy 线速度跟踪奖励权重

        self.rewards.lin_vel_z_l2.weight = -3.7          # 机身 z 方向速度惩罚
        self.rewards.ang_vel_xy_l2.weight = -0.1         # 机身 roll/pitch 角速度惩罚
        self.rewards.flat_orientation_l2.weight = -1.0   # 机身姿态倾斜惩罚；越负越希望身体保持水平

        self.rewards.action_rate_l2.weight = -0.02  # 动作变化率惩罚
        self.rewards.dof_acc_l2.weight = -5.0e-7    # 关节加速度惩罚

        # 目标速度小于0.4时，要限制髋关节偏离默认角度。
        self.rewards.stand_still_hip_deviation = RewTerm(
            func=mdp.stand_still_joint_deviation_l1,
            weight=-0.8,
            params={
                "command_name": "base_velocity",
                "command_threshold": 1.0,  # 目标速度小于1.0时，才会惩罚髋关节偏离默认角度
                "asset_cfg": SceneEntityCfg("robot", joint_names=".*_hip_joint"),
            },
)





        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None


class SYSU219FlatEnvCfg_PLAY(SYSU219FlatEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
