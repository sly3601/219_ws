# Sysu219 Guide Controller

This is a ros2-control controller for Sysu219. I used KDL for the kinematic and dynamic calculation, so
the controller performance has difference with the original one (sometimes very unstable).

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

[![](http://i1.hdslb.com/bfs/archive/310e6208920985ac43015b2da31c01ec15e2c5f9.jpg)](https://www.bilibili.com/video/BV1aJbAeZEuo/)

## 1. Interfaces

Required hardware interfaces:

* command:
    * joint position
    * joint velocity
    * joint effort
    * KP
    * KD
* state:
    * joint effort
    * joint position
    * joint velocity
    * imu sensor
        * linear acceleration
        * angular velocity
        * orientation

## 2. Build

```bash
cd ~/219_ws
colcon build --packages-up-to sysu219_guide_controller
```

## 3. Launch

### 3.1 Real Sysu219 Robot

Uses `config/hardware_config.yaml` (through `ros2_control.xacro`) as the hardware description and
`config/robot_control.yaml` as the controller manager parameter file.

```bash
source ~/219_ws/install/setup.bash
ros2 launch sysu219_guide_controller robot_hardware.launch.py pkg_description:=sysu219_description
```

### 3.2 Mujoco Simulation
> **Warm Reminder**: You need to launch the Mujoco C++ simulation before launch the controller.
```bash
source ~/219_ws/install/setup.bash
ros2 launch sysu219_guide_controller mujoco.launch.py pkg_description:=sysu219_description
```

### 3.3 Gazebo Classic 11 (ROS2 Humble)

Uses the `GAZEBO=true CLASSIC=true` branch of `robot.xacro`, which loads `config/gazebo.yaml`, and
additionally spawns `leg_pd_controller` because Gazebo only exposes effort interfaces.

```bash
source ~/219_ws/install/setup.bash
ros2 launch sysu219_guide_controller gazebo.launch.py pkg_description:=sysu219_description
```

### 3.4 Gazebo Harmonic (ROS2 Jazzy)

Gazebo Harmonic lives in the `gz_quadruped_playground` package, which spawns `sysu219_guide.launch.py`
(or `ocs2.launch.py` with `controller:=ocs2`) after starting Gazebo:

```bash
source ~/219_ws/install/setup.bash
ros2 launch gz_quadruped_playground gazebo.launch.py
```

## 4. FSM States and Key Mapping

The controller is driven by a state machine (`libraries/controller_common`). `PASSIVE` is the only state
that disables the motors; every other state holds the joints with the MIT gains from the parameter file.

| State | Meaning |
| --- | --- |
| `PASSIVE` | Motors disabled: torque / position / velocity commands zeroed, `kp = 0`, `kd = 1` |
| `FIXEDPRONE` | Fully lying down, target pose `prone_pos` |
| `FIXEDDOWN` | Half lying down, target pose `down_pos` |
| `FIXEDSTAND` | Fixed stand, target pose `stand_pos` |
| `FREESTAND` | Body pose controlled by the joystick, feet stay on the ground |
| `TROTTING` | Trot gait (open loop / closed loop switchable) |
| `SWINGTEST` | Single leg swing test |
| `BALANCETEST` | Foot force distribution test |
| `RLWALK` | Reinforcement learning policy walk |

Key mapping (`commands/keyboard_input`, same `command` ids as the gamepad nodes):

| Key | `command` | `PASSIVE` | `FIXEDPRONE` | `FIXEDDOWN` | `FIXEDSTAND` | `FREESTAND`/`TROTTING`/`SWINGTEST`/`BALANCETEST`/`RLWALK` |
| --- | --- | --- | --- | --- | --- | --- |
| `1` | 1 | — | `PASSIVE` | `PASSIVE` | `PASSIVE` | `PASSIVE` |
| `2` | 2 | `FIXEDPRONE` | `FIXEDDOWN` | `FIXEDSTAND` | `FIXEDDOWN` | `FIXEDSTAND` |
| `3` | 3 | — | — | — | `FREESTAND` | — |
| `4` | 4 | — | — | — | `TROTTING` | — |
| `5` | 5 | — | — | — | `SWINGTEST` | — |
| `6` | 6 | — | — | — | `BALANCETEST` | — |
| `7` | 7 | — | — | — | `RLWALK` | — |
| `8` | 8 | — | — | `FIXEDPRONE` | `FIXEDDOWN`, then `FIXEDPRONE` | — |

Notes:

* Key `1` always wins: it is checked before the pose-transition lock, so the robot can be disabled at any
  moment, including while a pose is still moving.
* Key `2` never disables the robot. It cycles the fixed poses:
  `PASSIVE -2-> FIXEDPRONE -2-> FIXEDDOWN -2-> FIXEDSTAND -2-> FIXEDDOWN -2-> FIXEDSTAND ...`
* Key `8` from `FIXEDSTAND` first goes to `FIXEDDOWN` and sets the `go_prone_after_down` flag, so
  `FIXEDDOWN` continues to `FIXEDPRONE` automatically once its own transition finishes.
* Pose transitions use a `tanh` ramp: `percent_` accumulates one step per control tick and
  `duration_ = update_rate * 1.2` steps. While `percent_ < 1.5` the state refuses to change
  (except for key `1`).

## 5. Fixed Pose Parameters

The fixed poses are **not hard-coded in the controller**. They are read from the
`sysu219_guide_controller` parameters, 12 joint angles each in the order
`FR/FL/RR/RL × (hip, thigh, calf)`, in radians:

```yaml
sysu219_guide_controller:
  ros__parameters:
    stand_pos: [...]   # FIXEDSTAND
    down_pos:  [...]   # FIXEDDOWN
    prone_pos: [...]   # FIXEDPRONE
    stand_kp: 260.0    # FIXEDDOWN / FIXEDSTAND share these
    stand_kd: 3.8
    prone_kp: 260.0    # FIXEDPRONE only
    prone_kd: 3.8
```

* Simulation (Gazebo Classic / Harmonic): `descriptions/sysu219/sysu219_description/config/gazebo.yaml`
* Real robot / Mujoco: `descriptions/sysu219/sysu219_description/config/robot_control.yaml`

The parameters are validated in `on_init`: each pose must contain exactly 12 values, otherwise
activation fails with an error log. To tune a pose, use the
[tools](../../../tools) package (`ros2 launch tools joint_tuner.launch.py`) or plot the actual and target
joint positions with `ros2 launch tools joint_state_viewer.launch.py` (the controller mirrors its joint
position commands on `/joint_cmd_states`).
