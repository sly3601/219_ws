# Keyboard Input

This node will read the keyboard input and publish a control_input_msgs/Input message.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy
* Ubuntu 22.04
  * ROS2 Humble

### Build Command
```bash
cd ~/219_ws
colcon build --packages-up-to keyboard_input
```

### Launch Command
```bash
source ~/219_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```

## 1. Use Instructions for Sysu219 Guide

### 1.1 Control Mode
The keys map directly to the `command` field of `control_input_msgs/msg/Inputs`:

| Key | command | Effect |
| --- | --- | --- |
| 1 | 1 | PASSIVE (kp=0, kd=1 passive damping), always available |
| 2 | 2 | Cycle the fixed poses: PRONE → DOWN → STAND → DOWN → STAND → ... |
| 3 | 3 | FREE STAND (only from FIXED STAND) |
| 4 | 4 | TROTTING (only from FIXED STAND) |
| 5 | 5 | SWING TEST (only from FIXED STAND) |
| 6 | 6 | BALANCE TEST (only from FIXED STAND) |
| 7 | 7 | PPO in-place trot / RLWALK (only from FIXED STAND) |
| 8 | 8 | PRONE: from FIXED STAND it goes to DOWN first and then continues to PRONE automatically |
| 9 / 0 | 9 / 10 | Reserved, not handled by the guide controller |

* Starting from PASSIVE, the first press of `2` goes to PRONE, then DOWN, then STAND, and afterwards
  `2` toggles between DOWN and STAND.
* `1` switches to PASSIVE from any state, and is not blocked by the pose transition lock.
* Wait for each fixed-pose transition to finish (about 2 seconds) before pressing the next mode key.
* In RLWALK, `2` returns to FIXEDSTAND; `1` enters PASSIVE (passive damping, not power-off).
  To switch between RLWALK and TROTTING, first return to FIXEDSTAND with `2`, wait for its
  transition to finish, then press `7` for RLWALK or `4` for TROTTING.
* The in-place PPO ignores WASD/IJKL. Space does not exit RLWALK. Ctrl+C only ends the keyboard
  node and does not stop the controller's current mode; exit RL with `2` or `1` first.

### 1.2 Control Input
* WASD IJKL: Move robot
* Space: command back to 0 and reset speed input

## 2. FixedStand Offset Keyboard

The same package also installs `fixedstand_offset_keyboard`, a small debug node that publishes a
per-joint offset on `/fixedstand_offset_cmd`. It is not started by any launch file.

```bash
ros2 run keyboard_input fixedstand_offset_keyboard
```

* `[` / `]`: select the current joint (0~11)
* `-` / `+`: decrease / increase the current joint offset by 0.01 rad
* `r`: clear the current joint offset
