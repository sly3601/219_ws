# Joystick Input

This node will listen to the joystick topic and publish a control_input_msgs/Input message.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy
  * Logitech F310 Gamepad

```bash
cd ~/219_ws
colcon build --packages-up-to joystick_input
```

```bash
source ~/219_ws/install/setup.bash
ros2 launch joystick_input joystick.launch.py
```

## 1. Use Instructions for Sysu219 Guide

### 1.1 Control Mode

| Buttons | command | Effect |
| --- | --- | --- |
| LB + B | 1 | PASSIVE (motors disabled), always available |
| LB + A | 2 | Cycle the fixed poses: PRONE → DOWN → STAND → DOWN → STAND → ... |
| LB + X | 3 | FREE STAND (only from FIXED STAND) |
| LB + Y | 4 | TROTTING (only from FIXED STAND) |
| LT + B | 5 | SWING TEST (only from FIXED STAND) |
| LT + A | 6 | BALANCE TEST (only from FIXED STAND) |
| LT + X | 7 | RL WALK (only from FIXED STAND) |
| LT + Y | 8 | PRONE: from FIXED STAND it goes to DOWN first and then continues to PRONE automatically |
| START | 9 | Reserved, not handled by the guide controller |

* Starting from PASSIVE, the first press of `LB + A` goes to PRONE, then DOWN, then STAND, and afterwards
  `LB + A` toggles between DOWN and STAND.
* `LB + B` switches to PASSIVE from any state, and is not blocked by the pose transition lock.

### 1.2 Control Input

* Left / right sticks: Move robot
* No button pressed: `command = 0` and the stick values are published
