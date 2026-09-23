# Sysu219 Joystick Input Node

This node will listen to the wireless remote topic and publish a `control_input_msgs/Input` message by using `unitree_sdk2`.

> Before use this node, please use `ifconfig` command to check the network interface connected to the robot, then change the `network_interface` parameter in the launch file
> ([joystick.launch.py](launch/joystick.launch.py), default `enp46s0`). The `domain` parameter defaults to `0`.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy

### Build Command
```bash
cd ~/219_ws
colcon build --packages-up-to sysu219_joystick_input --symlink-install
```

### Launch Command
```bash
source ~/219_ws/install/setup.bash
ros2 launch sysu219_joystick_input joystick.launch.py
```

## 1. Use Instructions for the Sysu219 Guide Controller

The unitree remote buttons map to the `command` field of `control_input_msgs/msg/Inputs`:

| Buttons | command | Effect |
| --- | --- | --- |
| select | 1 | PASSIVE (motors disabled), always available |
| start | 2 | Cycle the fixed poses: PRONE → DOWN → STAND → DOWN → STAND → ... |
| right + B | 3 | FREE STAND (only from FIXED STAND) |
| right + A | 4 | TROTTING (only from FIXED STAND) |
| right + X | 5 | SWING TEST (only from FIXED STAND) |
| right + Y | 6 | BALANCE TEST (only from FIXED STAND) |
| left + B | 7 | RL WALK (only from FIXED STAND) |
| left + A | 8 | PRONE: from FIXED STAND it goes to DOWN first and then continues to PRONE automatically |
| left + X | 9 | Reserved, not handled by the guide controller |
| left + Y | 10 | Reserved, not handled by the guide controller |

* Starting from PASSIVE, the first press of `start` goes to PRONE, then DOWN, then STAND, and afterwards
  `start` toggles between DOWN and STAND.
* `select` switches to PASSIVE from any state, and is not blocked by the pose transition lock.
