# Leg PD Controller

This package contains a simple PD controller for the leg joints of the quadruped robot. By using this controller, other ros2-control controllers based on position control can also work on the hardware interface which only contains effort control (for example, gazebo ros2 control).

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy
* Ubuntu 22.04
  * ROS2 Humble (Gazebo Classic 11)

## 1. Interfaces

It is a chainable controller: it exports the reference interfaces listed below and consumes the
`effort` command interface of the hardware.

Provided (reference) interfaces:
* joint position
* joint velocity
* joint effort
* KP
* KD

Required hardware interfaces:
* command:
  * joint effort
* state:
  * joint position
  * joint velocity

## 2. Build
```bash
cd ~/219_ws
colcon build --packages-up-to leg_pd_controller
```

## 3. Run

On Gazebo Classic 11 (ROS2 Humble) it is spawned automatically by
[sysu219_guide_controller/launch/gazebo.launch.py](../sysu219_guide_controller/launch/gazebo.launch.py):
```bash
ros2 launch sysu219_guide_controller gazebo.launch.py pkg_description:=sysu219_description
```
