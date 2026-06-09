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
cd ~/ros2_ws
colcon build --packages-up-to sysu219_guide_controller
```

## 3. Launch

### 3.1 Mujoco Simulation
> **Warm Reminder**: You need to launch the Mujoco C++ simulation before launch the controller.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sysu219_guide_controller mujoco.launch.py pkg_description:=sysu219_description
```

### 3.2 Gazebo Classic 11 (ROS2 Humble)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sysu219_guide_controller gazebo_classic.launch.py pkg_description:=sysu219_description
```

### 3.3 Gazebo Harmonic (ROS2 Jazzy)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sysu219_guide_controller gazebo.launch.py pkg_description:=sysu219_description
```
