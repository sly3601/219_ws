# Quadruped ROS2 Control

**ROS2 Humble Branch**

This repository contains the ros2-control based controllers for the quadruped robot.

* [Controllers](controllers): contains the ros2-control controllers
* [Commands](commands): contains command node used to send command to the controller
* [Descriptions](descriptions): contains the urdf model of the robot
* [Hardwares](hardwares): contains the ros2-control hardware interface for the robot
* [Tools](../tools): standalone RQt debug tools (joint tuning / joint state viewer), built independently from the controller

Todo List:

- [x] **[2025-02-23]** Add Gazebo Playground
  - [x] OCS2 controller for Gazebo Simulation
  - [x] Refactor FSM and Sysu219 Guide Controller
- [x] **[2025-03-30]** Add Real Sysu219 Robot Support
- [ ] OCS2 Perceptive locomotion demo

Video on Real Sysu219 Robot:
[![](http://i0.hdslb.com/bfs/archive/7d3856b3c5e5040f24990d3eab760cf8ba4cf80d.jpg)](https://www.bilibili.com/video/BV1QpZaY8EYV/)

## 1. Quick Start

* rosdep
    ```bash
    cd ~/219_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
* Compile the package
    ```bash
    colcon build --packages-up-to sysu219_guide_controller sysu219_description keyboard_input --symlink-install
    ```

### 1.1 Mujoco Simulator or Real Sysu219 Robot
> **Warning:** CycloneDDS ROS2 RMW may conflict with unitree_sdk2, which is used by the
> [sysu219_joystick_input](commands/sysu219_joystick_input) node. If that node cannot reach the wireless
> remote without `sudo`, one of the below two methods could solve this conflict:
> 1. Uninstall CycloneDDS ROS2 RMW, used another ROS2 RMW, such as FastDDS **[Recommended]**.
> 2. Follow the guide in [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) to configure the ROS2 RMW by
>     compiling cyclone dds.

* Compile Sysu219 Hardware Interfaces
    ```bash
    cd ~/219_ws
    colcon build --packages-up-to hardware_sysu219
    ```
* Launch the ros2-control on the **real robot** (`robot_hardware.launch.py` loads
  `config/robot_control.yaml` plus `config/hardware_config.yaml` through `ros2_control.xacro`)
    ```bash
    source ~/219_ws/install/setup.bash
    ros2 launch sysu219_guide_controller robot_hardware.launch.py
    ```
* Launch the **Mujoco** simulation environment (the Mujoco C++ simulator is an external program, start it first)
    ```bash
    source ~/219_ws/install/setup.bash
    ros2 launch sysu219_guide_controller mujoco.launch.py
    ```
* Run the keyboard control node
    ```bash
    source ~/219_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

### 1.2 Gazebo Classic Simulator (ROS2 Humble)

* Install Gazebo Classic
  ```bash
  sudo apt-get install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control
  ```
* Compile Leg PD Controller (Gazebo only exposes effort interfaces, so the PD controller is required)
    ```bash
    colcon build --packages-up-to leg_pd_controller
    ```
* Launch the ros2-control
    ```bash
    source ~/219_ws/install/setup.bash
    ros2 launch sysu219_guide_controller gazebo.launch.py
    ```
* Run the keyboard control node
    ```bash
    source ~/219_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

### 1.3 Gazebo Harmonic Simulator (ROS2 Jazzy)

> Gazebo Harmonic support lives in [gz_quadruped_playground](libraries/gz_quadruped_playground) and is only tested on
> ROS2 Jazzy, since the `ros_gz` package name differs on Humble.

* Install Gazebo
  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```

* Compile Gazebo Playground
  ```bash
  colcon build --packages-up-to gz_quadruped_playground --symlink-install
  ```
* Launch the ros2-control
  ```bash
  source ~/219_ws/install/setup.bash
  ros2 launch gz_quadruped_playground gazebo.launch.py
  ```
* Run the keyboard control node
    ```bash
    source ~/219_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

For more details, please refer to the [sysu219 guide controller](controllers/sysu219_guide_controller/)
and [sysu219 description](descriptions/sysu219/sysu219_description/).

## What's Next
Congratulations! You have successfully launched the quadruped robot in the simulation. Here are some suggestions for you to have a try:
* **More Robot Models** could be found at [description](descriptions/)
* **Try more controllers**.
  * [Sysu219 Guide Controller](controllers/sysu219_guide_controller): FSM based controller with the fixed pose, free stand, trotting, swing test, balance test and RL walk states
  * [OCS2 Legged Robot Controller](libraries/ocs2_ros2/ocs2_robotic_examples/ocs2_legged_robot_ros): Robust MPC-based controller for quadruped robot
* **Simulate with more sensors**
  * [Gazebo Quadruped Playground](libraries/gz_quadruped_playground): Provide gazebo simulation with lidar or depth camera.
* **Debug and tune the robot**
  * [Tools](../tools): RQt joint tuning / joint state viewer, plus snapshot saving for the fixed poses.
* **Real Robot Deploy**
  * [Sysu219 Robot](descriptions/sysu219/sysu219_description): Check here about how to deploy on Sysu219.

## Reference

### Conference Paper

[1] Liao, Qiayuan, et al. "Walking in narrow spaces: Safety-critical locomotion control for quadrupedal robots with
duality-based optimization." In *2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp.
2723-2730. IEEE, 2023.

### Miscellaneous

[1] Unitree Robotics. *unitree\_guide: An open source project for controlling the quadruped robot of Unitree Robotics,
and it is also the software project accompanying 《四足机器人控制算法--建模、控制与实践》 published by Unitree
Robotics*. [Online].
Available: [https://github.com/unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)

[2] Qiayuan Liao. *legged\_control: An open-source NMPC, WBC, state estimation, and sim2real framework for legged
robots*. [Online]. Available: [https://github.com/qiayuanl/legged_control](https://github.com/qiayuanl/legged_control)

[3] Ziqi Fan. *rl\_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.*

2024. Available: [https://github.com/fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 
