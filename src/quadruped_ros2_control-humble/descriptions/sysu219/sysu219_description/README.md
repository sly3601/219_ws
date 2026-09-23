# Sysu219 Description

This repository contains the URDF model of Sysu219.

## Build

```bash
cd ~/219_ws
colcon build --packages-up-to sysu219_description  --symlink-install
```

## Visualize the robot

To visualize and check the configuration of the robot in rviz, simply launch:

```bash
source ~/219_ws/install/setup.bash
ros2 launch sysu219_description visualize.launch.py
```

## Launch ROS2 Control

### Real Sysu219 Robot

* Sysu219 Guide Controller
  ```bash
  source ~/219_ws/install/setup.bash
  ros2 launch sysu219_guide_controller robot_hardware.launch.py pkg_description:=sysu219_description
  ```

### Mujoco Simulator

* Sysu219 Guide Controller
  ```bash
  source ~/219_ws/install/setup.bash
  ros2 launch sysu219_guide_controller mujoco.launch.py pkg_description:=sysu219_description
  ```

### Gazebo Classic 11 (ROS2 Humble)

* Sysu219 Guide Controller
  ```bash
  source ~/219_ws/install/setup.bash
  ros2 launch sysu219_guide_controller gazebo.launch.py pkg_description:=sysu219_description
  ```

### Gazebo Harmonic (ROS2 Jazzy)

Gazebo Harmonic is provided by the `gz_quadruped_playground` package:

```bash
source ~/219_ws/install/setup.bash
ros2 launch gz_quadruped_playground gazebo.launch.py
```

## Config Files

The `config/` folder holds every parameter file used by the launch files above:

| File | Used by | Content |
| --- | --- | --- |
| `hardware_config.yaml` | real robot (`ros2_control.xacro`) | per-motor port / CAN id / direction / zero offset |
| `robot_control.yaml` | real robot, Mujoco | controller manager + controller parameters, including the fixed poses |
| `gazebo.yaml` | Gazebo (Classic and Harmonic) | same as `robot_control.yaml`, loaded by the Gazebo ros2-control plugin (`gazebo_ros2_control` / `gz_ros2_control`) |

The fixed poses (`stand_pos`, `down_pos`, `prone_pos`, `stand_kp`, `stand_kd`, `prone_kp`, `prone_kd`) must be kept
in sync between `robot_control.yaml` and `gazebo.yaml` — they are duplicated on purpose because Gazebo and the real
robot load different files.
