# Robot Descriptions

This folder contains the URDF and SRDF files for the Sysu219 quadruped robot.

* Sysu219
    * [Sysu219](sysu219/sysu219_description/)

## 1. Steps to transfer urdf to Mujoco model

* Install [Mujoco](https://github.com/google-deepmind/mujoco)
* Transfer the mesh files to mujoco supported format, like stl.
* Adjust the urdf tile to match the mesh file. Transfer the mesh file from .dae to .stl may change the scale size of the
  mesh file.
* use `xacro` to generate the urdf file.
  ```
  xacro robot.xacro > ../urdf/robot.urdf
  ```
* use mujoco to convert the urdf file to mujoco model.
  ```
  compile robot.urdf robot.xml
  ```

## 2. Dependencies for Gazebo Simulation

Gazebo Simulation only tested on ROS2 Jazzy. It add support for ROS2 Humble because the package name is different.

* Gazebo Harmonic
  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```
* Ros2-Control for Gazebo
  ```bash
  sudo apt-get install ros-jazzy-gz-ros2-control
  ``` 
* Legged PD Controller
    ```bash
    cd ~/ros2_ws
    colcon build --packages-up-to leg_pd_controller
    ```

## 2. Dependencies for Gazebo Classic Simulation

Gazebo Classic (Gazebo11) Simulation only tested on ROS2 Humble.

* Gazebo Classic
  ```bash
  sudo apt-get install ros-humble-gazebo-ros
  ```
* Ros2-Control for Gazebo
  ```bash
  sudo apt-get install ros-humble-gazebo-ros2-control
  ``` 
* Legged PD Controller
    ```bash
    cd ~/ros2_ws
    colcon build --packages-up-to leg_pd_controller
    ```
