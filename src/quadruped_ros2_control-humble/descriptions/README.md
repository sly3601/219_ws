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

## 2. Dependencies for Gazebo Classic 11 Simulation

Gazebo Classic (Gazebo11) Simulation is used on ROS2 Humble.

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
    cd ~/219_ws
    colcon build --packages-up-to leg_pd_controller
    ```
* Launch
  ```bash
  ros2 launch sysu219_guide_controller gazebo.launch.py pkg_description:=sysu219_description
  ```

## 3. Dependencies for Gazebo Harmonic Simulation

Gazebo Harmonic Simulation is used on ROS2 Jazzy, and is launched through the `gz_quadruped_playground` package.

* Gazebo Harmonic
  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```
* Ros2-Control for Gazebo
  ```bash
  sudo apt-get install ros-jazzy-gz-ros2-control
  ```
* Launch
  ```bash
  ros2 launch gz_quadruped_playground gazebo.launch.py pkg_description:=sysu219_description
  ```
