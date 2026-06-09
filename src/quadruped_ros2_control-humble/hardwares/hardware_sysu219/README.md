# Hardware Sysu219

This package contains the real hardware interface for the Sysu219 robot motors and IMU.

*[x] **[2025-01-16]** Add odometer states. 

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
  * foot force sensor

## 2. Build

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

Build Command:
```bash
cd ~/ros2_ws
colcon build --packages-up-to hardware_sysu219 --symlink-install
```

## 3. Config hardware
Set the Sysu219 hardware plugin parameters in the xacro file.
```xml
<hardware>
    <plugin>hardware_sysu219/HardwareSysu219</plugin>
    <param name="domain">1</param>
    <param name="network_interface">lo</param>
</hardware>
```

After modified the config, you can tried to visualize the robot info from real robot by following command:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hardware_sysu219 visualize.launch.py
```
