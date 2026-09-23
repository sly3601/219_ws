# Hardware Sysu219

This package contains the real hardware interface for the Sysu219 robot motors and IMU.

* [x] **[2025-01-16]** Add odometer states.

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
cd ~/219_ws
colcon build --packages-up-to hardware_sysu219 --symlink-install
```

## 3. Config hardware

The hardware plugin is declared in
[ros2_control.xacro](../../descriptions/sysu219/sysu219_description/xacro/ros2_control.xacro):

```xml
<hardware>
    <plugin>hardware_sysu219/HardwareSysu219</plugin>
    <param name="yaml_file_path">$(find sysu219_description)/config/hardware_config.yaml</param>
    <param name="imu_serial_port">/dev/ttyUSB0</param>
    <param name="imu_serial_baud">921600</param>
    <param name="debug">false</param>
    <param name="imu_frame_id">imu_link</param>
</hardware>
```

* `yaml_file_path`: DM motor configuration, resolved by `HardwareSysu219` via `parseDmActData`.
  See [hardware_config.yaml](../../descriptions/sysu219/sysu219_description/config/hardware_config.yaml),
  which holds the per-leg serial ports (baudrate) and, for every motor, its `can_id` / `master_id` /
  `type` / `direction` / `offset`.
* `imu_serial_port` / `imu_serial_baud`: serial device and baudrate of the IMU.
* `debug`: verbose logging switch of the hardware interface.
* `imu_frame_id`: frame id of the IMU.

After modifying the config, you can try to visualize the real robot info with the following command
(the IMU and DM motors must be connected, otherwise `on_init` fails):
```bash
source ~/219_ws/install/setup.bash
ros2 launch hardware_sysu219 visualize.launch.py pkg_description:=sysu219_description
```
