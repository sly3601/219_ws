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
</hardware>
```

* `yaml_file_path`: configuration file for both the DM motors and the IMU, resolved by
  `HardwareSysu219`. See
  [hardware_config.yaml](../../descriptions/sysu219/sysu219_description/config/hardware_config.yaml),
  which holds:
  - `motor_ports` / `motors`: per-leg serial ports (baudrate) and, for every motor, its
    `can_id` / `master_id` / `type` / `direction` / `offset`.
  - `imu`: `serial_port` / `baudrate` / `frame_id` / `debug` of the IMU.

Fixed names `/dev/sysu219/ttyACM0..4` correspond to RR, FR, FL, RL and IMU.
Install [99-sysu219-serial.rules](99-sysu219-serial.rules) in `/etc/udev/rules.d/`
and reload the udev rules. Motor names follow the current USB sockets (keep these
connections unchanged); the IMU name follows its unique USB serial number.

After modifying the config, you can try to visualize the real robot info with the following command
(the IMU and DM motors must be connected, otherwise `on_init` fails):
```bash
source ~/219_ws/install/setup.bash
ros2 launch hardware_sysu219 visualize.launch.py pkg_description:=sysu219_description
```
