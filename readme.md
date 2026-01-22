* 该版本正式成为云端版本V1系列
* 2026.01.14 V1.1 完成了新硬件接口的初步对接！
* 2026.01.16 V1.2 补充了imu的硬件底层（未测试）
  *使用下面指令编译整个工程：
  colcon build   --packages-up-to unitree_guide_controller go1_description keyboard_input hardware_unitree_mujoco   --symlink-install   --cmake-args -DCMAKE_BUILD_TYPE=Release   --event-handlers console_direct+   --continue-on-error   --cmake-args -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install  # 仅加这行，确保路径统一
* 2026.01.17 V1.3 完全完成了imu和电机底层移植 在rviz中显示
* 2026.01.17 V1.4 周博然发现重大问题：热插拔接口之后串口顺序混乱。解决方法：不需要代码作任何修改，不要再热插拔了，插拔一定全部关机即可。但是功率电源可热插拔。
* 2026.01.22 V1.5 机器人可以正常固定站立并且踹不倒，制定不准再热插拔的规则
  * 重大修改1：使用以下命令开启plotjugger进行数据可视化：
  LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 QT_QPA_PLATFORM=xcb LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib ros2 run plotjuggler plotjuggler
  * 修改2：把所有ttyACM* 全部换成了ttyCAN* ，但是这并没有必要，是一次尝试USB接口热插拔的失败尝试，可忽略也可改回
  * 重大修改3：尝试改变fix stand状态的kp和kd，kp从80→100→120，机器人可以正常固定站立并且踹不倒
  * 发现问题：第一是imu零飘非常严重。第二是四足的挂钩在站立后自动脱落，有很大危险性，并且有毛刺会伤人。
