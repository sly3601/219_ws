该版本正式成为云端版本V1系列


* 2026.01.14 V1.1 完成了新硬件接口的初步对接！
* 2026.01.16 V1.2 补充了imu的硬件底层（未测试）
  *使用下面指令编译整个工程： colcon build --packages-up-to unitree_guide_controller go1_description keyboard_input hardware_unitree_mujoco --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ --continue-on-error --cmake-args -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install # 仅加这行，确保路径统一
* 2026.01.17 V1.3 完全完成了imu和电机底层移植 在rviz中显示
* 2026.01.17 V1.4 周博然发现重大问题：热插拔接口之后串口顺序混乱。解决方法：不需要代码作任何修改，不要再热插拔了，插拔一定全部关机即可。但是功率电源可热插拔。
* 2026.01.22 V1.5 机器人可以正常固定站立并且踹不倒，制定不准再热插拔的规则
  * 重大修改1：使用以下命令开启plotjugger进行数据可视化： LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 QT_QPA_PLATFORM=xcb LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib ros2 run plotjuggler plotjuggler
  * 修改2：把所有ttyACM* 全部换成了ttyCAN* ，但是这并没有必要，是一次尝试USB接口热插拔的失败尝试，可忽略也可改回
  * 重大修改3：尝试改变fix stand状态的kp和kd，kp从80→100→120，机器人可以正常固定站立并且踹不倒
  * 发现问题：第一是imu零飘非常严重。第二是四足的挂钩在站立后自动脱落，有很大危险性，并且有毛刺会伤人。
* 2026.01.24 v1.51 尝试了freestand状态，基本成功但是电流不足 机器人立不住。另外串口底层的库总是缺失导致串口初始化无法完成，最终解决方法都是与下方内容有关，具体也没多研究：
  * 强制将/usr/local/lib加到LD_LIBRARY_PATH最前面（覆盖ROS2的默认配置）
  * export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
* 2026.01.24 v1.53 发现了足底打滑的问题导致运动学解算出了问题，调大了2和3状态的kp值到160。
  * free stand状态调试完成，功能全部正常。
* 2026.01.24 v1.54 机器人报废。
  * 小腿连杆全部因冲击断裂。
  * 电机发热，但经测试，髋关节和大腿的电机仍然可正常工作，小腿电机生死未卜，但是大概是没有问题的。
  * 多根电源线/can信号线被拽断，有电路烧糊的味道，猜测是电源转接PCB板温度过高
  * troting状态的调试搁置，整理思路
