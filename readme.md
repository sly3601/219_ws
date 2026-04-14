# 1. 代码运行方式
1. 编译
```
colcon build --packages-up-to unitree_guide_controller go1_description keyboard_input hardware_unitree_mujoco --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ --continue-on-error --cmake-args -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install
```
2. source一下资源目录
```
source install/setup.bash
```
3. 运行launch文件
```
ros2 launch unitree_guide_controller gazebo_classic.launch.py
```
4. 开启plotjugger进行数据可视化
```
LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 QT_QPA_PLATFORM=xcb LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib ros2 run plotjuggler plotjuggler
```


# 2. 四足机器人开发过程记录
## V1系列
**V1系列跑通了所有底层硬件通讯和初始机器人数学建模和姿态调试，并完成至troting状态的正常开环运行**
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
* 2026.02.06 v1.6
  * 硬件软件再次全部修好
* 2026.04.04 v1.7
  * 修改1：将整体控制系统频率降为500hz（首先在gazebo仿真中验证，与1000hz区别不大，可以正常行走）然而我发现之前其实早就修改成500hz了
  * 重大修改2：StateTrotting.cpp中加入一个标志位：troting_kalman，其值为1时使用卡尔曼滤波的位姿闭环，其值为0时则开环调试。
  * 重大修改3：passive到fixdown状态的平滑启动，不会再有冲击启动了。
  * 重大修改4：troting步态大量调整，加入大量的脱离estimator的开环troting代码，代码基本可以用了，理论上可以站在地上troting，但是实际上应该需要再对准一下电机绝对位置。
* 2026.04.05 v1.8
  * 修改1：优化了补全开环troting的代码
  * 修改2：将整个步态周期时长大幅提升，直接从0.65提升到5.65,发现了即使在过量冗余时间内，四条腿仍然是突然的超调运动，而不是平稳遵循足底摆线，排查问题
  * 重大修改3：第一次跑通开环troting！新加入了大量的开环troting内容。其中有大量需要优化的代码如自己写的二连杆的运动学逆解。
  * 明确了：关节最大扭矩的限幅处，就在HardwareUnitree.cpp文件的注释：修改限幅2026.02.06这里，我改大了一些。
  * 至此，V1系列完结，流程全部跑通。
 
 ## V2系列
**V2系列正在开发中**
* 2026.04.07 v2.01
  * 修复1：单圈第二编码器导致电机初始化移动位置舍近求远的问题（已实机验证）
* 2026.04.09 v2.02
  * 无实际功能修改，修改大量注释和冗余代码，troting开环整个算法运动学流程总结至飞书
* 2026.04.09 v2.03
  * 成功改掉所有的不标准正逆解,完成了很完美的纯位置的正逆解(还未考虑旋转).足端位置已经有明显的摆线运行轨迹
* 2026.04.12 v2.11
  * 尝试加入MarkerArray库进行轨迹可视化，成功，可在rviz中实时显示100个队列的实时最新的四个足底坐标
* 2026.04.13 v2.2
  * 机器人落地开环troting成功，进行了一定的kp kd调参，机器人基本troting完成了，开始闭环控制。
* 2026.04.13 v2.21
  * 开始闭环调试之前，最后补充了雅可比关节速度逆解,实机测试成功，正式开始闭环控制。
* 2026.04.13 v2.3
  * 完成了第一个闭环：roll/pitch闭环的实机调试。还需要调参数
* 2026.04.14 v2.4
  * 加入了一个新的FixedStandOffsetKeyboard.cpp，键盘控制节点，可以同时与原先的键盘控制节点同时开启，此节点只可以用于fixedstand模式！用于微调四足机器人的标准站立步态！