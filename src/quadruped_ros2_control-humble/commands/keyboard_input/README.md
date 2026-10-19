# Keyboard Input

This node will read the keyboard input and publish a control_input_msgs/Input message.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy
* Ubuntu 22.04
  * ROS2 Humble

### Build Command
```bash
cd ~/219_ws
colcon build --packages-up-to keyboard_input
```

### Launch Command
```bash
source ~/219_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```

## 1. Use Instructions for Sysu219 Guide

### 1.1 Control Mode
The keys map directly to the `command` field of `control_input_msgs/msg/Inputs`:

| Key | command | Effect |
| --- | --- | --- |
| 1 | 1 | PASSIVE (motors disabled), always available |
| 2 | 2 | Cycle the fixed poses: PRONE → DOWN → STAND → DOWN → STAND → ... |
| 3 | 3 | FREE STAND (only from FIXED STAND) |
| 4 | 4 | TROTTING (only from FIXED STAND) |
| 5 | 5 | SWING TEST (only from FIXED STAND) |
| 6 | 6 | BALANCE TEST (only from FIXED STAND) |
| 7 | 7 | RL WALK (only from FIXED STAND) |
| 8 | 8 | PRONE: from FIXED STAND it goes to DOWN first and then continues to PRONE automatically |
| 9 / 0 | 9 / 10 | Reserved, not handled by the guide controller |

* Starting from PASSIVE, the first press of `2` goes to PRONE, then DOWN, then STAND, and afterwards
  `2` toggles between DOWN and STAND.
* `1` switches to PASSIVE from any state, and is not blocked by the pose transition lock.

### 1.2 Control Input
* WASD IJKL: Move robot
* Space: command back to 0 and reset speed input

## 2. FixedStand Offset Keyboard

The same package also installs `fixedstand_offset_keyboard`, a small debug node that publishes a
per-joint offset on `/fixedstand_offset_cmd`. It is not started by any launch file.

```bash
ros2 run keyboard_input fixedstand_offset_keyboard
```

* `[` / `]`: select the current joint (0~11)
* `-` / `+`: decrease / increase the current joint offset by 0.01 rad
* `r`: clear the current joint offset

## 3. 腿部参数辨识专用键盘

```bash
ros2 run keyboard_input leg_identification_keyboard
```

`b` 从 PASSIVE 进入辨识，`w/e/r/t/y/u/a` 选择测试并自动先缓慢归位、确认到位，再正式开始；
`q` 可单独归位准备而不开始测试，
`h` 平滑回中，`1` 急停并保持当前位置和增益、不失能，`?` 查看帮助，`x` 或 Ctrl+C 先急停再退出键盘。
进入辨识后不能用此键盘切到其他 FSM。

本键盘终端按测试名称与次数显示：**开始准备 → 准备完成 → 正式开始 → 测试完成 → 数据记录完成**。
“数据记录完成”由记录线程确认对应样本已写入并刷新CSV，给出文件位置和正式段样本序号。
中途急停或换动作会显示“中止”，保留已有原始数据。“已发送请求”不代表动作已经开始。
辨识中心为 `[0, 0.5135, -1.605] rad`，原 FixedStand 参数不改。
正式髋目标只在0～37°（约0.645772 rad）耸肩侧往复，不进入负角溜肩侧；大腿范围为
`[-0.2865,1.3135] rad`，小腿范围为 `[-2.085,-1.125] rad`。
单关节20秒，联合40秒×3、验证40秒×2，静态仍15秒×3；全套正式段305秒，准备时间另计。
动作恢复原时间轴，时间倍率为1，目标速度/加速度阈值和准备到位判据保持原值，源数据仍按250Hz采集。
运动异常仅按明显越界（既有限位外约20°）或实测角速度超过10 rad/s判断，轻微偏差不直接中止。
异常、过渡失败、记录器故障或键盘离线超过2秒时，取消当前测试并保持左前腿，不清零增益。
