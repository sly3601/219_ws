# tools 调试工具集

存放 sysu219 调试用 GUI / 节点；和主控制器解耦，可以独立编译。

## 当前包含

两个 RQt 窗口工具：

- `joint_tuner`（插件名 `JointTuner`）：12 关节滑条调姿 + 左右腿批次同步 + 状态姿态切换 + 一键保存。
- `joint_state_viewer`（插件名 `JointStateViewer`）：**只读**双列显示 12 关节的实际位置与目标位置。

以及 JointTuner 可视化链路依赖的辅助节点：

- `joint_state_relay_node`：把 `/joint_tuner/joint_targets` 镜像成 `/joint_states`，供 `robot_state_publisher` 生成 TF。这是本包在 `setup.py` 里注册的唯一 console script。

> 中间话题 `/joint_tuner/joint_targets` 是 **tools 包私有话题**（`std_msgs/Float64MultiArray`），
> 刻意放在 `/joint_tuner/` 命名空间下，避免与控制器侧可能存在的全局命令话题（如
> `/joint_position_command`）撞名——两者语义不同，前者只是调参 GUI 给 RViz 预览用的目标角。
> 需要真正驱动控制器时，请把控制器接到这个私有话题上，不要改回全局名。

## 编译

```bash
cd ~/219_ws
colcon build --packages-select tools --symlink-install
source install/setup.bash
```

## 启动

### JointTuner —— 自带 RViz，不依赖仿真 / 实机

```bash
ros2 launch tools joint_tuner.launch.py
# 可选参数：
#   pkg_description=sysu219_description（默认）
#   base_height=0.35（RViz 里把 base 抬离地面的 z 偏移）
```

这条 launch 是**自包含**的，一次拉起整条可视化链路：

```
JointTuner RQt 窗口
    ↓ /joint_tuner/joint_targets (Float64MultiArray)
joint_state_relay 节点（tools 包）
    ↓ /joint_states (sensor_msgs/JointState)
robot_state_publisher
    ↓ TF
RViz RobotModel / TF
```

具体会启动：

| 节点 | 作用 |
| --- | --- |
| `robot_state_publisher` | 用 xacro 现场展开 sysu219，只读 `robot_description` + `/joint_states` |
| `joint_state_relay` | 命令镜像 |
| `static_transform_publisher` | 补 `world -> base`（xacro 根 link 是 `base`），z 由 `base_height` 控制，默认 0.35 |
| `rviz2` | **自动加载** `sysu219_description/config/visualize_urdf.rviz` |
| `rqt --force-discover --standalone JointTuner` | **直接弹出**调姿窗口，无需在 rqt 主壳里点菜单 |

所以**不需要先启动 Gazebo 或实机**——它自带 RViz 和描述文件，纯粹的可视化调姿。窗口里拖滑条只影响 RViz 里的模型，不会驱动电机。

> 独立预览的前提是系统里**没有**别的节点在发 `/joint_states`。如果实机/仿真的
> `joint_state_broadcaster` 也在跑，relay 会和它抢同一个话题，RViz 里的姿态会跳变——
> 这种场景请直接看 `joint_state_viewer`，不要用这条 launch。

用法：

1. 拖动 12 个滑条（或右侧数值框），每个滑条旁边的 `actual:` 显示当前 `/joint_states` 的实际值。
2. 用「状态切换」下拉框选 `FixedProne` / `FixedDown` / `FixedStand` 后点「切换」，滑条会直接设成参数文件里的对应姿态；选 `FreeStand` 则保留当前值，让你自由微调。
3. 「批量」区按 **前腿（FL+FR）/ 后腿（RL+RR）× hip/thigh/calf** 分成 6 组，拖动 master 滑条即把该组两条腿一起设置，不影响另两条腿。hip 在同一组内左右反号（填左腿的值，右腿自动取反），thigh/calf 左右同号。
4. 调出满意姿态后点「一键保存 (snapshot)」：
   - 终端会打印一段可直接贴到 `Sysu219GuideController` 姿态参数（`stand_pos` / `down_pos` / `prone_pos`）的 12 个浮点数。
   - YAML 快照写到 `~/219_ws/src/tools/config/joint_states/joint_snapshot_YYYYMMDD_HHMMSS.yaml`（路径在插件里写死，方便把快照提交进 git）。
   - 注意保存的是**滑条上的目标值**，不是 `/joint_states` 的实测值。

### JointStateViewer —— 只读，需要仿真 / 实机已经在跑

```bash
ros2 launch tools joint_state_viewer.launch.py
```

只弹一个 RQt 窗口，订阅系统里已有的 `/joint_states` 和 `/joint_cmd_states`，仿真与实机通用（前提是 `joint_state_broadcaster` 和 `sysu219_guide_controller` 在运行）。

「保存当前 12 关节实测位置到 YAML」写到 `~/219_ws/src/tools/config/joint_states/joint_snapshot_YYYYMMDD_HHMMSS.yaml`。

### 手动起 rqt 主壳

两个插件都注册在 `plugin.xml` 里，也可以自己起主壳：

```bash
ros2 run rqt_gui_py rqt_gui_py
# Plugins → Tools → Joint Tuner / Joint State Viewer
```

## 姿态参数来源

三个姿态预设**不再写死在代码里**。JointTuner 每次点「切换」都会重新读一遍 `sysu219_guide_controller` 的参数文件：

```yaml
sysu219_guide_controller:
  ros__parameters:
    stand_pos: [...]   # 站立
    down_pos:  [...]   # 半趴
    prone_pos: [...]   # 全趴
    stand_kp: 260.0
    stand_kd: 3.8
    prone_kp: 260.0
    prone_kd: 3.8
```

工具只读 `stand_pos` / `down_pos` / `prone_pos` 三个键（必须各 12 个值）。kp/kd 不由工具读取，列在这里只是为了和姿态对照。

所以改完 YAML **不用重启 rqt**，点一次「切换」就生效（状态栏会显示实际读的是哪个文件）。

参数文件查找顺序：

1. 环境变量 `JOINT_TUNER_POSE_CONFIG` 指定的文件
2. `sysu219_description` 包 `share/config/robot_control.yaml`
3. 源码树相对路径 `src/quadruped_ros2_control-humble/descriptions/sysu219/sysu219_description/config/robot_control.yaml`

三条都读不到时退回 `common.py` 里的内置默认值（与 `Sysu219GuideController.h` 的同名成员一致），工具仍能启动。

跑仿真时可以指向仿真那份配置：

```bash
export JOINT_TUNER_POSE_CONFIG=~/219_ws/src/quadruped_ros2_control-humble/descriptions/sysu219/sysu219_description/config/gazebo.yaml
```

## 话题约定

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/joint_states` | sensor_msgs/JointState | sub | 读回实际关节角用于显示 |
| `/joint_tuner/joint_targets` | std_msgs/Float64MultiArray | pub | JointTuner 目标，按 `JOINT_NAMES` 顺序；tools 包私有话题 |
| `/joint_cmd_states` | sensor_msgs/JointState | sub | 控制器发布的目标位置（JointStateViewer 第二列） |

## 与现有代码的耦合

- 关节顺序与 `descriptions/sysu219/.../config/robot_control.yaml::joints` 完全一致。
- 三个姿态参数直接读 `sysu219_guide_controller` 的参数文件，和控制器共用同一份来源。
- 关节限位与 [const.xacro](file:///home/yzz/219_ws/src/quadruped_ros2_control-humble/descriptions/sysu219/sysu219_description/xacro/const.xacro) 一致。

## 后续扩展

`tools` 目录预留为「调试工具集」专用，后续可以添加：
- foot marker / footprint 可视化
- estimator 调参 GUI
- controller 内部状态实时折线图
等等。所有新工具放到 `src/tools/tools/<your_tool>/` 子包中，并在 `setup.py::entry_points` 与 `plugin.xml` 中按需注册 RQt 插件或 console script。
