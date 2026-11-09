# 原地踏步 PPO 策略

来源：[sunjy1029/sysu219-rl，30f0ae8](https://github.com/sunjy1029/sysu219-rl/tree/30f0ae822c95729504103c4f7488c778d5a5a1bd)。仅引入以下三个 ONNX 和同名 JSON；JSON 保留训练来源、随机化范围及文件摘要。

| `rl_policy_id` | 模型文件名 | 训练变化与用途 |
| --- | --- | --- |
| `1` | `policy_trot_dr7cm_ar002_q010_dq030.onnx` | action_rate 惩罚权重由旧版的 -0.01 增至 -0.02，抑制相邻动作变化；扩大初始姿态、速度扰动，训练接管时的恢复能力。 |
| `2` | `policy_trot_dr7cm_ar003_q010_dq030.onnx` | 上述版本的对照策略，仅将 action_rate 权重增至 -0.03，更强调动作平滑。 |
| `3` | `policy_trot_dr7cm_ar002_delay008_q010_dq030.onnx` | 保持 -0.02，将动作、观测各自的额外延迟范围由 0～4 ms 扩至 0～8 ms，训练延迟适应能力。 |

三版初始关节角偏移均为 ±0.10 rad，初始关节速度为 ±0.30 rad/s。延迟步长为训练物理步长 2 ms，并非策略周期 20 ms。以上是训练设计目的，效果尚未经本工程仿真或真机验证。

三版与旧版的部署契约逐项一致：45 维观测、12 维动作、关节顺序、默认站姿、裁剪与缩放、50 Hz 策略频率、0.975 s 步态周期、kp=260、kd=3.8 均保持一致。7 cm 是训练足端高度参考，不在 ROS 侧叠加。此次仅修改模型选择入口，现有 RLWALK 的观测、推理、接管与执行流程及模型契约校验保持不变，也不向部署端加入训练噪声、延迟。

仿真和真机使用同一份 `sysu219_description/config/rl_policy.yaml`，当前保留用户选择 `rl_policy_id: 1`。切换时只需在这一处把标志改为 `1`、`2` 或 `3`，无需填写模型路径。RLWALK 的入口与时序参数也集中在该文件，观测、策略推理、动作映射和执行算法共用同一套代码，不按仿真/真机分支。`gazebo.yaml` 和 `robot_control.yaml` 不再分别设置 RL 参数。编号无效或模型加载失败时拒绝进入 RLWALK，不回退旧模型。旧 `rl_model_path` 参数已移除，旧模型文件仅保留归档，不参与选择。站稳后按原有按键 `7` 进入 RLWALK，`2` 返回 FIXEDSTAND，`1` 进入 PASSIVE。

真机和 MuJoCo launch 在基础配置后加载公共文件；Gazebo Classic 与 Gazebo Sim 的 ros2_control 插件同样加载该公共文件。插件的多个 `<parameters>` 文件入口由 [ROS 2 Control 官方文档](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)说明。

模型按编号从已安装控制器包的 `model/` 读取。此次没有修改 `install/`，未编译或启动仿真、真机；用户须自行更新实际运行的控制器程序及新模型安装文件后，新标志才生效。现有 CMake 安装规则已包含整个 `model/` 目录。模型在控制器激活、创建状态时加载；修改 YAML 不会使运行中的进程热切换，需重新读取配置并激活控制器。

训练实现依据：[奖励与变体配置](https://github.com/sunjy1029/sysu219-rl/blob/30f0ae822c95729504103c4f7488c778d5a5a1bd/rl_training/robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/quadruped/sysu219/trot_env_cfg.py)、[随机化范围](https://github.com/sunjy1029/sysu219-rl/blob/30f0ae822c95729504103c4f7488c778d5a5a1bd/rl_training/robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/quadruped/sysu219/domain_randomization.json)。
