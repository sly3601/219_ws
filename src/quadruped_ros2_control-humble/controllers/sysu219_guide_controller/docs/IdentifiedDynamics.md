# 冻结辨识模型的接入与力矩含义

本次只接入多刚体动力学模型。旧 QP、MPC 求解器、WBC 任务与松弛优化算法、
状态机、电机方向与零位、机身参数均不修改。接入时仅做源码修改和静态检查。
2026-10-21 用户已自行编译运行；13:29 的 Gazebo 日志确认 12 关节 B/Fc/Ja
已加载，并在混合比例 1.00 的 WBC 输出中使用。尚未定量证明控制效果改善或确认真机效果。

## 参数来源和唯一计算位置

`sysu219_description/xacro/const.xacro` 保存每类关节的非负 B、Fc、Ja：

| 关节 | B (N·m·s/rad) | Fc (N·m) | Ja (kg·m²) |
|---|---:|---:|---:|
| hip | 9.833942875227822e-10 | 1.0170625680080705 | 0 |
| thigh | 0.09591841174893605 | 1.2980958388469037 | 0 |
| calf | 1.6499038712067307e-09 | 1.0794450668988391 | 0.055704868111569605 |

`leg.xacro` 给每个实际关节输出标准 `<dynamics damping="B" friction="Fc"/>`，
以及工程自定义 `<identified_dynamics Ja="..."/>`。标准 URDF 不会自动使用 Ja。
`FloatingBaseModel` 从原始 `robot_description` 的关节 XML 读取 Ja，B/Fc 从同一
关节的标准 URDF dynamics 读取，按完整关节名映射到自身足端路径的关节索引。
XML 顺序、硬件配置中的电机排列顺序不决定参数索引。

模型初始化时缓存全部参数，检查缺失、重复、非有限值及负数，并打印
`[IDENTIFIED_DYNAMICS] joint=... velocity_index=... B=... Fc=... Ja=...`。
异常沿用现有模型初始化失败处理：`QuadrupedRobot` 打印 `[WBC_MODEL] disabled`，
完整模型不可用，上层按原有方式回退；本次没有改变该回退策略。

每周期先清零、重建刚体 M/C，再在 `FloatingBaseModel::update` 内加入一次：

\[
M_t=M_r+\operatorname{diag}(0_6,J_{a,1},\ldots,J_{a,12}),
\qquad
C_t=C_r+\begin{bmatrix}0_6\\B\odot\dot q_j+F_c\odot\operatorname{sign}(\dot q_j)\end{bmatrix}.
\]

这里 C 是 18 维偏置力向量，含重力、科氏/离心和新加入的摩擦需求。
当前实测关节速度决定摩擦方向；不使用目标速度来覆盖它。
`sign(0)=0`，没有 `0.05 rad/s` 的运行时开关，不另拟合静摩擦或平滑模型。
Ja 是常数质量矩阵对角附加项，本身不增加重力、科氏项。
控制周期不解析 XML、不查文件、不动态分配参数存储。

## 四腿方向为什么不额外乘镜像符号

URDF 中四腿 hip 轴均为 `(1,0,0)`，thigh/calf 轴均为 `(0,1,0)`。
已有刚体惯性镜像表达式继续负责几何和惯性转换。
B、Fc、Ja 是关节输出轴上的非负标量，不乘 `mirror` 或 `front_hind`。

硬件层 `correctMotorState` 和 `correctMotorCommand` 已完成：

\[
q_j=sq_m+b,\quad \dot q_j=s\dot q_m,\quad
\tau_{fb,j}=s\tau_{fb,m},\quad \tau_{cmd,m}=\tau_{cmd,j}/s,\quad s\in\{-1,+1\}.
\]

模型只使用已经转换后的关节速度，不再乘一次 s，也不再折算减速比。
同一坐标关系给出 `ddq_j=s*ddq_m`。因此：

\[
\frac{J_a\ddot q_j+B\dot q_j+F_c\operatorname{sign}(\dot q_j)}{s}
=J_a\ddot q_m+B\dot q_m+F_c\operatorname{sign}(\dot q_m).
\]

所以方向翻转由速度/加速度与硬件边界的转换自然处理。
摩擦作用在关节上的被动力矩为负的需求项，满足：

\[
\tau_{passive}\dot q_j=-B\dot q_j^2-F_c|\dot q_j|\le0.
\]

当前硬件配置的 s，按 hip/thigh/calf 排列：

| 腿 | s | 18 维小腿索引（从 0 开始） |
|---|---|---:|
| FR | (+1,-1,-1) | 8 |
| FL | (+1,+1,+1) | 11 |
| RR | (-1,-1,-1) | 14 |
| RL | (-1,+1,+1) | 17 |

这些核对证明源码与现有校准配置的坐标约定一致，不能证明真实接线、编码器反馈、
校准方向、延迟或低速换向行为在实机上一定正确。静态检查不能替代实机验证。

## 修正后的 tau 是什么

定义 `q,v` 为当前状态，`a_cmd` 为调用者要求实现的广义加速度，`f_cmd` 为其使用的
地面对机器人的接触力。逆动力学得到：

\[
\boxed{\tau_{ff,cmd}=\left[M_t(q)a_{cmd}+C_t(q,v)-J_c(q)^T f_{cmd}\right]_j.}
\]

它是“根据当前状态，为实现目标加速度和接触力，模型预测需要电机提供的前馈力矩”，
属于目标指令量。使用当前速度计算摩擦，不会把结果变成反馈力矩。
在保持相同 q、v、a_cmd、f_cmd 时，和刚体计算相比增加：

\[
\tau_{ff,cmd}=\tau_{rigid,cmd}+J_a\odot a_{j,cmd}
+B\odot\dot q_{j,meas}+F_c\odot\operatorname{sign}(\dot q_{j,meas}).
\]

实际调用者如果重新求解 a_cmd、f_cmd，最终力矩变化还会包含求解结果的变化。
例如小腿刚体需求 3 N·m、实测速度 +1 rad/s、要求加速度 +10 rad/s²，修正后
前馈约为 `3 + 1.07945 + 0.55705 = 4.63649 N·m`。

`tau_feedback / tau_est` 仍是电机上报并经硬件方向换算的反馈估计值；本次不修改它。
电机 MIT 模式还会按原有设定叠加位置/速度反馈控制，理想请求关系为：

\[
\tau_{motor,request}\approx\tau_{ff,cmd}
+K_p(q_{des}-q_{meas})+K_d(\dot q_{des}-\dot q_{meas}).
\]

执行前还经过原有混合、限值和回退路径；实际反馈力矩受驱动器与机构影响，
不能假定等于目标前馈或上述理想请求。不要把 B 填进 Kd，不要再往最终力矩或
硬件 write 加一次摩擦或 Ja。若用离线实测加速度代替 a_cmd，则公式得到的是
模型对该段实际运动的力矩预测，用途才变成离线评估。

## 不编译的检查

加载已有 ROS 和工作区环境后，运行纯静态检查（不会启动节点或机器人）：

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 \
  src/quadruped_ros2_control-humble/controllers/sysu219_guide_controller/scripts/check_identified_dynamics.py \
  --handoff '/mnt/hgfs/vmware共享文件夹/参数辨识交接文档.md'
```

检查四种 xacro 分支、12 个关节的名称/索引/轴、冻结参数、附件不重复计惯性、
当前硬件方向下的正负速度/加速度变换及被动摩擦耗散，并用已安装的 `check_urdf`
验证自定义 Ja 元数据兼容标准 URDF 解析。省略 `--handoff` 时仍可检查关节项，
该选项仅为额外比较全部冻结主总成惯性。

脚本不执行 C++，不代替 C++ 编译检查或实机验证。xacro 的 Ja 扩展目前只由
自写多刚体模型使用；Gazebo/MuJoCo 植物侧的 Ja 支持仍属于单独的后端适配。
