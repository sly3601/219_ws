当前连续采样、参数版本留档与自动体检说明见 [RealTrialCheck.md](RealTrialCheck.md)。下文的起步/事件导出流程描述旧版本，用于解释旧 CSV；字段含义继续有效。新记录源码需用户自行更新实际运行程序后生效，代理不得执行编译。

统一开关为 `include/sysu219_guide_controller/debug/DebugConfig.h` 中的 `quadruped_debug::Csv_DebugMode`，当前为 `true`；改动后需要用户自行更新实际运行程序。
关闭时不创建记录线程/缓冲，不采集 CSV，不计算诊断用正解误差/SVD，不输出详细 MPC 日志；原有保底警告保留。
开启后 QP/MPC 共用 Trotting 诊断。每次进入 trot 无条件保存起步约 10 秒；之后突变触发保存名义前 2 秒、后 1 秒，退出时也保存最后一段。
后台线程生成 `/tmp/trotting_debug_<pid>_<timestamp>.csv`，终端输出 `[TROT_DEBUG] capture=startup/event saved=...`。
导出期间冻结这一个缓冲，完成后重新积累；记录力矩反馈和控制器命令，不修改控制命令。

CSV 的四腿顺序为 FR、FL、RR、RL；矩阵按列展开，每腿 xyz 或三个关节。
位置单位 m，角度 rad，速度 m/s 或 rad/s。`*_s` 是秒，`*_ms` 是毫秒。
`imu_0..3` 为原接口四元数顺序（当前为 wxyz），`imu_4..6` 为机身角速度，`imu_7..9` 为加速度。
`q_raw/qd_raw` 是限幅前逆解，`q_cmd/qd_cmd` 是最终关节目标。
力矩诊断列追加在原有 CSV 列之后，每组 12 列，关节顺序为 FR、FL、RR、RL，每腿 hip、thigh、calf；力矩单位 N·m：

- `tau_feedback`：effort 状态接口；实机为 CAN 回传的电机估算力矩，已转换为模型关节方向，不是实测足底接触力。缺失接口记 `nan`，不记零。仿真为仿真后端提供的 effort。
- `tau_ff_cmd`：控制器输出的前馈力矩，已经过控制器限幅，但尚未经过硬件 write 限幅和协议量化。
- `kp_cmd/kd_cmd`：本帧控制器输出的电机增益。
- `tau_p_est = kp_cmd × (q_cmd − q)`；`tau_d_est = kd_cmd × (qd_cmd − qd)`；`tau_mit_est` 为前馈与两项 PD 之和。均为控制器接口处的诊断估算，不代表电机实际总力矩。

这些列沿用控制器采样时刻。反馈可能对应此前发出的命令，也可能在相邻控制帧重复；不能把控制器采样频率当作 CAN 新反馈频率，不能要求反馈与同帧命令完全一致。`p/v` 是实际提供给控制器的估计；`acc_G` 和四腿反推速度继续保留。新日志删除旧 KF 的 `v_predicted/v_unfiltered`，旧归档仍可读取。

`touchdown_*` 是仅观察的落地候选诊断，不参与接触、增益或力分配。`touchdown_fz_proxy` 由 `-(R*J)^(-T)*tau_feedback` 得到，未扣除腿重力、惯性、摩擦或反馈延迟，不能当作实测脚力。缺失反馈、非有限值或病态雅可比记 NaN。

`touchdown_gates` 的位含义为：1=摆动相位0.85～1；2=相对当前支撑脚平均高度−5～25 mm；4=目标比反馈低超过8 mm且目标向下速度超过0.05 m/s；8=相对支撑脚平均竖直速度绝对值小于0.05 m/s；16=代理竖直力达到模型重量的15%（阈值见 `touchdown_fz_threshold_N`，本模型约60 N）。全部通过为31；有效连续帧累计12 ms后 `touchdown_candidate=1`。间断、复位、换相或任一条件失败均清除累计。`touchdown_candidate=1` 不证明真实着地，0也不证明未着地；这些阈值尚未验证可用于主动交接。

位置 IK 只做一次；`ik_q` 为返回码，兼容列 `ik_qd` 复制该返回码。
`fk_q` 为原始 IK 位置误差，`fk_qd` 为限位后关节角的位置误差；`sigma_qd` 为限位后速度逆解使用的雅可比最小奇异值。
`contact` 是计划支撑状态，不代表实际触地；`transition` 为步态切换标志。
`update_ms` 覆盖控制器 update 到快照采集完成，不含硬件 read/write 和缓冲写入。

`mode`：0=QP，1=MPC。`solver_id` 不变表示本周期未产生新的求解记录。
MPC 的 `solver_status` 为 HPIPM 原状态；QP 为 0=目标值和输出有限、1=非有限、-1=未返回。
`solver_result`：1=新解、2=缓存、0=保底/无成功记录；QP 的 `solver_iter=-1`。
`flags`：1=非法输入提前返回，2=原异常分支。

`event` 为位掩码，可以同时包含多项：

| 位值 | 变化 |
|---:|---|
| 1 | 时间间隔异常或系统时钟与单调时钟变化不一致 |
| 2 | 估计位置/速度突变 |
| 4 | 足端目标位置突变 |
| 8 | 限幅前关节位置/速度目标突变 |
| 16 | IK 新失败、正解误差或雅可比条件异常 |
| 32 | roll/pitch 超过诊断阈值 |
| 64 | 非有限值或异常分支 |
| 128 | 同一支撑状态下相位异常跳跃 |
| 256 | 退出时保存 |
| 512 | 进入 trot 后的第一帧，起步录制 |
| 1024 | 计划支撑状态切换，只标记，不单独触发导出 |

阈值只决定是否保存数据，不参与控制。起步录制不需要先攒够 2 秒。

仿真估计器对照（同一个 `Csv_DebugMode` 开关）：控制器仅在 `use_sim_time=true` 时创建 `/estimator_debug`，有订阅者时以 50 Hz 发布，覆盖 fixed stand、trot 等所有 FSM 状态。
Gazebo 可视化进程用已有 `/gazebo/body_ground_truth`（25 Hz）与时间最近的估计样本配对，时间差超过 25 ms 则跳过，实际时间差写入 `pair_dt_ms`。
它保存 `/tmp/estimator_debug_<pid>_<timestamp>.csv`，每秒或 FSM 状态改变时输出 `[EST_DEBUG]`，退出时关闭文件。关闭大量 debug 时不创建估计器发布器，Python 不创建这个 CSV。

新 CSV 的 `truth_p/truth_v` 是 Gazebo 机身真值；`est_p/estimated_v` 是最终送给控制器的估计位置/速度；`acc_G` 是去重力后的世界系 IMU 加速度，不是 InEKF 的独立真值。删除旧 KF 的预测速度和低通前速度。
`state`：4=FIXEDSTAND，6=TROTTING；`fsm_mode`：0=NORMAL，1=CHANGE。`truth_s/est_s` 都是仿真秒，不用系统时间配对。两次仿真 reset 之间分别分析，不能跨时间回退积分。

`/estimator_debug` 的 22 个元素依次为：仿真秒、FSM 状态、实际控制周期秒、在役估计位置 xyz、在役估计速度 xyz、去重力加速度 xyz、四腿计划 contact、四腿 phase、旧估计器内部固定步长秒、FSM mode。四腿均按 FR、FL、RR、RL 顺序。可视化脚本兼容旧的 28 元素消息，但不再导出旧 KF 速度。

接触影子（本轮新增源码）：`ContactShadowInEKF.hpp/.cpp` 独立封装力观测、接触状态机与记录，滤波数学复用另一个 `ShadowInEKF` 实例。红色实例仍是唯一在役状态来源；黄色实例没有控制结果接口，内部也禁止 `selectForControl` 写回。两者的滤波状态、模型、队列、线程和轨迹独立。主线程只复制固定长度快照，有界队列满时丢弃；黄色后台普通调度、nice=19，模型计算、文件写入和 DDS 发布都在后台。共享进程/CPU 仍有开销，实际调度时延必须在用户更新运行程序后验证，不能宣称严格实时隔离。

- 配置阶段读取 `contact_shadow_inekf_enabled`（默认 true）；设 false 完全不创建黄色实例，原红色路径继续使用。源码构建选项 `BUILD_CONTACT_SHADOW_INEKF=OFF` 可排除黄色 cpp 和调用，不改变原控制算法。本次不执行编译或启动。
- 黄色构造/接口绑定失败只停用黄色；后台模型失败单独重试；后台数值或持续创新异常由该实例自身故障锁存，终端给出原因，重新激活后恢复。不会驱动主 FSM 的里程计故障分支。关闭 CSV 用 `contact_shadow_write_csv=false`，仍保留黄色计算/显示。
- 现有硬件 `foot_force` 为空值，观测使用 **反馈力矩**，不使用命令力矩替代测量。广义动量残差扣除已有模型中的重力、惯性、摩擦项，再用足端雅可比反算世界竖直力；它仍受模型误差、反馈延迟和速度噪声影响，不是实测足底法向力。思路参考 [Bledt 等 Contact Model Fusion，ICRA 2018](https://dspace.mit.edu/handle/1721.1/120350)，本实现并非完整复现。
- 状态为 `0=Unknown,1=Air,2=Touchdown,3=Contact,4=Releasing,5=Unloaded`。`Unloaded` 只表示缺少载荷证据，不能当作几何离地或撤脚指令。进入接触需力滞回、持续时间、至少两次数值变化及相位/相对高度/相对速度门控；卸载只改变黄色观测状态，主回路支撑不变。无效力或时间断档不沿用旧的确认。
- `contact_shadow_force_on_N/off_N` 默认 25/12 N，`contact_shadow_confirm_on_s/off_s` 默认 0.012/0.020 s；仅为待实测标定的旁路起点，不是按体重缩放后已验证的配置。独立残差增益 80 s⁻¹、启动稳定期 0.12 s、最大连续间隔 0.04 s；初始稳定期不输出可信接触。没有设备时间戳/序号，数值变化不能证明传感器新鲜。
- 每次激活由黄色后台生成 `/tmp/contact_shadow_inekf_<pid>_<session>_<stamp_ns>.csv`，含原计划相位/接触、未补偿力代理、补偿后力估计、有效性、状态/原因/门控、累计确认时间、反馈变化数、雅可比条件、相对足高/速度、两套机身估计、q/qd/反馈力矩/残差及实际阈值。四腿 FR/FL/RR/RL，12 关节按每腿 hip/thigh/calf；力 N、力矩 N·m、位置 m、时间 s。两套估计都不是真机真值，`reference_ready=0` 时在役参考不作有效比较。
- `reason`：0复位、1候选、2确认、3候选取消、4计划卸载、5意外失载、6摆动抬脚、7无效力；记录最近一次状态转换原因。`gates` 位：1有效力、2近落地意图、4相对高度通过、8未明显上升、16反馈数值变化、32观测器稳定。`estimator_contact` 仅给黄色 InEKF，`support_authorized` 始终 0。
- 黄色发布 `/contact_shadow_inekf/{pose,twist,diagnostics,contact_diagnostics}`；姿态位置在 world，twist 在 `contact_shadow_inekf_base`，CSV 速度在 world。RViz 沿用 `/shadow_inekf/markers`，id=0 红色、id=1 黄色，旧浅蓝 KF 停止发布；两者只能删除自己的 marker。诊断话题给出队列丢帧/处理年龄/滤波故障，判定接触时应同时检查时间连续性、力源与故障日志。

新增通用一致性字段（Gazebo/实机都可用；均不参与控制）：

- `check_dt_s`：实际 ROS 帧间隔；0 表示差分不可用。首帧、时间回退或超过三个控制周期的间断不求差分，累计量重启。
- `qd_from_position`：关节角变化/实际间隔；`qd_error`：相邻两帧平均反馈速度减去角度差分。`q_delta/qd_integral`：该连续记录内角度净变化/反馈速度梯形积分。高频采样不足时不能仅凭两者不一致判传感器故障。
- `body_v_from_qd/body_v_from_position`：分别由速度接口、身体坐标系脚位置差分反推机身世界系速度。只在实际固定支撑足时成立，比较时筛选计划支撑中段，并用仿真实际接触/脚速度检查打滑。
- `goal_v_from_position/goal_v_error`：足端目标的实际差分/与相邻两帧平均速度目标之差。`end_v_from_position`：摆动终点重规划速度。相位切换帧不能作为连续摆动误差。
- `com_p`：MPC 当前采用的质心近似；`support_line_distance_m`：该点到两条计划支撑足连线的水平距离，非两足支撑时为 -1。
- `force_sum`：规划合力；`moment_vertical/moment_horizontal`：竖直/水平规划力相对 `com_p` 产生的世界系力矩。均为规划值，绝非实测接触力。

Gazebo 物理步记录：同一 `Csv_DebugMode` 开关，独立插件在每个物理步结束后只读采样，预分配缓冲和后台导出，不在物理回调写文件或逐帧打印。开启时订阅 Gazebo 原生接触信息，因此会增加接触信息采集开销；`sample_us` 记录插件采样耗时，控制器已有 `update_ms`。不改变物理参数、关节命令和控制算法。
每次进入 trot 保存之前约 2 秒和之后 10 秒，提前退出/关闭仿真也导出，文件为 `/tmp/physics_debug_<pid>_<timestamp>.csv`，终端有 `[PHYSICS_DEBUG] saved=...`。
CSV 四腿/关节顺序仍为 FR、FL、RR、RL。包含每物理步的 `q/qd`、真实机身位置/速度/姿态/角速度、按 Gazebo 所有 link 质量加权的真实整机 `com_p/com_v/mass`、四脚球心位置/速度、脚与环境实际接触点数及世界系接触力。脚球心不等于地面接触点，需考虑脚球半径。`contact_available=0` 时不根据零接触点数判断悬空。`sim_s` 为步末仿真时刻，`est_s` 是最近收到的估计器样本时间，不能把它当作每步同步真值。分析需按仿真时间对齐控制记录，并核对 0/±一个物理步的时间偏移。
容量上限 50000 帧；物理步长极小时可能保不满前 2 秒，使用文件的实际首尾时间。接触状态由 Gazebo 后端报告，仅用于诊断。

插件只在 `gazebo.launch.py` 注入，控制器本体无 Gazebo 依赖。构建机没有 Gazebo 时跳过；实机可用 `-DBUILD_GAZEBO_PHYSICS_DEBUG=OFF` 明确不构建。不把 Gazebo 真值喂给估计器/MPC/步态，不改变实机状态接口。

启动时 `[JOINT_DEBUG]` 打印实际位置/速度/力矩接口名称与索引，供实机/仿真核对。后台 `[TROT_CHECK]` 汇总关节积分差、连续摆动中的目标速度差和支撑线距离，阈值不参与控制。`[PHYSICS_CHECK]` 汇总站立段每物理步速度积分与角度净变化的平均差；`stand_s=0` 时没有站立证据，`gaps>0` 时积分证据不完整。`contact_s` 是该步接触记录中的最新后端时间，-1 表示没有接触记录；用它检查接触与位姿的时间偏移。

验证步骤：开启 `Csv_DebugMode` 后自行编译；进入 fixed stand 停留 5 秒，再进入 trot 运行 12 秒，退出至 fixed stand；再进入一次 trot 运行 12 秒，正常结束。失稳则提前退出也会保存，不必硬撑。提供终端日志和该次进程生成的三类 CSV：`estimator_debug`、`trotting_debug`、`physics_debug`。必须看到 `[PHYSICS_DEBUG] enabled step_ms=...`，若只有 plugin unavailable，则该次没有物理步证据。
