大量诊断默认关闭。统一开关为 `include/sysu219_guide_controller/debug/DebugConfig.h` 中的 `quadruped_debug::kLargeDebug`，改成 `true` 后重新编译开启。
关闭时不创建记录线程/缓冲，不采集 CSV，不计算诊断用正解误差/SVD，不输出详细 MPC 日志；原有保底警告保留。
开启后 QP/MPC 共用 Trotting 诊断。每周期写预分配内存；突变触发后保存名义前 2 秒、后 1 秒，退出时也保存最后一段。
后台线程生成 `/tmp/trotting_debug_<pid>_<timestamp>.csv`，终端输出 `[TROT_DEBUG] saved=...`。
导出期间冻结这一个缓冲，完成后重新积累；不记录力矩，不修改控制命令。

CSV 的四腿顺序为 FR、FL、RR、RL；矩阵按列展开，每腿 xyz 或三个关节。
位置单位 m，角度 rad，速度 m/s 或 rad/s。`*_s` 是秒，`*_ms` 是毫秒。
`imu_0..3` 为原接口四元数顺序（当前为 wxyz），`imu_4..6` 为机身角速度，`imu_7..9` 为加速度。
`q_raw/qd_raw` 是限幅前逆解，`q_cmd/qd_cmd` 是最终关节目标。
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

阈值只决定是否保存数据，不参与控制。`event` 标记所有候选异常；启动后先积累 2 秒，再自动触发导出。
