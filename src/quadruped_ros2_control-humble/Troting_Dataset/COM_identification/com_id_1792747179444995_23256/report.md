# COM identification

[COM_ID][COMPUTED] 采集完整，离线计算成功；内部质量检查未通过。
失败检查数：17；参数状态：候选值，未独立验证。未自动修改模型。
数据目录：/home/sysu219/219_ws/src/quadruped_ros2_control-humble/Troting_Dataset/COM_identification/com_id_1792747179444995_23256
输出目录：/home/sysu219/219_ws/src/quadruped_ros2_control-humble/Troting_Dataset/COM_identification/com_id_1792747179444995_23256

## 未通过的质量检查

- reference: [MOTION_QUALITY] 正式段速度质量超限：joint=RR_calf_joint, |dq|max=0.034187317 rad/s (limit<=0.015), gyro_norm_max=0.005741677 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- pitch_positive: [MOTION_QUALITY] 正式段速度质量超限：joint=RL_thigh_joint, |dq|max=0.034187317 rad/s (limit<=0.015), gyro_norm_max=0.005952096 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- pitch_positive: [LOW_VALID_RATIO] 可用帧=26/34，低于80%质量参考；rejected_counts={'motion': 8, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- roll_positive: [MOTION_QUALITY] 正式段速度质量超限：joint=RL_calf_joint, |dq|max=0.034187317 rad/s (limit<=0.015), gyro_norm_max=0.005988736 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- roll_positive: [LOW_VALID_RATIO] 可用帧=25/34，低于80%质量参考；rejected_counts={'motion': 9, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- pitch_negative: [MOTION_QUALITY] 正式段速度质量超限：joint=FL_hip_joint, |dq|max=0.034187317 rad/s (limit<=0.015), gyro_norm_max=0.006756127 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- pitch_negative: [LOW_VALID_RATIO] 可用帧=24/34，低于80%质量参考；rejected_counts={'motion': 10, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- roll_negative: [MOTION_QUALITY] 正式段速度质量超限：joint=FR_hip_joint, |dq|max=0.024421692 rad/s (limit<=0.015), gyro_norm_max=0.005170314 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- roll_negative: [LOW_VALID_RATIO] 可用帧=26/34，低于80%质量参考；rejected_counts={'motion': 8, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- reference_repeat: [MOTION_QUALITY] 正式段速度质量超限：joint=RR_hip_joint, |dq|max=0.034187317 rad/s (limit<=0.015), gyro_norm_max=0.005267794 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- reference_repeat: [LOW_VALID_RATIO] 可用帧=24/34，低于80%质量参考；rejected_counts={'motion': 10, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- [MASS_CONSISTENCY] 跨姿态质量标准差/整机质量=0.080392 > 0.05；检查静摩擦、力矩标定、模型或接触。
- [MOMENT_FIT] 跨姿态重力矩拟合RMS=11.104194 N·m > 2 N·m。
- [REPEATABILITY] 回中最大差异=0.044586203 m > 0.015 m；存在方向相关摩擦、漂移或反馈偏差。
- [LEAVE_ONE_OUT] 留一姿态重算最大XY变化=0.029255004 m（限值0.015 m），最大Z变化=0.276138278 m（限值0.04 m）；至少一项超限。
- [COM_MAGNITUDE] 躯干质心最大绝对坐标=0.944916 m > 0.5 m。
- [FRICTION_SENSITIVITY] Fc尺度敏感性Z=0.038514238 m > 0.03 m；该尺度不是实测静摩擦上界。

## 各姿态诊断

raw=CSV正式段帧数；selected=离线选取帧数；valid=通过足力检查帧数，均不是独立测量次数。
累计角变化=反馈q相对本段首帧的变化，不是q与目标q_cmd之差。
| 姿态 | raw/selected/valid | 时长 s | 最大累计角变化 rad（限0.003） | 最大关节速度 rad/s（限0.015） | 最大IMU角速度 rad/s（限<0.015） |
|---|---:|---:|---:|---:|---:|
| 0: reference | 201/35/28 | 2.000 | 0.000381470 | 0.034187317 | 0.005741677 |
| 1: pitch_positive | 200/34/26 | 1.992 | 0.000381470 | 0.034187317 | 0.005952096 |
| 2: roll_positive | 200/34/25 | 1.988 | 0.000381470 | 0.034187317 | 0.005988736 |
| 3: pitch_negative | 200/34/24 | 1.988 | 0.000381470 | 0.034187317 | 0.006756127 |
| 4: roll_negative | 200/34/26 | 1.992 | 0.000381470 | 0.024421692 | 0.005170314 |
| 5: reference_repeat | 200/34/24 | 1.988 | 0.000381470 | 0.034187317 | 0.005267794 |
定位：reference / FR_hip_joint，CSV sequence=101，正式段+0.007983s；各项丢弃计数={'motion': 7, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：pitch_positive / FR_hip_joint，CSV sequence=702，正式段+0.008129s；各项丢弃计数={'motion': 8, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：roll_positive / RL_hip_joint，CSV sequence=1303，正式段+0.008007s；各项丢弃计数={'motion': 9, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：pitch_negative / FL_hip_joint，CSV sequence=1907，正式段+0.048066s；各项丢弃计数={'motion': 10, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：roll_negative / FL_calf_joint，CSV sequence=2504，正式段+0.011936s；各项丢弃计数={'motion': 8, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：reference_repeat / RR_hip_joint，CSV sequence=3105，正式段+0.008005s；各项丢弃计数={'motion': 10, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。

## 模型假设与参考对照（独立于内部质量检查）

- [UNMODELED_STATIC_FRICTION] URDF含非零Fc，单帧足力反算未计入未知静摩擦；静止不会消除静摩擦。Fc敏感性分析不是已验证补偿，也不是静摩擦上界。
- [CONTACT_POINT_ASSUMPTION] 反算使用foot链接原点的点力模型；URDF脚底是有半径的球，实际接触点偏置和接触力矩未纳入该反算。
- 未根据速度或驱动力矩的符号猜测静摩擦方向；本次输出修改没有补偿静摩擦。
- 模型值来自本次URDF快照，不是外部实测真值；下述差值只报告，不新增验收门槛。
整机质量：估计=47.911384 kg；URDF=41.281930 kg；差值=+6.629455 kg (+16.059%)。
躯干质量候选 kg：29.891783
躯干质心候选 / trunk m：[0.047069121763263805, 0.01283906722077719, -0.9449161383305364]
URDF躯干质心 / trunk m：[-0.008263775204270183, 0.00013850204932197672, -0.0038367281726919174]
躯干质心与URDF之差 / trunk m：[0.05533289696753399, 0.012700565171455213, -0.9410794101578446]
URDF关节Fc / N·m：{'FR_hip_joint': 1.0170625680080705, 'FR_thigh_joint': 1.2980958388469037, 'FR_calf_joint': 1.0794450668988391, 'FL_hip_joint': 1.0170625680080705, 'FL_thigh_joint': 1.2980958388469037, 'FL_calf_joint': 1.0794450668988391, 'RR_hip_joint': 1.0170625680080705, 'RR_thigh_joint': 1.2980958388469037, 'RR_calf_joint': 1.0794450668988391, 'RL_hip_joint': 1.0170625680080705, 'RL_thigh_joint': 1.2980958388469037, 'RL_calf_joint': 1.0794450668988391}
URDF足底球碰撞半径 / m：{'FR_foot': [0.02], 'FL_foot': [0.02], 'RR_foot': [0.02], 'RL_foot': [0.02]}

## 候选结果与拟合诊断

FixStand反馈腿姿整机质心 / B m：[0.015479018843247587, 0.008001100652555533, -0.6070132445557398]
FixStand目标腿姿整机质心 / B m：[0.01551309592315628, 0.008008869372417435, -0.6102842948901118]
秩=3（要求3）；条件数=17.2963763796102（限150）；力矩RMS=11.104194 N·m（限2）。
回中最大差异=0.0445862031733669 m（限0.015）；回中段索引=[5]。
Fc尺度力矩偏差敏感性 m（非置信区间、非已验证补偿）：[0.020373687353477027, 0.008741051760826875, 0.038514238358721156]
FixStand腿姿不等于动态trot腿姿；传感器静止检查不能证明全机绝对静止。

debug.json保存处理阶段、原始数据校验值、每段统计、丢弃计数和模型诊断。
