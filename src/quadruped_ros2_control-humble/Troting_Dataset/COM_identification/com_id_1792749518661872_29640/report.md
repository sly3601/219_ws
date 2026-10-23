# 机身质心XY辨识（trunk本体，排除四条腿；Z不辨识）

[COM_ID][COMPUTED] 采集完整，离线计算成功；内部质量检查未通过。
失败检查数：15；参数状态：候选值，未独立验证。未自动修改模型。
机身质心XY候选 / trunk坐标 m：[-0.02032109018115323, 7.552371492129625e-05]
Z不辨识、不验收；仅固定参考=-0.003836728 m，来源=URDF trunk inertial origin。倾斜时XY依赖该参考。
采用整机质量=41.800000 kg；非机身模型质量=18.019602 kg；机身质量=23.780398 kg；来源=external supplied mass。
CAN力矩换算按用户确认准确使用，不自动缩放、不猜测零偏、不重复叠加PD。
数据目录：/home/sysu219/219_ws/src/quadruped_ros2_control-humble/Troting_Dataset/COM_identification/com_id_1792749518661872_29640
输出目录：/home/sysu219/219_ws/src/quadruped_ros2_control-humble/Troting_Dataset/COM_identification/com_id_1792749518661872_29640

## 未通过的质量检查

- reference: [MOTION_QUALITY] 正式段速度质量超限：joint=FL_hip_joint, |dq|max=0.034187317 rad/s (limit<=0.015), gyro_norm_max=0.005303665 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- reference: [LOW_VALID_RATIO] 可用帧=23/35，低于80%质量参考；rejected_counts={'motion': 12, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- pitch_positive: [MOTION_QUALITY] 正式段速度质量超限：joint=FR_calf_joint, |dq|max=0.024419785 rad/s (limit<=0.015), gyro_norm_max=0.005761023 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- pitch_positive: [LOW_VALID_RATIO] 可用帧=23/35，低于80%质量参考；rejected_counts={'motion': 12, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- roll_positive: [MOTION_QUALITY] 正式段速度质量超限：joint=RL_thigh_joint, |dq|max=0.034187317 rad/s (limit<=0.015), gyro_norm_max=0.006648875 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- roll_positive: [LOW_VALID_RATIO] 可用帧=25/34，低于80%质量参考；rejected_counts={'motion': 9, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- pitch_negative: [MOTION_QUALITY] 正式段速度质量超限：joint=FR_thigh_joint, |dq|max=0.034187317 rad/s (limit<=0.015), gyro_norm_max=0.005179541 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- pitch_negative: [LOW_VALID_RATIO] 可用帧=23/35，低于80%质量参考；rejected_counts={'motion': 12, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- roll_negative: [MOTION_QUALITY] 正式段速度质量超限：joint=RL_hip_joint, |dq|max=0.034187317 rad/s (limit<=0.015), gyro_norm_max=0.005343633 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- roll_negative: [LOW_VALID_RATIO] 可用帧=27/36，低于80%质量参考；rejected_counts={'motion': 9, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。仍使用可用帧计算候选，不宣称静态质量合格。
- reference_repeat: [MOTION_QUALITY] 正式段速度质量超限：joint=RL_thigh_joint, |dq|max=0.024421692 rad/s (limit<=0.015), gyro_norm_max=0.005590127 rad/s (limit<0.015)；保留原始数据，继续离线计算。
- [MASS_CONSISTENCY] 跨姿态质量标准差/整机质量=0.078575 > 0.05；CAN换算按用户确认准确，分析模型、接触及静力假设。
- [MOMENT_FIT] 跨姿态重力矩拟合RMS=13.966992 N·m > 2 N·m。
- [REPEATABILITY] 回中最大差异=0.036021357 m > 0.015 m；存在方向相关摩擦、漂移或反馈偏差。
- [LEAVE_ONE_OUT] 留一姿态重算最大XY变化=0.027996269 m > 0.015 m。

## 各姿态诊断

raw=CSV正式段帧数；selected=离线选取帧数；valid=通过足力检查帧数，均不是独立测量次数。
累计角变化=反馈q相对本段首帧的变化，不是q与目标q_cmd之差。
| 姿态 | raw/selected/valid | 时长 s | 最大累计角变化 rad（限0.003） | 最大关节速度 rad/s（限0.015） | 最大IMU角速度 rad/s（限<0.015） |
|---|---:|---:|---:|---:|---:|
| 0: reference | 200/35/23 | 1.988 | 0.000381470 | 0.034187317 | 0.005303665 |
| 1: pitch_positive | 200/35/23 | 1.992 | 0.000381470 | 0.024419785 | 0.005761023 |
| 2: roll_positive | 200/34/25 | 1.988 | 0.000381470 | 0.034187317 | 0.006648875 |
| 3: pitch_negative | 201/35/23 | 1.997 | 0.000381470 | 0.034187317 | 0.005179541 |
| 4: roll_negative | 200/36/27 | 1.992 | 0.000381470 | 0.034187317 | 0.005343633 |
| 5: reference_repeat | 200/36/31 | 1.988 | 0.000382423 | 0.024421692 | 0.005590127 |
定位：reference / RR_hip_joint，CSV sequence=101，正式段+0.008041s；各项丢弃计数={'motion': 12, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：pitch_positive / RR_thigh_joint，CSV sequence=702，正式段+0.011965s；各项丢弃计数={'motion': 12, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：roll_positive / FR_calf_joint，CSV sequence=1303，正式段+0.007973s；各项丢弃计数={'motion': 9, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：pitch_negative / RL_thigh_joint，CSV sequence=1904，正式段+0.020024s；各项丢弃计数={'motion': 12, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：roll_negative / FL_thigh_joint，CSV sequence=2504，正式段+0.012041s；各项丢弃计数={'motion': 9, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。
定位：reference_repeat / RL_calf_joint，CSV sequence=3124，正式段+0.196147s；各项丢弃计数={'motion': 5, 'jacobian': 0, 'mass_range': 0, 'insufficient_vertical_support': 0, 'friction_ratio': 0}。

## 逐姿态机身静力平衡（均为模型反算，不是实测接触力）

静态无外部力矩时，合力矩沿重力的分量应为0；该分量无法通过改变质心坐标消除。
| 姿态 | 合力残差模长 N（相对采用质量） | 沿重力的力矩 N·m | 机身拟合残差XYZ N·m |
|---|---:|---:|---|
| reference | 132.338492 | 2.529032 | [1.304128, -3.361035, 2.570651] |
| pitch_positive | 79.776368 | 1.389596 | [1.19872, -32.87021, 1.267502] |
| roll_positive | 52.500588 | 6.014193 | [-8.107206, 29.064182, 3.898709] |
| pitch_negative | 39.822014 | 2.862710 | [8.902638, 27.576073, 3.00332] |
| roll_negative | 74.266519 | 1.914407 | [9.292207, -19.26423, 0.954912] |
| reference_repeat | 76.898863 | 0.070394 | [-13.152981, -1.361556, -0.831724] |

## 模型假设与参考对照（独立于内部质量检查）

- [UNMODELED_STATIC_FRICTION] URDF含非零Fc，单帧足力反算未计入未知静摩擦；静止不会消除静摩擦。Fc敏感性分析不是已验证补偿，也不是静摩擦上界。
- [CONTACT_POINT_ASSUMPTION] 反算使用foot链接原点的点力模型；URDF脚底是有半径的球，实际接触点偏置和接触力矩未纳入该反算。
- 未根据速度或驱动力矩的符号猜测静摩擦方向；本次输出修改没有补偿静摩擦。
- 模型值来自本次URDF快照，不是外部实测真值；下述差值只报告，不新增验收门槛。
整机质量：采用=41.800000 kg；URDF=41.281930 kg；差值=+0.518070 kg (+1.255%)。
躯干质量候选 kg：23.780398
躯干质心XY候选 / trunk m：[-0.02032109018115323, 7.552371492129625e-05]
URDF躯干质心 / trunk m：[-0.008263775204270183, 0.00013850204932197672, -0.0038367281726919174]
躯干质心与URDF之差 / trunk m：[-0.012057314976883048, -6.297833440068046e-05, None]
力矩反算的等效总质量（仅诊断，不作为真机质量输入）：48.682135 kg。
URDF关节Fc / N·m：{'FR_hip_joint': 1.0170625680080705, 'FR_thigh_joint': 1.2980958388469037, 'FR_calf_joint': 1.0794450668988391, 'FL_hip_joint': 1.0170625680080705, 'FL_thigh_joint': 1.2980958388469037, 'FL_calf_joint': 1.0794450668988391, 'RR_hip_joint': 1.0170625680080705, 'RR_thigh_joint': 1.2980958388469037, 'RR_calf_joint': 1.0794450668988391, 'RL_hip_joint': 1.0170625680080705, 'RL_thigh_joint': 1.2980958388469037, 'RL_calf_joint': 1.0794450668988391}
URDF足底球碰撞半径 / m：{'FR_foot': [0.02], 'FL_foot': [0.02], 'RR_foot': [0.02], 'RL_foot': [0.02]}

## 候选结果与拟合诊断

秩=2（要求2，仅XY）；条件数=1.0022263034258752（限150）；力矩RMS=13.966992 N·m（限2）。
回中最大差异=0.03602135701981672 m（限0.015）；回中段索引=[5]。
XY的Fc尺度力矩偏差敏感性 m（非置信区间、非已验证补偿）：[0.024494602929639113, 0.010926119849096283]
FixStand腿姿不等于动态trot腿姿；传感器静止检查不能证明全机绝对静止。

debug.json保存完整A/b矩阵、一阶矩、奇异值、逐姿态力/力矩分解、拟合贡献、数据校验值及可观测范围。
body_wrench_debug.csv逐帧保留过渡/记录段，包括未入选帧：各腿足端、J、反算力、力矩、重力和拟合残差。
body_joint_debug.csv逐关节逐帧输出CAN tau、q/dq、目标、P/D分量、腿部重力项、B*dq和求解右端；PD代理量不是实测力矩。
不能从现有接口独立观测足底载荷、外部扶持/拉拽、机身绝对平动及传感器采样年龄；不保证一次采集能识别所有原因。
