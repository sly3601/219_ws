仅 `gazebo.launch.py` 启动质心对比节点。RViz 已配置 `/com_markers`：橙色为 Gazebo 真实机身位置，青色为控制器估计质心近似，球为当前位置、线为近期轨迹。

按当前 MPC 定义，`pcb_B=0`，质心近似等于 `base` 原点；真实值来自 `gazebo_ros_p3d` 的 `base` 世界位姿，估计值来自估计器（`p_body + R * pcb_B`）。本显示不计算全机器人质量加权质心，不使用 Gazebo 数据修正估计器，也不自动对齐两条轨迹。

两种数据以约 25 Hz 发布，带各自仿真时间戳，坐标系均为 `world`。显示比较轨迹，不计算两次异步采样之间的瞬时误差。默认保留最近 10 仿真秒；时间窗口使用时间戳裁剪，重置仿真后清空，未收到的数据不画假点。

可调整窗口：

```bash
ros2 launch sysu219_guide_controller gazebo.launch.py com_history_seconds:=20.0
```

估计位置发布仅在 `gazebo_com_visualization=true` 且 `use_sim_time=true` 时开启；两项只在 `gazebo.yaml` 中开启。实机默认关闭。原 `/foot_markers` 继续只显示足底目标；原质心显示移到上述 Gazebo 专用节点。

修改涉及控制器和 `sysu219_description` 的仿真插件配置；安装这两个包的新版本后需重启 Gazebo 才会加载真实位姿插件。
