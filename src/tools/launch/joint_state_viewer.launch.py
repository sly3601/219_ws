"""JointStateViewer 独立启动。

直接弹出一个 RQt 窗口，订阅当前 ROS 系统里的 /joint_states 和 /joint_cmd_states，
实时显示 12 关节的实际位置与目标位置（双列）。

仿真与实机通用（前提是 joint_state_broadcaster 和 sysu219_guide_controller 在跑）。

一键保存按钮把当前 12 个目标位置（cmd_pos）写到 ~/219_ws/src/tools/config/joint_states/，
文件名 joint_snapshot_YYYYMMDD_HHMMSS.yaml。

用法：
    ros2 launch tools joint_state_viewer.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['rqt', '--force-discover', '--standalone', 'JointStateViewer'],
            name='joint_state_viewer',
            output='screen',
        ),
    ])