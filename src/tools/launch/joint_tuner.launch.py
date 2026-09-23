"""Joint Tuner 调试工具：在 RViz 中查看 sysu219 12 个关节角。

启动链路：
    JointTuner RQt 窗口
        ↓ /joint_tuner/joint_targets (Float64MultiArray)
    joint_state_relay 节点（tools 包）
        ↓ /joint_states (sensor_msgs/JointState)
    robot_state_publisher
        ↓ TF
    RViz RobotModel / TF

完全独立于 Gazebo 和 sysu219_guide_controller，纯粹的可视化调试。
使用的中间话题 /joint_tuner/joint_targets 是 tools 包私有话题，不会和控制器
的命令话题撞名；relay 输出的 /joint_states 只在没有 joint_state_broadcaster
抢发时才安全，因此该 launch 适用于"不启动仿真/实机"的独立预览。

用法：
    ros2 launch tools joint_tuner.launch.py
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    pkg_path = os.path.join(get_package_share_directory(package_description))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')

    # 不传 GAZEBO/CLASSIC，走默认 hardware（HARDWARE sysu219）即可，robot_state_publisher
    # 只读 robot_description + /joint_states，不调 hardware 接口。
    robot_description = xacro.process_file(xacro_file).toxml()

    rviz_config = os.path.join(pkg_path, 'config', 'visualize_urdf.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'publish_frequency': 50.0,
        }],
    )

    joint_state_relay = Node(
        package='tools',
        executable='joint_state_relay_node',
        name='joint_state_relay',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
    )

    # xacro 默认没有 'world' link，根 link 是 'base'。补一个 world -> base 的静态变换，
    # 让 RViz 的 Fixed Frame 用 'world' 即可。base 默认 z=0，但 base 在躯干中点（trunk
    # 上下都伸出），站着时一半嵌在地面里——所以把它抬高到地面之上。抬高值用 launch
    # 参数 base_height 控制（默认 0.35，与 gazebo 启动时的 init_height 接近）。
    world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base',
        arguments=[
            '--x', '0', '--y', '0',
            '--z', LaunchConfiguration('base_height'),
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'world', '--child-frame-id', 'base',
        ],
    )

    return [
        robot_state_publisher,
        joint_state_relay,
        world_to_base,
        rviz,

        # JointTuner 独立窗口（独立 RViz 之外）
        ExecuteProcess(
            cmd=['rqt', '--force-discover', '--standalone', 'JointTuner'],
            name='joint_tuner',
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pkg_description',
            default_value='sysu219_description',
            description='robot description package',
        ),
        DeclareLaunchArgument(
            'base_height',
            default_value='0.35',
            description='把 base 抬高到地面之上的 z 偏移 (m)。默认 0.35 与 gazebo init_height 接近。',
        ),
        OpaqueFunction(function=launch_setup),
    ])