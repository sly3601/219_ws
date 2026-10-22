import os
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    LogInfo,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def cpu_monitor_actions():
    prefix = Path(get_package_prefix('sysu219_guide_controller'))
    executable = prefix / 'lib' / 'sysu219_guide_controller' / 'sysu219_cpu_monitor'
    workspace = next((str(path.parent) for path in (prefix, *prefix.parents)
                      if path.name == 'install'), '')
    if not executable.is_file() or not os.environ.get('DISPLAY') or not workspace:
        return [LogInfo(msg='处理器监视：无图形桌面或监视程序不可用，已跳过窗口。')]
    return [ExecuteProcess(
        cmd=[str(executable), '--workspace', workspace, '--launch-pid', str(os.getpid())],
        name='sysu219_cpu_monitor',
        output='log',
        sigterm_timeout='2',
        sigkill_timeout='2',
    )]


def prepend_env_value(name, value):
    current_value = os.environ.get(name, "")
    if not current_value:
        return value
    if value in current_value.split(os.pathsep):
        return current_value
    return value + os.pathsep + current_value


def on_success(*actions):
    def on_exit(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f'Gazebo 启动步骤失败（退出码 {event.returncode}），停止后续初始化。')]
        return list(actions)
    return on_exit


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    init_height = context.launch_configurations['height']
    com_history_seconds = float(context.launch_configurations['com_history_seconds'])

    pkg_path = os.path.join(get_package_share_directory(package_description))
    leg_pd_prefix = get_package_prefix('leg_pd_controller')
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'GAZEBO': 'true',
            'CLASSIC': 'true',
        },
    ).toxml()

    # 仅此 Gazebo launch 注入只读插件；实机 URDF/launch 不加载它。
    physics_debug_path = os.path.join(get_package_prefix('sysu219_guide_controller'),
                                     'lib', 'sysu219_guide_controller', 'libsysu219_physics_debug.so')
    diagnostics_status = '[PHYSICS_DEBUG] plugin unavailable: build optional Gazebo diagnostic target first'
    if os.path.isfile(physics_debug_path):
        root = ET.fromstring(robot_description)
        gazebo = ET.SubElement(root, 'gazebo')
        plugin = ET.SubElement(gazebo, 'plugin',
                               filename=physics_debug_path, name='sysu219_physics_debug')
        ros = ET.SubElement(plugin, 'ros')
        ET.SubElement(ros, 'namespace').text = '/gazebo'
        robot_description = ET.tostring(root, encoding='unicode')
        diagnostics_status = '[PHYSICS_DEBUG] plugin available; enabled by Csv_DebugMode'

    rviz_config_file = os.path.join(
        get_package_share_directory(package_description),
        "config",
        "visualize_urdf.rviz"
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    com_visualizer = Node(
        package='sysu219_guide_controller',
        executable='gazebo_com_visualizer.py',
        output='screen',
        parameters=[{'use_sim_time': True, 'history_seconds': com_history_seconds}],
    )

    gz_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sysu219',
            '-reference_frame', 'world',
            '-x', '0.0',
            '-y', '0.0',
            '-z', init_height,
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_sim_time': True,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True,
            }
        ],
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "leg_pd_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    sysu219_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "sysu219_guide_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 与界面 Reset Model Poses 相同的模型复位；控制器启动成功后只调用一次。
    # 不重置仿真时钟，不在控制循环中复位，不添加常驻进程。
    reset_model_poses = ExecuteProcess(
        cmd=['timeout', '--kill-after=2s', '15s',
             'ros2', 'service', 'call', '/reset_world', 'std_srvs/srv/Empty', '{}'],
        name='reset_model_poses',
        output='screen',
    )

    def report_reset_result(event, context):
        if event.returncode == 0:
            return [LogInfo(msg='Gazebo 启动复位完成：已执行一次 Reset Model Poses。')]
        return [LogInfo(msg=f'Gazebo 启动复位失败（退出码 {event.returncode}），请查看复位服务输出。')]

    return [
        LogInfo(msg=diagnostics_status),
        LogInfo(msg=f'Gazebo 生成位置：world，x=0，y=0，z={init_height}，姿态角均为 0。'),
        SetEnvironmentVariable(
            'AMENT_PREFIX_PATH',
            prepend_env_value('AMENT_PREFIX_PATH', leg_pd_prefix),
        ),

        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            prepend_env_value('LD_LIBRARY_PATH', os.path.join(leg_pd_prefix, 'lib')),
        ),

        rviz,
        com_visualizer,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare('gazebo_ros'),
                            'launch',
                            'gazebo.launch.py',
                        ]
                    )
                ]
            ),
            launch_arguments={
                'verbose': 'true',
                'world': str(Path(__file__).resolve().with_name('slippery_ground.world')),
            }.items(),
        ),

        robot_state_publisher,

        gz_spawn_entity,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=on_success(joint_state_publisher),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=on_success(imu_sensor_broadcaster),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=on_success(leg_pd_controller),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=on_success(sysu219_guide_controller),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sysu219_guide_controller,
                on_exit=on_success(
                    LogInfo(msg='Gazebo 控制器已启动，执行一次模型位姿复位。'),
                    reset_model_poses,
                ),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=reset_model_poses,
                on_exit=report_reset_result,
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='sysu219_description',
        description='package for robot description',
    )

    height = DeclareLaunchArgument(
        'height',
        # 零角几何的脚底在 base 下方 0.587 m；保留约 6 cm 离地间隙。
        default_value='0.65',
        description='Spawn height in simulation; sysu219 zero-angle geometry needs more than 0.587 m',
    )

    return LaunchDescription(
        [
            pkg_description,
            height,
            *cpu_monitor_actions(),
            DeclareLaunchArgument(
                'com_history_seconds', default_value='10.0',
                description='Duration of COM comparison traces in simulation seconds'),
            OpaqueFunction(function=launch_setup),
        ]
    )
