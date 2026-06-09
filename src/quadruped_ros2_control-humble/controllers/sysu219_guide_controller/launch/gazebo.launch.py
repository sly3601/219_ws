import os

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def prepend_env_value(name, value):
    current_value = os.environ.get(name, "")
    if not current_value:
        return value
    if value in current_value.split(os.pathsep):
        return current_value
    return value + os.pathsep + current_value


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    init_height = context.launch_configurations['height']

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

    gz_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sysu219',
            '-z', init_height,
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

    return [
        SetEnvironmentVariable(
            'AMENT_PREFIX_PATH',
            prepend_env_value('AMENT_PREFIX_PATH', leg_pd_prefix),
        ),

        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            prepend_env_value('LD_LIBRARY_PATH', os.path.join(leg_pd_prefix, 'lib')),
        ),

        rviz,

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
            launch_arguments={'verbose': 'true'}.items(),
        ),

        robot_state_publisher,

        gz_spawn_entity,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_publisher],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[imu_sensor_broadcaster],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[leg_pd_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=[sysu219_guide_controller],
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
        default_value='0.5',
        description='Init height in simulation',
    )

    return LaunchDescription(
        [
            pkg_description,
            height,
            OpaqueFunction(function=launch_setup),
        ]
    )
