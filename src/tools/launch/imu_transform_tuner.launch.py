"""IMU 变换调试工具：启动 RViz（四足模型）+ IMU 读取节点 + 轴映射调参 Qt 窗口。

启动链路：
    imu_reader 节点（独立读 hipnuc 串口，与电机解耦）
        ↓ /imu/data_raw (sensor_msgs/Imu)
    ImuTransformTuner RQt 窗口（按钮遍历 24 种右手系轴映射）
        ↓ 发布 TF world -> base（旋转 = IMU 姿态经轴映射后的机身姿态）
    RViz RobotModel 显示四足模型并随 IMU 姿态旋转

用法：
    ros2 launch tools imu_transform_tuner.launch.py
    # 可选参数：
    #   serial_port=/dev/sysu219/ttyACM4（IMU 串口）
    #   baud_rate=921600（IMU 波特率）
    #   imu_topic=/imu/data_raw（IMU 原始数据话题）
    #   parent_frame=world（TF 父坐标系）
    #   child_frame=base（TF 子坐标系，即四足模型的 base）
    #   base_height=0.35（RViz 里把 base 抬离地面）
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
    pkg_path = get_package_share_directory(package_description)
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')

    robot_description = xacro.process_file(xacro_file).toxml()
    rviz_config = os.path.join(pkg_path, 'config', 'visualize_urdf.rviz')

    imu_topic = context.launch_configurations['imu_topic']
    serial_port = context.launch_configurations['serial_port']
    baud_rate = int(context.launch_configurations['baud_rate'])
    parent_frame = context.launch_configurations['parent_frame']
    child_frame = context.launch_configurations['child_frame']
    base_height = context.launch_configurations['base_height']

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

    imu_reader = Node(
        package='tools',
        executable='imu_reader_node',
        name='imu_reader',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'imu_topic': imu_topic,
            'frame_id': 'imu_link',
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
    )

    # 用环境变量把话题/frame/高度传给 RQt 插件（rqt --standalone 不直接传 ROS 参数）
    tuner_env = {
        'IMU_TUNER_IMU_TOPIC': imu_topic,
        'IMU_TUNER_PARENT_FRAME': parent_frame,
        'IMU_TUNER_CHILD_FRAME': child_frame,
        'IMU_TUNER_BASE_HEIGHT': base_height,
    }

    return [
        robot_state_publisher,
        imu_reader,
        rviz,
        ExecuteProcess(
            cmd=['rqt', '--force-discover', '--standalone', 'ImuTransformTuner'],
            name='imu_transform_tuner',
            output='screen',
            additional_env=tuner_env,
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
            description='把 base 抬高到地面之上的 z 偏移 (m)',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/sysu219/ttyACM4',
            description='IMU 串口',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='921600',
            description='IMU 波特率',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU 原始数据话题',
        ),
        DeclareLaunchArgument(
            'parent_frame',
            default_value='world',
            description='TF 父坐标系',
        ),
        DeclareLaunchArgument(
            'child_frame',
            default_value='base',
            description='TF 子坐标系（四足模型的 base，随 IMU 姿态旋转）',
        ),
        OpaqueFunction(function=launch_setup),
    ])
