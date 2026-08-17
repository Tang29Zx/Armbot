"""
Base robot bringup — chassis + LiDAR + robot_state_publisher.

Launches:
  - chassis_control      (I2C motor driver + odometry)
  - LiDAR driver         (/scan)
  - robot_state_publisher (URDF -> all tf including base_link -> laser)

Usage:
  ros2 launch armbot_bringup robot.launch.py
  ros2 launch armbot_bringup robot.launch.py lidar_port:=/dev/ttyUSB1
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ── Package paths ──
    pkg_armbot_bringup = get_package_share_directory('armbot_bringup')
    pkg_armbot_description = get_package_share_directory('armbot_description')
    pkg_armbot_lidar = get_package_share_directory('armbot_lidar')
    pkg_chassis_control = get_package_share_directory('chassis_control')

    # ── Arguments ──
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='LiDAR serial port')
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='115200',
        description='LiDAR baud rate')
    lidar_motor_arg = DeclareLaunchArgument(
        'lidar_motor_hz', default_value='8.0',
        description='LiDAR motor frequency (Hz)')
    # use_lidar=false: 不启动 armbot_lidar 自写驱动（由外部官方 ydlidar 驱动启动，
    # 自写驱动帧乱导致 SLAM 无法建图）
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='Whether to start the armbot_lidar driver')
    # ── URDF: use xacro to process the model description ──
    # Falls back to reading the raw .xacro if xacro is not installed
    # (robot_state_publisher in Humble can parse simple xacro without macros).
    urdf_path = os.path.join(pkg_armbot_description, 'urdf', 'armbot.urdf.xacro')
    robot_desc_cmd = Command(['xacro ', urdf_path])
    robot_description = ParameterValue(robot_desc_cmd, value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # Static TF for base_link -> laser is handled by the URDF fixed joints
    # (base_lidar_joint + laser_joint) via robot_state_publisher.
    # No manual static_transform_publisher needed.

    # ── Chassis control ──
    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chassis_control'),
                'launch',
                'chassis_control.launch.py',
            ])
        ]),
    )

    # ── LiDAR ──
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('armbot_lidar'),
                'launch',
                'lidar.launch.py',
            ])
        ]),
        launch_arguments={
            'port': LaunchConfiguration('lidar_port'),
            'baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'laser',
            'motor_hz': LaunchConfiguration('lidar_motor_hz'),
        }.items(),
    )

    # ── Compose ──
    # TimerAction delays the LiDAR by 1s so the chassis I2C bus is ready first.
    # The chassis_control node uses I2C bus 5 (0x34) and the LiDAR uses serial,
    # so there's no actual bus conflict, but the delay gives the OS time to
    # enumerate USB devices.
    actions = [
        lidar_port_arg,
        lidar_baud_arg,
        lidar_motor_arg,
        use_lidar_arg,
        robot_state_pub,
        chassis_launch,
    ]
    # use_lidar=true 时才启动自写雷达驱动
    actions.append(
        TimerAction(
            period=1.0,
            actions=[lidar_launch],
            condition=IfCondition(LaunchConfiguration('use_lidar')),
        )
    )
    ld = LaunchDescription(actions)

    return ld
