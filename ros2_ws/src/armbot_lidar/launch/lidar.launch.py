"""
Standalone LiDAR launch — publishes /scan via ydlidar_raw node.

Usage:
  ros2 launch armbot_lidar lidar.launch.py
  ros2 launch armbot_lidar lidar.launch.py port:=/dev/ttyUSB1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('armbot_lidar')
    default_params = os.path.join(pkg_share, 'config', 'ydlidar_params.yaml')

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='LiDAR serial port')
    baud_arg = DeclareLaunchArgument(
        'baudrate', default_value='115200',
        description='Serial baud rate')
    frame_arg = DeclareLaunchArgument(
        'frame_id', default_value='laser',
        description='LiDAR TF frame id')
    motor_arg = DeclareLaunchArgument(
        'motor_hz', default_value='8.0',
        description='Motor scan frequency (Hz)')

    lidar_node = Node(
        package='armbot_lidar',
        executable='ydlidar_raw',
        name='ydlidar_raw_node',
        output='screen',
        parameters=[default_params, {
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'motor_hz': LaunchConfiguration('motor_hz'),
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_arg,
        motor_arg,
        lidar_node,
    ])
