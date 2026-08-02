"""
SLAM mapping mode — build a map while driving the robot manually.

Launches:
  - robot.launch.py    (chassis + LiDAR + tf)
  - slam_toolbox       (online async SLAM, publishes map->odom tf + /map)
  - rviz2              (pre-configured visualization)

Usage:
  ros2 launch armbot_bringup slam.launch.py
  ros2 launch armbot_bringup slam.launch.py lidar_port:=/dev/ttyUSB1

To save the map after driving:
  ros2 run nav2_map_server map_saver_cli -f ~/my_map
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_armbot_bringup = get_package_share_directory('armbot_bringup')
    slam_params = os.path.join(pkg_armbot_bringup, 'config', 'slam_toolbox_params.yaml')

    # ── Arguments ──
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0')
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='230400')

    # ── Robot base ──
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('armbot_bringup'),
                'launch',
                'robot.launch.py',
            ])
        ]),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
        }.items(),
    )

    # ── SLAM Toolbox (online async mapping) ──
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    # ── RViz2 ──
    rviz_config = os.path.join(pkg_armbot_bringup, 'rviz', 'armbot_nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=None,  # Always launch; remove if running headless
    )

    return LaunchDescription([
        lidar_port_arg,
        lidar_baud_arg,
        robot_launch,
        slam_node,
        rviz_node,
    ])
