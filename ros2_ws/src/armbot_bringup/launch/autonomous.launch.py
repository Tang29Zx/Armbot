"""
Full autonomous mode — SLAM + Navigation running simultaneously.

This mode builds the map while navigating. It is suitable for:
  - Exploring unknown environments
  - Navigating while expanding the map
  - Single-launch "just drive" scenarios

Launches:
  - robot.launch.py        (chassis + LiDAR + tf)
  - slam_toolbox           (online async SLAM: map->odom tf + /map)
  - Nav2 nodes             (planner, controller, behaviors, BT, smoother)
  - lifecycle_manager
  - rviz2

Usage:
  ros2 launch armbot_bringup autonomous.launch.py
  ros2 launch armbot_bringup autonomous.launch.py lidar_port:=/dev/ttyUSB0

After launch, use RViz "2D Nav Goal" to set a navigation target.
The robot will navigate while continuing to build the map.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_armbot_bringup = get_package_share_directory('armbot_bringup')
    nav2_params = os.path.join(pkg_armbot_bringup, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_armbot_bringup, 'config', 'slam_toolbox_params.yaml')

    # ── Arguments ──
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0')
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='230400')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true')

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

    # ── SLAM Toolbox (mapping mode, provides map->odom tf) ──
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    # ── Nav2: Controller Server ──
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Nav2: Planner Server ──
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Nav2: Behavior Server ──
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Nav2: BT Navigator ──
    bt_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Nav2: Waypoint Follower ──
    waypoint_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Nav2: Velocity Smoother ──
    smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Nav2: Lifecycle Manager ──
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    # ── RViz2 ──
    rviz_config = os.path.join(pkg_armbot_bringup, 'rviz', 'armbot_nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ── Compose ──
    ld = LaunchDescription([
        lidar_port_arg,
        lidar_baud_arg,
        use_rviz_arg,
        robot_launch,
        TimerAction(period=1.0, actions=[slam_node]),
        TimerAction(period=2.0, actions=[
            controller_node,
            planner_node,
            behavior_node,
            bt_node,
            waypoint_node,
            smoother_node,
        ]),
        TimerAction(period=3.0, actions=[lifecycle_node]),
        rviz_node,
    ])

    return ld
