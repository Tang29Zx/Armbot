"""
Visual SLAM + Nav2 launch (lingbot-map → bridge → Nav2).

Launches:
  - lingbot_nav_bridge (camera → 3D → /map + TF)
  - chassis_control (I2C motor driver)
  - Nav2: controller, planner, behaviors, BT navigator
  - lifecycle_manager

Usage:
  ros2 launch lingbot_nav_bridge vision_nav.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lingbot_nav_bridge')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_vision.yaml')

    # ── Arguments ──
    camera_arg = DeclareLaunchArgument(
        'camera_id', default_value='0',
        description='USB camera device ID')
    model_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='Path to lingbot-map checkpoint')
    map_res_arg = DeclareLaunchArgument(
        'map_resolution', default_value='0.05')

    # ── Bridge node ──
    bridge_node = Node(
        package='lingbot_nav_bridge',
        executable='bridge_node',
        name='lingbot_nav_bridge',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'model_path': LaunchConfiguration('model_path'),
            'map_resolution': LaunchConfiguration('map_resolution'),
        }],
    )

    # ── Chassis ──
    chassis_node = Node(
        package='chassis_control',
        executable='chassis_control_node',
        name='chassis_control',
        output='screen',
    )

    # ── Nav2 Controller ──
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Nav2 Planner ──
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Nav2 Behaviors ──
    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Nav2 BT Navigator ──
    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    # ── Lifecycle Manager ──
    lifecycle = Node(
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
            ],
        }],
    )

    return LaunchDescription([
        camera_arg,
        model_arg,
        map_res_arg,
        bridge_node,
        chassis_node,
        TimerAction(period=3.0, actions=[controller, planner]),
        TimerAction(period=3.5, actions=[behavior, bt_nav]),
        TimerAction(period=4.0, actions=[lifecycle]),
    ])
