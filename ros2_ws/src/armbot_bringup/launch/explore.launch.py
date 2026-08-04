"""
Autonomous exploration + mapping mode.

The robot explores an unknown environment completely on its own:
  - SLAM Toolbox builds the map in real time (mapping mode)
  - explore_lite detects frontier boundaries and generates nav targets
  - Nav2 plans paths and avoids obstacles to reach each frontier

No human interaction needed — the robot drives itself until the entire
reachable area is mapped, then returns to the start position.

Launches:
  - robot.launch.py              (chassis + LiDAR + tf)
  - slam_toolbox                 (online async SLAM, map->odom tf + /map)
  - explore_lite                 (frontier detection + auto goal selection)
  - Nav2 nodes                   (planner, controller, behaviors, BT, smoother)
  - lifecycle_manager            (auto-activates Nav2 nodes)
  - rviz2                        (visualization, optional)

Usage:
  ros2 launch armbot_bringup explore.launch.py
  ros2 launch armbot_bringup explore.launch.py lidar_port:=/dev/ttyUSB1

To save the map after exploration completes:
  ros2 run nav2_map_server map_saver_cli -f ~/my_map
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

    # ── Robot base (chassis + LiDAR + tf) ──
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

    # ── Explore Lite: frontier-based autonomous exploration ──
    # This node continuously detects map frontiers (boundaries between
    # known and unknown space) and sends Nav2 goals to explore them.
    # It stops automatically when no more frontiers remain.
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_base_frame': 'base_link',
            'costmap_topic': '/global_costmap/costmap_raw',
            'visualize': True,
            # Exploration speed
            'planner_frequency': 0.33,     # plan new goal every ~3s
            'progress_timeout': 30.0,      # give up if no progress in 30s
            # Frontier scoring weights
            'potential_scale': 3.0,        # prefer nearby frontiers
            'orientation_scale': 0.5,      # low weight on facing direction
            'gain_scale': 1.0,             # prefer large frontiers
            # Frontier detection
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.5,      # ignore tiny frontiers (meters)
            # Behaviour
            'return_to_init': True,        # return to start when done
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

    # ── Compose: launch nodes with staggered timing ──
    # The ordering ensures each layer is ready before the next starts:
    #   0.0s  robot base (chassis, LiDAR, TF)
    #   1.0s  SLAM (needs /scan + tf tree)
    #   2.0s  Nav2 nodes (need SLAM to be running)
    #   4.0s  lifecycle_manager (auto-activate Nav2)
    #   6.0s  explore_lite (need full Nav2 stack active)
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
        TimerAction(period=4.0, actions=[lifecycle_node]),
        TimerAction(period=6.0, actions=[explore_node]),
        rviz_node,
    ])

    return ld
