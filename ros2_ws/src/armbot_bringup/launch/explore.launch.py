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
import yaml


def load_costmap_params(nav2_yaml, section):
    """Read a top-level costmap section (local_costmap / global_costmap) from
    the Nav2 params YAML and flatten it into '<section>.ros__parameters.*'
    prefixed parameters.

    launch_ros only loads the YAML top-level section matching the node name
    (e.g. 'controller_server'), so the standalone costmap sections are dropped
    and the in-process costmap sub-nodes fall back to defaults (no obstacle
    layer scan subscription -> no obstacle avoidance). Injecting them as
    prefixed params lets the sub-nodes (namespace /local_costmap etc.) read
    their own ros__parameters.* again.
    """
    with open(nav2_yaml, 'r') as f:
        data = yaml.safe_load(f)
    # Nav2 params use a two-level section: 'local_costmap: local_costmap: ros__parameters:'
    # matching the in-process costmap node full name /local_costmap/local_costmap.
    section_params = data[section][section]['ros__parameters']
    prefix = f'{section}.{section}.ros__parameters'
    out = {}

    def walk(d, pre):
        for k, v in d.items():
            key = f'{pre}.{k}' if pre else k
            if isinstance(v, dict):
                walk(v, key)
            else:
                out[key] = v

    walk(section_params, prefix)
    return out


def generate_launch_description():
    pkg_armbot_bringup = get_package_share_directory('armbot_bringup')
    nav2_params = os.path.join(pkg_armbot_bringup, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_armbot_bringup, 'config', 'slam_toolbox_params.yaml')
    local_costmap_params = load_costmap_params(nav2_params, 'local_costmap')
    global_costmap_params = load_costmap_params(nav2_params, 'global_costmap')

    # ── Arguments ──
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0')
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='115200')
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
        parameters=[nav2_params, local_costmap_params],
    )

    # ── Nav2: Planner Server ──
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, global_costmap_params],
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

    # ── Static seed TF: map -> odom ──
    # slam_toolbox only re-broadcasts map->odom when it processes a scan
    # (vehicle stationary -> filtered by minimum_travel_distance), so the
    # transform expires from the tf buffer. A static seed keeps the tree
    # complete so costmaps can generate; slam's dynamic broadcast overrides it.
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    # ── Costmap bridge: Nav2 Costmap (nav2_msgs) -> OccupancyGrid ──
    # Humble Nav2 publishes /global_costmap/costmap_raw as nav2_msgs/msg/Costmap,
    # but explore_lite expects nav_msgs/msg/OccupancyGrid. Bridge converts it.
    costmap_bridge_node = Node(
        package='armbot_bringup',
        executable='costmap_bridge',
        name='costmap_bridge',
        output='screen',
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
            'costmap_topic': '/map',
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
            'return_to_init': False,       # 探索完原地停（不回起点，避免 recovery 倒车撞障碍）
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
        TimerAction(period=0.5, actions=[static_map_odom]),
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
        TimerAction(period=5.0, actions=[costmap_bridge_node]),
        TimerAction(period=6.0, actions=[explore_node]),
        rviz_node,
    ])

    return ld
