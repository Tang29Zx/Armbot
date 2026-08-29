"""
Navigation mode — load a saved map and navigate autonomously.  (2026-08-22 重写)

Launches:
  - ydlidar_node          (official LiDAR driver, X2 single channel)
  - scan_filter           (filter body/mount points < 0.8m -> /scan_filtered)
  - map_server            (serve the saved .pgm/.yaml map)
  - map_relay             (relay full map to /map_static for costmap static layer)
  - slam_toolbox          (localization mode: scan-to-map matching)
  - Nav2 nodes: controller_server, planner_server, behavior_server,
    bt_navigator, waypoint_follower, velocity_smoother
  - lifecycle_manager     (auto-activates Nav2 nodes)
  - static tf base_link -> laser  (chassis runs separately, no robot_state_pub)

IMPORTANT (8-22 lessons baked in):
  * 底盘(chassis_control) 不在此 launch 内！nav.launch 内嵌底盘经常 I2C Errno 121
    不发 tf，必须单独启动：ros2 launch chassis_control chassis_control.launch.py
    推荐用 ~/start_nav.sh 一键启动（含底盘独立拉起 + tf 验证）。
  * slam 不传 map_file_name（.yaml 会导致 DeserializePoseGraph 崩溃），
    初始地图从 map_server 的 /map 订阅；用 TimerAction 延迟 6s 等 map_server 激活。
  * costmap 静态层订阅 /map_static（map_relay 转发），避免 slam 局部小图覆盖完整图。

Usage:
  ros2 launch armbot_bringup nav.launch.py map:=/home/sunrise/map_verify.yaml use_rviz:=false lidar_port:=/dev/ttyUSB0
  use_lidar:=false  = 无雷达纯里程计导航（静态 map->odom + base_link->laser 不需要）
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_armbot_bringup = get_package_share_directory('armbot_bringup')
    nav2_params = os.path.join(pkg_armbot_bringup, 'config', 'nav2_params.yaml')
    local_params = os.path.join(pkg_armbot_bringup, 'config', 'localization_params.yaml')

    # ── Arguments ──
    map_arg = DeclareLaunchArgument(
        'map', default_value='/home/sunrise/map_verify.yaml',
        description='Full path to map.yaml (e.g. /home/sunrise/map_verify.yaml)')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0')
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='115200')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for visualization')
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='true=雷达定位导航; false=静态 map->odom 纯里程计导航')

    # ── Official YDLidar driver（X2 单通道，唯一可用稳定驱动）──
    ydlidar_node = Node(
        package='ydlidar',
        executable='ydlidar_node',
        name='ydlidar_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        parameters=[{
            'port': LaunchConfiguration('lidar_port'),
            'frame_id': 'laser',
            'baudrate': 115200,
            'singleChannel': True,
            'angle_min': -180.0,
            'angle_max': 180.0,
            'frequency': 5.0,    # 8-22 15:33: 8Hz 数据量超 slam 处理→MessageFilter 队列满丢帧→slam 假死
        }],
    )

    # ── Scan filter：滤 <0.8m 车体/支架/无效点（防 costmap 障碍环 + 保留 0.8-1m 定位特征）──
    scan_filter_node = ExecuteProcess(
        cmd=['python3', '/tmp/scan_filter.py'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    # ── static tf base_link -> laser（底盘单独启动后没有 robot_state_pub，需手动发布）──
    # 数值来自 URDF: base_lidar_joint(0.175,0,0.15) + laser_joint(0,0,0.03) = (0.175,0,0.18)
    # 8-29: 曾误判雷达装反加 yaw=π，实测车头指示反向，撤销恢复 yaw=0（参数顺序 x y z yaw pitch roll）
    static_laser_tf = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '0.175', '0', '0.18', '0', '0', '0', 'base_link', 'laser'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    # ── Map Server ──
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {
            'yaml_filename': LaunchConfiguration('map'),
        }],
    )

    # ── Map Relay：只把 map_server 完整图转发到 /map_static（costmap 静态层用）──
    map_relay_node = ExecuteProcess(
        cmd=['python3', '/tmp/map_relay.py'],
        output='screen',
    )

    # ── SLAM Toolbox (localization mode) ──
    # 不传 map_file_name（.yaml 会崩）；从 map_server 的 /map 订阅初始地图
    localize_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        parameters=[local_params],
    )

    # ── 静态 map->odom（use_lidar=false 时替代 slam 定位，纯里程计导航）──
    static_map_odom = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_lidar')),
    )

    # ── Nav2: Controller Server（输出 /cmd_vel_raw，经 smoother 平滑后给底盘）──
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
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

    # ── Nav2: Velocity Smoother（订阅 /cmd_vel_raw，平滑后输出 /cmd_vel 给底盘）──
    smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        remappings=[('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel')],
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
                'map_server',
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
        map_arg,
        lidar_port_arg,
        lidar_baud_arg,
        use_rviz_arg,
        use_lidar_arg,
        # 雷达 → 过滤 → 静态tf（use_lidar 链路）
        ydlidar_node,
        scan_filter_node,
        static_laser_tf,
        # 地图
        map_relay_node,
        map_server_node,
        # slam 等 map_server 激活发布 /map 后再启动（map_server 需 2-3s 激活）
        TimerAction(period=6.0, actions=[localize_node]),
        # 静态 map->odom（无雷达模式）
        static_map_odom,
        # Nav2 全套
        TimerAction(period=1.0, actions=[
            controller_node,
            planner_node,
            behavior_node,
            bt_node,
            waypoint_node,
            smoother_node,
        ]),
        TimerAction(period=2.0, actions=[lifecycle_node]),
        rviz_node,
    ])

    return ld
