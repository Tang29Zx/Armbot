import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('action_pkg')
    arm_config = os.path.join(pkg_share, 'config', 'arm_config.yaml')
    teleop_config = os.path.join(pkg_share, 'config', 'teleop_config.yaml')
    filter_config = os.path.join(
        pkg_share, 'config', 'state_filter_config.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'autorepeat_rate': 20.0}],
            output='screen',
        ),
        Node(
            package='action_pkg',
            executable='arm_controller',
            name='arm_controller_node',
            parameters=[arm_config],
            output='screen',
        ),
        Node(
            package='action_pkg',
            executable='arm_teleop',
            name='arm_teleop_node',
            parameters=[teleop_config],
            output='screen',
        ),
        Node(
            package='action_pkg',
            executable='arm_state_filter',
            name='arm_state_filter_node',
            parameters=[filter_config],
            output='screen',
        ),
    ])
