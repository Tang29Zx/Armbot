import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('action_pkg')
    config_file = os.path.join(pkg_share, 'config', 'teleop_config.yaml')

    return LaunchDescription([
        Node(
            package='action_pkg',
            executable='arm_teleop',
            name='arm_teleop_node',
            parameters=[
                config_file,
                {'shadow_mode': True, 'joy_timeout_sec': 5.0},
            ],
            output='screen',
        ),
    ])
