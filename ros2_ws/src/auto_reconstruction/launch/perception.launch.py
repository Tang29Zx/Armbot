"""
Launch file for the perception stack.

Starts:
  - yolo_verifier_node  (medicine-box detection + confirmation)
  - (future) vla_client_node

Usage:
  ros2 launch auto_reconstruction perception.launch.py \
    model_path:=/path/to/model.pt
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('auto_reconstruction')
    default_config = os.path.join(pkg_share, 'config', 'detection_config.yaml')

    # Declare launch arguments so users can override from CLI
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to YOLO26 weights (.pt or .engine)',
    )
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum detection confidence',
    )
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description="Inference device: 'cpu' or 'cuda'",
    )
    calib_file_arg = DeclareLaunchArgument(
        'calib_file',
        default_value='',
        description='Path to calibrated_bbox YAML (optional)',
    )

    verifier_node = Node(
        package='auto_reconstruction',
        executable='yolo_verifier',
        name='yolo_verifier_node',
        output='screen',
        parameters=[
            default_config,
            {
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'device': LaunchConfiguration('device'),
                'calib_file': LaunchConfiguration('calib_file'),
            },
        ],
    )

    return LaunchDescription([
        model_path_arg,
        confidence_arg,
        device_arg,
        calib_file_arg,
        verifier_node,
    ])
