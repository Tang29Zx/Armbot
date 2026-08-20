import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("vla_runtime")
    config_file = os.path.join(package_share, "config", "vla_bridge.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("policy_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("policy_port", default_value="8000"),
            DeclareLaunchArgument("shadow_mode", default_value="true"),
            DeclareLaunchArgument(
                "inference_logging_enabled", default_value="false"
            ),
            DeclareLaunchArgument("prompt", default_value="抓取药盒"),
            Node(
                package="vla_runtime",
                executable="vla_bridge",
                name="vla_bridge_node",
                parameters=[
                    config_file,
                    {
                        "policy_host": LaunchConfiguration("policy_host"),
                        "policy_port": ParameterValue(
                            LaunchConfiguration("policy_port"), value_type=int
                        ),
                        "shadow_mode": ParameterValue(
                            LaunchConfiguration("shadow_mode"), value_type=bool
                        ),
                        "inference_logging_enabled": ParameterValue(
                            LaunchConfiguration("inference_logging_enabled"),
                            value_type=bool,
                        ),
                        "prompt": LaunchConfiguration("prompt"),
                    },
                ],
                output="screen",
            ),
        ]
    )
