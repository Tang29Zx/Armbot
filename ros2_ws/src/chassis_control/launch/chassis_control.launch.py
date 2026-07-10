from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_control',
            executable='chassis_control_node',
            name='chassis_control',
            output='screen',
            parameters=[{
                'i2c_bus': 5,
                'i2c_addr': 0x34,
                'update_rate': 50.0,
                'max_linear_speed': 0.5,
                'max_angular_speed': 2.0,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'wheel_radius': 0.0325,
                'wheel_dist_lr': 0.130,
                'wheel_dist_fb': 0.130,
            }],
        ),
    ])
