from setuptools import find_packages, setup

package_name = 'chassis_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/chassis_control.launch.py']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='Tang29Zx',
    maintainer_email='Tang29Zx@outlook.com',
    description='LeArm mecanum chassis driver and odometry for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'chassis_control_node = chassis_control.chassis_control_node:main',
        ],
    },
)
