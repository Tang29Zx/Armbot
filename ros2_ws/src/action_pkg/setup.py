from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'action_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Traceable config (contract sec 4)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Arm/gripper control node implementing the stable ROS2 interface contract',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = action_pkg.arm_controller_node:main',
            'arm_teleop = action_pkg.arm_teleop_node:main',
            # Backward-compatible alias for the old i2c_controller entry point.
            'i2c_controller = action_pkg.arm_controller_node:main',
        ],
    },
)
