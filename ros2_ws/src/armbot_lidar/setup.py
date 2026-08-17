from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'armbot_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='YDLIDAR launch configuration for Armbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['ydlidar_raw = armbot_lidar.ydlidar_raw_node:main'],
    },
)
