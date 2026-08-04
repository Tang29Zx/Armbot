from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lingbot_nav_bridge'

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
    maintainer='Tony Huang',
    maintainer_email='tony@armbot.local',
    description='Bridge: lingbot-map 3D → Nav2 navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = lingbot_nav_bridge.bridge_node:main',
        ],
    },
)
