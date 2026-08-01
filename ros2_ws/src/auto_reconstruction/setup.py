from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auto_reconstruction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='determinant',
    maintainer_email='13816096470@163.com',
    description='Perception stack: YOLO26 medicine-box verifier, VLA client, grasp pipeline',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_verifier = auto_reconstruction.verifier_node:main',
        ],
    },
)
