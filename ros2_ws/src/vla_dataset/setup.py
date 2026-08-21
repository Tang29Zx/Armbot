from setuptools import find_packages, setup


package_name = 'vla_dataset'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Armbot VLA recording, review, and LeRobot export tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_episode = vla_dataset.record_episode:main',
            'review_episode = vla_dataset.review_episode:main',
            'export_lerobot = vla_dataset.lerobot_export:main',
        ],
    },
)
