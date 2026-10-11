from setuptools import setup
import os
from glob import glob

package_name = 'wall_follower_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Navjot Singh',
    maintainer_email='navjotsinghsodhi52@gmail.com',
    description='Autonomous wall following using LiDAR-based perception and ROS 2.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower_executable = wall_follower_turtlebot.wall_following:main',
            'wall_finder_executable = wall_follower_turtlebot.wall_finder:main',
            'simple_wall_follower_executable = wall_follower_turtlebot.wall_following_basic:main',
            'odom_recorder_executable = wall_follower_turtlebot.odom_recorder:main',
        ],
    },
)
