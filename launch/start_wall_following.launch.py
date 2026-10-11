from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
        package='wall_follower_turtlebot',
        executable='simple_wall_follower_executable',
        output='screen'),
    ])