from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    wall_finder_node = Node(
        package='wall_follower_turtlebot',
        executable='wall_finder_executable',
        name='wall_finder',
        output='screen'
    )

    # ⭐ ADD ACTION SERVER NODE
    odom_recorder_node = Node(
        package='wall_follower_turtlebot',
        executable='odom_recorder_executable',
        name='odom_recorder',
        output='screen'
    )

    wall_follower_node = Node(
        package='wall_follower_turtlebot',
        executable='wall_follower_executable',
        name='wall_follower',
        output='screen'
    )

    return LaunchDescription([
        wall_finder_node,
        odom_recorder_node,   # ⭐ start action server before follower
        wall_follower_node
    ])
