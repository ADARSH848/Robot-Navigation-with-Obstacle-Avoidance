from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_assignment',
            executable='path_generator',
            name='path_generator',
            output='screen'
        ),
        Node(
            package='navigation_assignment',
            executable='controller',
            name='controller',
            output='screen'
        ),
    ])