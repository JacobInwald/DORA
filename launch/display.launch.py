from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='display',
            package='remote',
            executable='display_node'
        )
    ])
