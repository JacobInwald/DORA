from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='gpu',
            package='sensors',
            executable='gpu_node'
        )
    ])
