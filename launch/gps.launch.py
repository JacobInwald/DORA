from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='gps',
            package='sensors',
            executable='gps_node'
        ),
    ])
