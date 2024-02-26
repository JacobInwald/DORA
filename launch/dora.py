from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='camera',
            package='sensors',
            executable='camera_node'
        ),
        Node(
            namespace='gps',
            package='sensors',
            executable='gps_node'
        ),
        Node(
            namespace='lds',
            package='sensors',
            executable='lds_node'
        ),
        Node(
            namespace='sweeper',
            package='actuators',
            executabale='sweeper'
        ),
        Node(
            namespace='wheels',
            package='actuators',
            executable='wheels'
        ),
        Node(
            namespace='controller',
            package='control',
            executable='controller'
        ),
        Node(
            namespace='loop',
            package='dora',
            executable='loop'
        ),
        Node(
            namespace='dora',
            package='dora',
            executable='dora'
        )
    ])