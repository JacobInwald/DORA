import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    if len(sys.argv) > 1 and sys.argv[1] == "--mono":
        camera_node = Node(
            namespace='camera',
            package='sensors',
            executable='camera_node'
        )
    else:
        camera_node = Node(
            namespace='stereo',
            package='sensors',
            executable='stereo_node'
        )

    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0',
                              'frame_id': 'base_scan'}.items(),
        ),
        camera_node,
        # Node(
        #     namespace='gps',
        #     package='sensors',
        #     executable='gps_node'
        # ),
        Node(
            namespace='lds',
            package='sensors',
            executable='lds_node'
        ),
        Node(
            namespace='sweeper',
            package='actuators',
            executable='sweeper'
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
            namespace='dora',
            package='initialise',
            executable='dora'
        ),
        Node(
            namespace='loop',
            package='initialise',
            executable='loop'
        )
    ])
