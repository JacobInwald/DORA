import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html
from dora_msgs.msg import Map, Pose
from dora_srvs.srv import LdsCmd
from control.control.occupancy_map import OccupancyMap
from control.control.point_cloud import PointCloud

class LdsNode(Node):
    """
    Represents the LDS (Laser Distance Sensor).

    TODO: Implement LDS node (Jacob)
    Subscribes to the Turtlebot3 LDS-01 to get pointcloud.
    Publishes occupancy map of environment.

    The control flow will be as follows:
     - The LDS node will subscribe to the LDS
     - The listen to the 'lds_scan' client call
     - Calculates the occupancy map from the last laser scan when triggered by hardware
     - Optional: shift pointcloud by time difference using odometry data from hardware
     - Publishes the occupancy map and return True to client

    Attributes:
    - last_scan: numpy.ndarray - The most recent laser scan
    """

    def __init__(self):
        super().__init__('lds_node')
        self.lds_sub_ = self.create_subscription(LaserScan, 'scan', self.lds_callback, 10)
        self.gps_sub_ = self.create_subscription(Pose, 'gps', self.gps_callback, 10)
        # change where appropriate
        self.map_pub_ = self.create_publisher(Map, 'map')
        self.scan_srv_ = self.create_service(LdsCmd, 'lds_service', self.scan_callback, 1)
        self.max_range = 1.5
        self.last_scan = None

    def gps_callback(self, msg: Pose):
        """
        Store last GPS message

        Args:
            msg: message received
        """
        self.pos = (msg.x, msg.y)
        self.rot = msg.rot
        self.get_logger().info(f'Heard: GPS {msg.x} {msg.y} {msg.rot}')
    
    def scan_callback(self, msg: LdsCmd) -> bool:
        """
        Calculate occupancy map from last scan and publish map
        """
        if self.last_scan is None:
            return False
        
        res = msg.angle_increment
        start = msg.angle_min
        scan = []
        a = start
        for i in range(len(msg.ranges)):
            scan.append([a, msg.ranges[i]])
            a += res
            
        # Calculate occupancy map (TODO: add rotation)
        cloud = PointCloud(scan, self.pos, self.max_range)
        occupancy_map = OccupancyMap(self.pos, cloud)
        
        self.map_pub_.publish(occupancy_map.to_msg())
        return True

    def lds_callback(self, msg: 'LaserScan'):
        """
        Store last laser scan

        Args:
            msg: message received
        """

        self.last_scan = msg
        header = msg.header
        self.get_logger().info(f'Heard: LDS scan {header.frame_id} at {header.stamp.sec}s{header.stamp.nanosec}')

    # functions below are not necessary
    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
    
    def destroy(self):
        self.destroy_node()
        rclpy.shutdown()