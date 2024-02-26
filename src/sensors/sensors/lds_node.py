import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html
from dora_msgs.msg import Map
from dora_srvs.srv import LdsCmd


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
        # change where appropriate
        self.map_pub_ = self.create_publisher(Map, 'map', 10)
        self.scan_srv_ = self.create_service(LdsCmd, 'lds_service', self.scan_callback)
        self.max_range = 0.0
        self.last_scan = None

    def scan_callback(self, msg: LdsCmd) -> bool:
        """
        Calculate occupancy map from last scan and publish map
        """
        return True

    def lds_callback(self, msg: 'LaserScan'):
        """
        Store last laser scan

        Args:
            msg: message received
        """
        # res = msg.angle_increment
        # start = msg.angle_min
        # end = msg.angle_max
        # scan = []
        # a = start
        # for i in range(len(msg.ranges)):
        #     scan.append([a, msg.ranges[i]])
        #     a += res

        self.last_scan = msg
        header = msg.header
        self.get_logger().info(f'Heard: LDS scan {header.frame_id} at {header.stamp.sec}s{header.stamp.nanosec}')


# Entry Point
def main():
    rclpy.init()
    lds_node = LdsNode()
    rclpy.spin(lds_node)
    lds_node.destroy_node()
    rclpy.shutdown()