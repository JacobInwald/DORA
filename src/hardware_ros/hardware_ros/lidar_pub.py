import roslibpy as rospy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
import rclpy
import numpy as np

class LidarPublisher(Node):
    """
    Represents the LiDAR.

    This is going to subscribe to the lidar in ROS to get the lidar pointcloud.
    There should be a call back function that automatically pre-processes the lidar.

    The control flow will be as follows:
     - The lidar sensor will subscribe to the lidar
     - The lidar will publish the image
     - The lidar sensor will pre-process the image
     - The lidar sensor will update its current frame attribute

    Attributes:
    - cur_scan: numpy.ndarray - The current scan of the camera.
    """

    def __init__(self):
        super().__init__("lidar_subscriber")
        self.subscriber_ = self.create_subscription(LaserScan, "lidar", self.callback, 10)
        self.i = 0
        self.max_range = 0.0
        self.cur_scan = None

    def preprocess(self, scan):
        """
        Prepares the scan for use in the network.
        """
        return scan

    def callback(self, msg: 'LaserScan'):
        # This assumes a LaserScan Class
        res = msg.angle_increment
        start = msg.angle_min
        end = msg.angle_max
        scan = []
        a = start
        for i in range(len(msg.ranges)):
            scan.append([a, msg.ranges[i]])
            a += res

        self.cur_scan = self.preprocess(scan)

        self.get_logger().info("Heard: LiDAR scan %d" % self.i)
        self.i += 1

    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
    
    def destroy(self):
        self.destroy_node()
        rclpy.shutdown()