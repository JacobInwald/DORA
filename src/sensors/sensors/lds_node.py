import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html


class LdsNode(Node):
    """
    Represents the LDS (Laser Distance Sensor).

    TODO: Implement LDS node (Jacob)
    Subscribes to the Turtlebot3 LDS-02 to get pointcloud.
    Publishes occupancy map of environment.

    The control flow will be as follows:
     - The LDS node will subscribe to hardware trigger topic and the LDS
     - Calculates the occupancy map from the last laser scan when triggered by hardware
     - Optional: shift pointcloud by time difference using odometry data from hardware
     - Publishes the occupancy map

    Attributes:
    - last_scan: numpy.ndarray - The most recent laser scan
    """

    def __init__(self):
        super().__init__('lds_node')
        self.lds_sub_ = self.create_subscription(LaserScan, 'scan', self.lds_callback, 10)
        # change where appropriate
        self.hardware_sub_ = self.create_subscription(..., 'topic', self.hardware_callback, 1)
        self.max_range = 0.0
        self.last_scan = None

    def hardware_callback(self):
        """
        Calculate occupancy map from last scan and publish map
        """
        pass

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

    # functions below can be removed as this class inherits all of them from Node
    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
    
    def destroy(self):
        self.destroy_node()
        rclpy.shutdown()