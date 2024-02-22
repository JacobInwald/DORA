import numpy as np
import rclpy
from dora_msgs.msg import Pose
from rclpy.node import Node


class GPSPublisher(Node):
    """
    Represents a GPS sensor. (Overhead camera)
    
    This is going to subscribe to the Overhead camera in ROS to get the image to process.
    There should be a call back function that automatically updates the position and rotation
    of the robot

    The control flow will be as follows:
     - The GPS sensor will subscribe to the overhead camera
     - The overhead camera will publish the image
     - The GPS sensor will process the image to find the position and rotation of the robot
     - The GPS sensor will update the position and rotation of the robot
     - Continue...

    Attributes:
    - pos: numpy.ndarray - The current position of the robot.
        - 1 is 1 meter in this scale
    - rot: float - The current rotation of the robot. ()
        - 0 is facing the positive x-axis
    """

    def __init__(self):
        super().__init__("gps_subscriber")
        self.subscriber_ = self.create_subscription(Pose, "gps", self.callback, 10)
        self.i = 0 
        self.rot = 0.0
        self.pos = None

    def preprocess(self, scan):
        """
        Prepares the scan for use in the network.
        """
        return scan

    def callback(self, msg: 'Pose'):
        self.pos = np.array([msg.x, msg.y])
        self.rot = msg.rot

        self.get_logger().info(f"Heard: GPS {self.i}: {self.pos} {self.rot}")
        self.i += 1

    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
    
    def destroy(self):
        self.destroy_node()
    