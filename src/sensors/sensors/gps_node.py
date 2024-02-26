import numpy as np
import rclpy
from dora_msgs.msg import Pose
from rclpy.node import Node


class GpsNode(Node):
    """
    Represents a GPS (Overhead camera).

    TODO: Implement GPS node (Wilfredo?)
    Subscribes to the overhead camera.
    Publishes global location of robot.

    The control flow will be as follows:
     - The GPS node will subscribe to the overhead camera
     - Calculate the current global position and rotation of the robot
     - Publish Pose of the robot with topic name "gps"

    Attributes:
    - pos: numpy.ndarray - The current position of the robot.
        - 1 is 1 meter in this scale
    - rot: float - The current rotation of the robot. ()
        - 0 is facing the positive x-axis
    """

    def __init__(self):
        super().__init__('gps_node')
        self.subscription_ = self.create_subscription(..., 'topic', self.callback, 10)
        self.publisher_ = self.create_publisher(Pose, 'gps', 10)
        self.i = 0 
        self.rot = 0.0
        self.pos = None

    def callback(self, msg: 'Pose'):
        self.pos = np.array([msg.x, msg.y])
        self.rot = msg.rot

        self.get_logger().info(f"Heard: GPS {self.i}: {self.pos} {self.rot}")
        self.i += 1


if __name__=='__main__':
    rclpy.init()
    gps_node = GpsNode()
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()
    