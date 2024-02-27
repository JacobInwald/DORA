import rclpy 
from rclpy.node import Node
import cv2
import numpy as np
from dora_msgs.msg import Pose
from .image_processor import ImageProcessor

class GpsNode(Node):
    """
    Represents a GPS (Overhead camera).

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
        super().__init__('gps')
        
        self.LAT_MIN = 0.0
        self.LAT_MAX = 1.2631578947
        self.LONG_MIN = 0.0
        self.LONG_MAX = 1.0
        self.IMAGE_Y = 950
        self.IMAGE_X = 1200

        self.robotPosition = None
        self._processor = ImageProcessor()

        self.pos_pub = self.create_publisher(Pose, "/gps", 10)
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(1/30, self.process_video)

    def process_video(self):
        try: 
            # Get current frame
            ret, frame = self.cap.read()
            
            # Resize frame to intended dimensions
            frame = cv2.resize(frame, None, fx=1, fy=1, interpolation=cv2.INTER_AREA)

            # Locate robot position
            pos, rot, _, _ = self._processor.runProcessor(frame)
            msg = Pose(x=float(pos[1]), y=float(pos[0]), rot=rot)
            self.pos_pub.publish(msg)

            self.get_logger().log(f'Robot at {pos} with angle {rot}')
        except:
            self.get_logger().error('Failed to capture video frame')


def main():
    rclpy.init()
    node = GpsNode()
    rclpy.spin(node)
    rclpy.destroy_node(node)
    rclpy.shutdown()

