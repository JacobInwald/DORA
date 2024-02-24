import rclpy
from rclpy.node impot Node
from object_detection.detect import Detect
from dora_msgs.msg import Toy


class CameraNode(Node):
    """
    Represents the front-facing camera.

    TODO: Implement the Camera class (Ardith)
    Subscribes to the RPi camera.
    Callback function publishes [Toy] with topic name 'toys'
    """

    def __init__(self):
        super.__init__('camera_node')
        self.subscription = self.create_subscription()
        self.publisher_ = self.create_publisher([Toy], 'toys', 10)

    def preprocess(self):
        """
        Updates the position of the robot based on the GPS sensor.
        """
