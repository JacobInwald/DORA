import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image


def msg_to_np(msg):
    """
    Converts ROS2 Image message to numpy array.

    Args:
        msg: ROS2 sensor_msgs.msg.Image

    Returns:
        np array of the image
    """
    height = msg.height
    width = msg.width
    channel = msg.step // msg.width
    return np.reshape(msg.data, (height, width, channel))


class DisplayNode(Node):
    """
    Represents the external GPU.

    Subscribes to the camera node.
    Publishes Toys with topic name 'toys'
    """

    def __init__(self):
        super().__init__('gpu_node')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscriber_ = self.create_subscription(Image, '/stereo', self.callback, qos)
        
    def callback(self, msg):
        frame = msg_to_np(msg)
        cv2.imshow('stereo', frame)
        cv2.waitKey(10)


def main():
    rclpy.init()
    display_node = DisplayNode()
    rclpy.spin(display_node)
    cv2.destroyAllWindows()
    display_node.destroy_node()
    rclpy.shutdown()
