import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from .utils import msg_to_np


class DisplayNode(Node):
    """
    Represents the remote display.

    Subscribes to the stereo node.
    Display real-time detection output.
    """

    def __init__(self):
        super().__init__('display_node')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscriber_ = self.create_subscription(Image, '/stereo', self.callback, qos)
        
    def callback(self, msg):
        frame = msg_to_np(msg)
        self.get_logger().info(f'Received frame {msg.header.frame_id}')
        cv2.imshow('stereo', frame)
        cv2.waitKey(10)


def main():
    rclpy.init()
    display_node = DisplayNode()
    rclpy.spin(display_node)
    cv2.destroyAllWindows()
    display_node.destroy_node()
    rclpy.shutdown()
