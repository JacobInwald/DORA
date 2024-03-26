import cv2
import time
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
        start_time = time.time()
        frame = msg_to_np(msg)
        end_time = time.time()
        self.get_logger().info('Received frame {} with process time {:.2f}ms'
                               .format(msg.header.frame_id, (end_time-start_time)*1000))
        cv2.imshow('stereo', frame)
        cv2.waitKey(10)


def main():
    rclpy.init()
    display_node = DisplayNode()
    rclpy.spin(display_node)
    cv2.destroyAllWindows()
    display_node.destroy_node()
    rclpy.shutdown()
