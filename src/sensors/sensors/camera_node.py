import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image


class CameraNode(Node):
    """
    Represents the front-facing camera.

    Subscribes to the Logitech webcam.
    Publishes Toys with topic name 'toys'
    """

    def __init__(self, rate=10):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, '/camera', rate)
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            raise IOError('Cannot open webcam')
        self.cap.set(cv2.CAP_PROP_FPS, rate)
        self.frame_no = 1

        self.capture()

    def capture(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            stamp = self.get_clock().now().to_msg()
            if ret:
                self.callback(frame, stamp)
                self.frame_no += 1

    def callback(self, frame, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = str(self.frame_no)

        msg = Image()
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] * np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()

        self.publisher_.publish(msg)
        self.get_logger().info(f'Frame {self.frame_no} published.')


def main():
    rclpy.init()
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()
