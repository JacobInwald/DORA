import numpy as np
import cv2
import yaml
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from dora_msgs.msg import Toy, Toys, Pose
from object_detection.detect import Detect
from object_detection.demo import annotate
from depth_estimation.estimator import Estimator


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


def display(frame, results):
    pred = annotate(results)
    out = cv2.vconcat([frame, pred])
    cv2.imshow('out', out)


class GpuNode(Node):
    """
    Represents the external GPU.

    Subscribes to the camera node.
    Publishes Toys with topic name 'toys'
    """

    def __init__(self, camera_info='data/camera_info/camerav2_1280x960.yaml'):
        super().__init__('gpu_node')
        gpu_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscriber_ = self.create_subscription(Image, '/camera', self.callback, gpu_qos)
        self.publisher_ = self.create_publisher(Toys, '/toys', 10)
        self.model = Detect()
        self.depth_model = Estimator(device='gpu')
        with open(camera_info, 'r') as file:
            self.camera_info = yaml.safe_load(file)

    def callback(self, msg):
        frame = msg_to_np(msg)
        start_time = time.time()
        results = self.model.predictions(frame)[0]  # detect toys
        boxes = results.boxes
        pub_msg = Toys()
        toy_arr = []
        for xywh, cls, conf in zip(boxes.xywh, boxes.cls, boxes.conf):
            toy_msg = Toy()
            toy_msg.cls = int(cls)
            toy_msg.conf = float(conf)
            toy_msg.position = Pose()
            toy_msg.position.x, toy_msg.position.y = self.estimate_position(frame, xywh)
            toy_arr.append(toy_msg)
        end_time = time.time()
        pub_msg.header = msg.header
        pub_msg.toys = toy_arr
        self.publisher_.publish(pub_msg)
        self.get_logger().info(f'Frame {msg.header.frame_id}: detection time {(end_time-start_time)*1000}ms')
        display(frame, results)

    def estimate_position(self, img, xywh):
        """
        Calculates position of toy given xywh bbox prediction.

        Args:
            img: image predicted on
            xywh: bbox prediciton; x, y represents the center coordinates of the bbox;
            w, h represents the width and heigh of the bbox

        Returns:
            x, y: position of the toy
        """
        bx, by = xywh.cpu().numpy()[:2]  # center of bbox
        depth_map = self.depth_model.predict(img)
        py = depth_map[round(by), round(bx)]  # get depth at bbox center
        fx, _, cx = self.camera_info['camera_matrix']['data'][:3]  # get camera focal length and center
        px = (bx - cx) / fx * py  # calculate x using similar triangles and estimated depth
        return px, py


def main():
    rclpy.init()
    gpu_node = GpuNode()
    rclpy.spin(gpu_node)
    gpu_node.destroy_node()
    rclpy.shutdown()
