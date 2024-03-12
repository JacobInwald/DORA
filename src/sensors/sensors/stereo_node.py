import numpy as np
import cv2
import yaml
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from dora_msgs.msg import Toy, Pose, Toys
from object_detection.detect import Detect
from object_detection.demo import annotate


def display(frame, results):
    pred = annotate(results)
    out = cv2.resize(cv2.vconcat([frame, pred]), dsize=(0, 0), fx=0.5, fy=0.5)
    cv2.imshow('out', out)


class StereoNode(Node):
    """
    Represents the front-facing stereo camera.
    
    Subscribes to the Logitech cameras.
    Publishes Toys with topic name 'toys'.
    """

    def __init__(self, rate=10, camera_info='data/camera_info/logitechC270_640x480.yaml'):
        super().__init__('stereo_node')
        self.publisher_ = self.create_publisher(Toys, '/toys', rate)
        self.capL = cv2.VideoCapture('/dev/video0')
        self.capR = cv2.VideoCapture('/dev/video2')
        if not self.capL.isOpened() or not self.capR.isOpened():
            raise IOError('Cannot open webcam')
        self.capL.set(cv2.CAP_PROP_FPS, rate)
        self.capR.set(cv2.CAP_PROP_FPS, rate)
        
        self.frame_no = 1
        self.model = Detect()
        self.stereo = cv2.StereoBM_create()
        self.stereo.setMinDisparity(0)
        self.stereo.setNumDisparities(8*16)

        with open(camera_info, 'r') as file:
            self.camera_info = yaml.safe_load(file)
        self.b = self.camera_info['camera_width'] # baseline distance
        self.f = self.camera_info['focal_length'] # focal length
        self.fx, _, self.cx = self.camera_info['camera_matrix']['data'][:3]

    def capture(self):
        while self.capL.isOpened() and self.capR.isOpened():
            retL, frameL = self.capL.read()
            retR, frameR = self.capR.read()
            stamp = self.get_clock().now()
            if retL and retR:
                self.callback(frameL, frameR, stamp)
                self.frame_no += 1

    def callback(self, frameL, frameR, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = str(self.frame_no)

        start_time = time.time()
        results = self.model.predictions(frameL)[0]  # detect toys
        boxes = results.boxes
        pub_msg = Toys()
        toy_arr = []
        for xywh, cls, conf in zip(boxes.xywh, boxes.cls, boxes.conf):
            toy_msg = Toy()
            toy_msg.cls = cls
            toy_msg.conf = conf
            toy_msg.position = Pose()
            toy_msg.x, toy_msg.y = self.calculate_position(frameL, frameR, xywh)
            toy_arr.append(toy_msg)
        end_time = time.time()
        pub_msg.header = header
        pub_msg.toys = toy_arr
        self.publisher_.publish(pub_msg)
        self.get_logger().info(f'Frame {self.frame_no}: detection time {(end_time-start_time)*1000}ms')
        display(frameL, results)

    def calculate_position(self, frameL, frameR, xywh):
        """
        Calculates position of toy given xywh bbox prediction.

        Args:
            frameL: image predicted on
            frameR: right frame
            xywh: bbox prediciton; x, y represents the center coordinates of the bbox;
            w, h represents the width and heigh of the bbox

        Returns:
            x, y: position of the toy
        """
        bx, by = xywh.cpu().numpy()[:2]  # center of bbox
        disparity = self.stereo.compute(cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY), 
                                        cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY))
        disparity = disparity.astype(np.float32)
        py = self.f * self.b / (disparity.at(by, bx)/16)
        px = (bx - self.cx) / self.fx * py - (self.b / 2)
        return px, py


def main():
    rclpy.init()
    stereo_node = StereoNode()
    rclpy.spin(stereo_node)
    stereo_node.capture()
    stereo_node.destroy_node()
    rclpy.shutdown()
