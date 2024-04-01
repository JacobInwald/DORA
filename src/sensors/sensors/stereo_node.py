import numpy as np
import cv2
import yaml
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from dora_msgs.msg import Toy, Pose, Toys
from object_detection.detect import Detect
from object_detection.demo import annotate
from statistics import mean, median
from .video_utils import VideoCapture, cv2_to_msg


def filter_matches(matches, kpL, kpR,
                   k=15, threshold=0.2,
                   minDisparity=0, maxDisparity=100):
    matches.sort(key=lambda x: x.distance)
    k = min(k, len(matches))
    matches = matches[:k]
    disparities = [0 for _ in range(k)]
    for i in range(k):
        ptL = kpL[matches[i].queryIdx].pt
        ptR = kpR[matches[i].trainIdx].pt
        disparities[i] = ptL[0] - ptR[0]
    matches = [matches[i] for i in range(k) if minDisparity <= disparities[i] < maxDisparity]
    disparities = list(filter(lambda x: minDisparity <= x < maxDisparity, disparities))
    if len(disparities) > 0:
        mid = median(disparities)
        matches = [matches[i] for i in range(len(matches)) if abs(abs(disparities[i]) - mid) < mid * threshold]
        disparities = list(filter(lambda x: abs(abs(x) - mid) < mid * threshold, disparities))
    return matches, disparities


class StereoNode(Node):
    """
    Represents the front-facing stereo camera.
    
    Subscribes to the Logitech cameras.
    Publishes Toys with topic name 'toys'.
    """

    def __init__(self, rate=2, show=True, detector='orb', camera_yaml='data/camera_info/logitechC270_640x480.yaml'):
        super().__init__('stereo_node')
        self.toy_pub_ = self.create_publisher(Toys, '/toys', rate)
        self.show = show
        if self.show: self.display_pub_ = self.create_publisher(Image, '/stereo', rate)
        
        self.frame_no = 1
        self.conversion = 0.001 # mm to metres
        self.model = Detect()

        self.detector = detector
        self.orb = cv2.ORB_create(1000)
        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        index_params = dict(algorithm=1, trees=5) # k-d tree
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        with open(camera_yaml, 'r') as file:
            camera_info = yaml.safe_load(file)
        self.b = camera_info['camera_width'] # baseline distance
        self.f = camera_info['focal_length'] # focal length
        self.sx = camera_info['sensor_width'] / camera_info['image_width']  # physical size of a pixel in camera sensor
        self.fx, _, self.cx = camera_info['camera_matrix']['data'][:3]

        self.capL = VideoCapture('/dev/video0')
        self.capR = VideoCapture('/dev/video2')
        if not self.capL.isOpened() or not self.capR.isOpened():
            raise IOError('Cannot open webcam')
        self.capL.set(cv2.CAP_PROP_FPS, rate)
        self.capR.set(cv2.CAP_PROP_FPS, rate)

        self.capture()

    def capture(self):
        while self.capL.isOpened() and self.capR.isOpened():
            retL, frameL = self.capL.read()
            retR, frameR = self.capR.read()
            stamp = self.get_clock().now().to_msg()
            if retL and retR:
                self.callback(frameL, frameR, stamp)
                self.frame_no += 1

    def callback(self, frameL, frameR, stamp):
        header = self.create_header(stamp)
        start_time = time.time()
        results = self.model.predictions(frameL)[0]  # detect toys
        boxes = results.boxes
        pub_msg = Toys()
        toy_arr = []
        for xywh, xyxy, cls, conf in zip(boxes.xywh, boxes.xyxy, boxes.cls, boxes.conf):
            pos = self.calculate_position(frameL, frameR, xywh, xyxy)
            if pos is not None:
                toy_arr.append(self.create_toy_msg(pos, cls, conf))
        end_time = time.time()
        pub_msg.header = header
        pub_msg.toys = toy_arr
        self.toy_pub_.publish(pub_msg)
        (self.get_logger()
         .info('Frame {}: {:.1f}ms detection'
               .format(self.frame_no,
                       (end_time-start_time)*1000)))
        if self.show: self.display(results, frameR, header)

    def create_toy_msg(self, pos, cls, conf):
        toy_msg = Toy()
        toy_msg.cls = int(cls)
        toy_msg.conf = float(conf)
        toy_msg.position = Pose()
        toy_msg.position.x = pos[0] * self.conversion
        toy_msg.position.y = pos[1] * self.conversion
        return toy_msg

    def create_header(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = str(self.frame_no)
        return header

    def calculate_position(self, frameL, frameR, xywh, xyxy):
        """
        Calculates position of toy given bbox prediction.

        Args:
            frameL: image predicted on
            frameR: right frame
            xywh: bbox prediction; x, y represents the center coordinates of the bbox;
            w, h represents the width and height of the bbox
            xyxy: bbox prediction; represents two corners of the box

        Returns:
            x, y: position of the toy in mm
        """
        bx = xywh.cpu().numpy()[0]  # center of bbox
        x1, y1, x2, y2 = list(map(round, xyxy.cpu().numpy()))
        maskL = np.zeros(frameL.shape[:2], dtype=np.uint8)
        maskL[y1:y2, x1:x2] = 1
        maskR = np.zeros(frameL.shape[:2], dtype=np.uint8)
        pad = round(frameL.shape[0] * 0.1)
        maskR[max(0, y1 - pad):min(frameL.shape[0], y2 + pad), :] = 1

        matches, kpL, kpR = self.match_frames(frameL, frameR, maskL, maskR)
        matches, disparities = filter_matches(matches, kpL, kpR)

        if len(disparities) > 0:
            disparity = mean(disparities)
            py = (self.f * self.b) / (self.sx * disparity)
            px = (bx - self.cx) / self.fx * py - (self.b / 2)
            return px, py
        return None

    def match_frames(self, frameL, frameR, maskL=None, maskR=None):
        matches = []
        if self.detector == 'orb':
            kpL, desL = self.orb.detectAndCompute(frameL, mask=maskL)
            kpR, desR = self.orb.detectAndCompute(frameR, mask=maskR)
            if desL is not None and desR is not None: 
                matches = list(self.bf.match(desL, desR))
        else:
            kpL, desL = self.sift.detectAndCompute(frameL, mask=maskL)
            kpR, desR = self.sift.detectAndCompute(frameR, mask=maskR)
            if desL is not None and desR is not None: 
                matches = list(self.flann.match(desL, desR))
        return matches, kpL, kpR
    
    def display(self, results, right, header):
        start_time = time.time()
        pred = annotate(results, thickness=2)
        annotate_time = (time.time() - start_time) * 1000
        start_time = time.time()
        frame = cv2.hconcat([pred, right])
        concat_time = (time.time() - start_time) * 1000
        msg = cv2_to_msg(frame, header)

        self.display_pub_.publish(msg)
        (self.get_logger()
         .info('Frame {}: {:.2f}ms annotate {:.2f}ms concat'
               .format(self.frame_no,
                       annotate_time,
                       concat_time)))


def main():
    rclpy.init()
    stereo_node = StereoNode()
    rclpy.spin(stereo_node)
    stereo_node.destroy_node()
    rclpy.shutdown()
