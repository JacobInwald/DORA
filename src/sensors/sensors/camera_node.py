import cv2
import torch
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from dora_msgs.msg import Toy, Pose, Toys
from object_detection.detect import Detect
from transformers import DPTFeatureExtractor, DPTForDepthEstimation


class CameraNode(Node):
    """
    Represents the front-facing camera.

    Subscribes to the RPi camera.
    Publishes [Toy] with topic name 'toys'
    """

    def __init__(self, camera_info='camera_info/camerav2_1280x960.yaml'):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Toys, 'toys', 10)
        self.cap = cv2.VideoCapture('dev/video0')
        if not self.cap.isOpened():
            raise IOError('Cannot open webcam')
        self.model = Detect()
        self.feature_extractor = DPTFeatureExtractor.from_pretrained("Intel/dpt-large")
        self.depth_model = DPTForDepthEstimation.from_pretrained("Intel/dpt-large")
        with open(camera_info, 'r') as file:
            self.camera_info = yaml.safe_load(file)

    def capture(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            stamp = self.get_clock().now()
            if ret:
                self.callback(frame, stamp)

    def callback(self, frame, stamp):
        header = Header()
        header.stamp = stamp
        results = self.model.predictions(frame)[0]  # detect toys
        boxes = results.boxes
        pub_msg = Toys()
        toy_arr = []
        for xywh, cls, conf in zip(boxes.xywh, boxes.cls, boxes.conf):
            toy_msg = Toy()
            toy_msg.header = header
            toy_msg.cls = cls
            toy_msg.conf = conf
            toy_msg.position = Pose()
            toy_msg.x, toy_msg.y = self.estimate_position(frame, xywh)
            toy_arr.append(toy_msg)
        pub_msg.toys = toy_arr
        self.publisher_.publish(pub_msg)

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
        depth_map = self.estimate_depth(img)
        py = depth_map[by, bx]  # get depth at bbox center
        fx, _, cx = self.camera_info['camera_matrix'][:3]  # get camera focal length and center
        px = (bx - cx) / fx * py  # calculate x using similar triangles and estimated depth
        return px, py

    def estimate_depth(self, img):
        """
        Estimates depth map of image

        Args:
            img: image for depth map estimation

        Returns:
            output: estimated depth map of image
        """
        pixel_values = self.feature_extractor(img, return_tensors="pt").pixel_values
        with torch.no_grad():
            outputs = self.depth_model(pixel_values)
            predicted_depth = outputs.predicted_depth
        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=img.size[::-1],
            mode="bicubic",
            align_corners=False,
        ).squeeze()
        output = prediction.cpu().numpy()
        return output


def main():
    rclpy.init()
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.capture()
    camera_node.destroy_node()
    rclpy.shutdown()
