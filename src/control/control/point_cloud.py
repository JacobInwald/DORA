import numpy as np
from dora_msgs.msg import Cloud
from .utils import *


class PointCloud:
    """
    Represents a point cloud generated from lidar data.

    Attributes:
    - lidar: numpy.ndarray - The lidar data.
    - origin: numpy.ndarray - The origin of the point cloud.
    - maxScanDist: float - The maximum scan distance.
    - rot: float - The rotation angle of the point cloud.
    - res: float - The resolution of the point cloud.

    Methods:
    - __init__(self, lidar: np.ndarray, origin: np.ndarray, maxScanDist: float, rot: float = 0, res: float = 0.05): Initializes a PointCloud object.
    - initClouds(self, lidar=None, pose=None) -> np.ndarray: Initializes the object cloud and empty cloud based on the lidar data.
    - generate(self, rot=None, res=None, fuzz=True) -> None: Generates an image representation of the point cloud.
    - cloud(self): Returns the combined point cloud.
    - transform(self, offset: np.ndarray) -> None: Transforms the object cloud and empty cloud by adding the given offset.
    - rotate(self, angle: float) -> None: Rotates the object cloud and empty cloud by the given angle.
    - isEmpty(self, n=5): Checks if the point cloud is empty.
    """

    def __init__(self, lidar: np.ndarray, origin: np.ndarray, maxScanDist: float, rot: float = 0, res: float = 0.05):
        """
        Initializes a PointCloud object.

        Parameters:
        - lidar: numpy.ndarray - The lidar data.
        - origin: numpy.ndarray - The origin of the point cloud.
        - maxScanDist: float - The maximum scan distance.
        - rot: float - The rotation angle of the point cloud. (default: 0)
        - res: float - The resolution of the point cloud. (default: 0.05)
        """
        self.lidar = lidar
        self.origin = np.copy(np.array(origin))
        self.maxScanDist = maxScanDist
        self.rot = rot
        self.res = res

        self.emptyCloud = np.array([])
        self.objectCloud = np.array([])
        self.initClouds()

    def initClouds(self, lidar=None, pose=None) -> np.ndarray:
        """
        Initializes the object cloud and empty cloud based on the lidar data.

        Args:
            lidar (np.ndarray, optional): Lidar data. Defaults to None.
            pose (list, optional): Pose information. Defaults to None.

        Returns:
            np.ndarray: The initialized object cloud.

        Raises:
            None

        """
        # Defaults
        if lidar is None:
            lidar = self.lidar
        if pose is None:
            pose = [self.origin[0], self.origin[1], self.rot]
        rot = pose[2]

        # Split the data into object and empty clouds
        oScan = np.array([p for p in lidar if np.isfinite(p).all()])
        eScan = np.array([p for p in lidar if not np.isfinite(p).all()])

        # Initialize the object cloud
        try:
            ox = oScan[:, 1] * np.cos((oScan[:, 0] + rot))
            oy = oScan[:, 1] * np.sin((oScan[:, 0] + rot))
            self.objectCloud = np.array([ox, oy]).T
        except Exception:
            pass

        # Initialize the empty cloud
        try:
            ex = (self.maxScanDist - 0.2) * np.cos((eScan[:, 0] + rot))
            ey = (self.maxScanDist - 0.2) * np.sin((eScan[:, 0] + rot))
            self.emptyCloud = np.array([ex, ey]).T
        except Exception:
            pass

    def generate(self, rot=None, res=None, fuzz=True) -> None:
        """
        Generate a point cloud image.

        Args:
            rot (float, optional): Rotation angle. Defaults to None.
            res (float, optional): Resolution. Defaults to None.
            fuzz (bool, optional): Flag to apply fuzzy filter. Defaults to True.

        Returns:
            numpy.ndarray: Generated point cloud image.
        """
        # Defaults
        if rot is None:
            rot = self.rot
        if res is None:
            res = self.res

        # initializing
        self.initClouds(pose=[self.origin[0], self.origin[1], rot])
        width = int(self.maxScanDist * 2)  # image width
        w = h = int(width / res)  # image size
        img = np.ones((h, w)) * 0.5  # create image
        center = (h // 2, w // 2)

        # Draw Empty Space
        for p in self.cloud():
            p = ((p + (width / 2)) / res).astype(int)
            img = cv2.line(img, center, (p[1], p[0]), 0, 1)

        # Draw on Obstacles
        wall_thickness = max(1, int(0.1 / res))
        for p in self.objectCloud:
            p = ((p + (width / 2)) / res).astype(int)
            cv2.circle(img, (p[1], p[0]), wall_thickness//2, 1, -1)

        if True:
            img = cv2.blur(img, (3, 3))

        # Apply Fuzzy Filter
        return img

    # Quality of Life

    def cloud(self):
        """
        Returns the combined point cloud.
        """
        if len(self.objectCloud) == 0:
            return self.emptyCloud
        elif len(self.emptyCloud) == 0:
            return self.objectCloud
        else:
            return np.append(self.objectCloud, self.emptyCloud, axis=0)

    def transform(self, offset: np.ndarray) -> None:
        """
        Transforms the object cloud and empty cloud by adding the given offset.

        Parameters:
        - offset: numpy.ndarray - The offset to be added to the point cloud.
        """
        self.origin += offset
        self.initClouds()

    def rotate(self, angle: float) -> None:
        """
        Rotates the object cloud and empty cloud by the given angle.

        Parameters:
        - angle: float - The angle to rotate the point cloud.
        """
        self.rot = angle
        self.initClouds(self.lidar)

    def isEmpty(self, n=5):
        """
        Checks if the point cloud is empty.

        Parameters:
        - n: int - The threshold value for considering the point cloud as empty.

        Returns:
        - bool - True if the point cloud is empty, False otherwise.
        """
        return len(self.objectCloud) + len(self.emptyCloud) <= n

    # Saving and loading

    # def to_msg(self):
    #     """
    #     Converts the point cloud to a ROS message.

    #     Returns:
    #     - sensor_msgs.msg.PointCloud2 - The ROS message representing the point cloud.
    #     """
    #     msg = Cloud()

    #     msg.pose.x = self.origin[0]
    #     msg.pose.y = self.origin[1]
    #     msg.pose.rot = self.rot

    #     msg.max_range = self.maxScanDist

    #     msg.scan.header.stamp = self.get_clock().now().to_msg()
    #     msg.scan.header.frame_id = "lidar"
    #     msg.scan.angle_min = float(self.rot)
    #     msg.scan.angle_max = float((self.rot + 2 * np.pi) % 2 * np.pi)
    #     msg.scan.angle_increment = float(np.deg2rad(self.lidar.shape[0] / 360))
    #     msg.scan.ranges = [float(reading[1]) for reading in self.lidar]
    #     return msg

    # def from_msg(msg: Cloud):
    #     offset = np.array([msg.pose.x, msg.pose.y])
    #     rot = msg.pose.rot
    #     maxScanDist = msg.max_range
    #     res = msg.angle_increment
    #     start = msg.angle_min
    #     scan = []
    #     a = start
    #     for i in range(len(msg.ranges)):
    #         scan.append([a, msg.ranges[i]])
    #         a += res

    #     return PointCloud(np.array(scan), offset, maxScanDist)
