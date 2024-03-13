import numpy as np
# from dora_msgs.msg import Cloud
from utils import *
import matplotlib.pyplot as plt
class PointCloud:
    """
    Represents a point cloud generated from lidar data.

    Attributes:
    - lidar: numpy.ndarray - The lidar data.
    - origin: numpy.ndarray - The origin of the point cloud.
    - maxScanDist: float - The maximum scan distance.
    - objectCloud: list - The cloud containing points representing objects.
    - emptyCloud: list - The cloud containing points representing empty space.
    - min: numpy.ndarray - The minimum values of the point cloud.
    - max: numpy.ndarray - The maximum values of the point cloud.
    - center: numpy.ndarray - The center point of the point cloud.
    """

    # Initialization

    def __init__(self, lidar: np.ndarray, origin: np.ndarray, maxScanDist: float, rot:float = 0, res:float = 0.05):
        self.lidar = lidar
        self.origin = np.copy(origin)
        self.rot = rot
        self.maxScanDist = maxScanDist
        self.res = res
        
        self.initClouds()
        

    def initClouds(self, lidar=None, pose=None) -> np.ndarray:
        """
        Initializes the object cloud and empty cloud based on the lidar data.
        TODO: use x, y not y, x
        """
        if lidar is None:
            lidar = self.lidar
        if pose is None:
            pose = [self.origin[0], self.origin[1], self.rot]    
        rot = pose[2]
        
        oScan = np.array([p for p in lidar if np.isfinite(p).all()])
        eScan = np.array([p for p in lidar if not np.isfinite(p).all()])

        try:
            ox = oScan[:, 1] * np.cos((oScan[:, 0] + rot))
            oy = oScan[:, 1] * np.sin((oScan[:, 0] + rot))
            self.objectCloud = np.array([ox, oy]).T
        except Exception:
            self.objectCloud = np.array([])

        try:
            ex = (self.maxScanDist - 0.2) * np.cos((eScan[:, 0] + rot))
            ey = (self.maxScanDist - 0.2) * np.sin((eScan[:, 0] + rot))
            self.emptyCloud = np.array([ex, ey]).T
        except Exception:
            self.emptyCloud = np.array([])

    # Generation:
    def generate(self, rot=None, res=None) -> None:
        
        if rot is None:
            rot = self.rot
        if res is None:
            res = self.res
            
        self.initClouds(pose=[self.origin[0], self.origin[1], rot])
        
        width = int(self.maxScanDist * 2)
        w = h = int(width / res)
        
        img = np.ones((h, w)) * 0.5
        
        # Draw Empty Space
        o = (h // 2, w // 2)
        for p in self.cloud():
            p = ((p + (width / 2))/ res).astype(int)
            # Draw ray between origin and point
            line = bresenham(o, p)
            for pl in line:
                try:
                    img[int(pl[0])][int(pl[1])] = 0  # free area 0.0
                except IndexError:
                    pass

        # Draw on Obstacles
        wallThickness = max(1, int(0.15 / res))
        for p in self.objectCloud:
            p = ((p + (width / 2))/ res).astype(int)
            for w in range(wallThickness):
                try:
                    prob = 1
                    # extend the occupied area
                    img[p[0] + w][p[1]] = prob
                    # extend the occupied area
                    img[p[0]][p[1] + w] = prob
                    # extend the occupied area
                    img[p[0] + w][p[1] + w] = prob
                except IndexError:
                    pass
        
        # Apply Fuzzy Filter
        return man_fuzz(img)

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
        if len(self.objectCloud) > 0:
            self.objectCloud += offset
        if len(self.emptyCloud) > 0:
            self.emptyCloud += offset
        # self.initMinMax()
        self.initClouds()

    def rotate(self, angle: float) -> None:
        """
        Rotates the object cloud and empty cloud by the given angle.

        Parameters:
        - angle: float - The angle to rotate the point cloud.
        """
        self.rot = angle
        # self.initMinMax()
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
        # offset = np.array([msg.pose.x, msg.pose.y])
        # rot = msg.pose.rot
        # maxScanDist = msg.max_range
        # res = msg.angle_increment
        # start = msg.angle_min
        # scan = []
        # a = start
        # for i in range(len(msg.ranges)):
        #     scan.append([a, msg.ranges[i]])
        #     a += res

        # return PointCloud(np.array(scan), offset, maxScanDist)