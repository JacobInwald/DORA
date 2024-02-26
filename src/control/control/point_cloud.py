import numpy as np


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

    def __init__(self, lidar: np.ndarray, origin: np.ndarray, maxScanDist: float):
        self.lidar = lidar
        self.origin = np.copy(origin)
        self.maxScanDist = maxScanDist

        self.initClouds(lidar)
        self.initMinMax()

    def initMinMax(self) -> None:
        """
        Initializes the minimum and maximum values of the point cloud.
        """
        emin = np.array([np.inf, np.inf])
        emax = np.array([-np.inf, -np.inf])
        omin = np.array([np.inf, np.inf])
        omax = np.array([-np.inf, -np.inf])

        if len(self.objectCloud) > 0:
            omin = np.array(
                [self.objectCloud[:, 0].min(), self.objectCloud[:, 1].min()])
            omax = np.array(
                [self.objectCloud[:, 0].max(), self.objectCloud[:, 1].max()])
        if len(self.emptyCloud) > 0:
            emin = np.array(
                [self.emptyCloud[:, 0].min(), self.emptyCloud[:, 1].min()])
            emax = np.array(
                [self.emptyCloud[:, 0].max(), self.emptyCloud[:, 1].max()])

        self.min = np.array([min(omin[0], emin[0]), min(omin[1], emin[1])])
        self.max = np.array([max(omax[0], emax[0]), max(omax[1], emax[1])])

        if np.isinf(self.min).any() or np.isinf(self.max).any():
            self.min = np.array([0, 0])
            self.max = np.array([0, 0])

        self.min -= 0.5
        self.max += 0.5
        self.center = np.round((self.max - self.min) / 2, 3)

    def initClouds(self, lidar) -> np.ndarray:
        """
        Initializes the object cloud and empty cloud based on the lidar data.
        TODO: use x, y not y, x
        """
        oScan = np.array([p for p in lidar if np.isfinite(p).all()])
        eScan = np.array([p for p in lidar if not np.isfinite(p).all()])

        try:
            ox = oScan[:, 1] * np.cos(oScan[:, 0])
            oy = oScan[:, 1] * np.sin(oScan[:, 0])
            self.objectCloud = np.array([ox, oy]).T
        except Exception:
            self.objectCloud = np.array([])

        try:
            ex = (self.maxScanDist - 0.2) * np.cos(eScan[:, 0])
            ey = (self.maxScanDist - 0.2) * np.sin(eScan[:, 0])
            self.emptyCloud = np.array([ex, ey]).T
        except Exception:
            self.emptyCloud = np.array([])

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

    def removeDuplicates(self, clouds: ["PointCloud"]) -> None:
        """
        Removes duplicate points from the object cloud and empty cloud based on the given list of clouds.

        Parameters:
        - clouds: list - The list of PointCloud objects to compare with.
        """

        # TODO: find efficient method of removing duplicates
        def f(p, cloud):
            return any((np.abs(c - p) < 0.001).any() for c in cloud)

        for cloud in clouds:
            for p in self.objectCloud:
                if f(p, cloud.objectCloud):
                    self.objectCloud = np.delete(
                        self.objectCloud,
                        np.where((self.objectCloud == p).all(axis=1)),
                        axis=0,
                    )
            for p in self.emptyCloud:
                if f(p, cloud.emptyCloud):
                    self.emptyCloud = np.delete(
                        self.emptyCloud,
                        np.where((self.emptyCloud == p).all(axis=1)),
                        axis=0,
                    )
        self.initMinMax()

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
        self.initMinMax()

    def isEmpty(self, n=5):
        """
        Checks if the point cloud is empty.

        Parameters:
        - n: int - The threshold value for considering the point cloud as empty.

        Returns:
        - bool - True if the point cloud is empty, False otherwise.
        """
        return len(self.objectCloud) + len(self.emptyCloud) <= n
