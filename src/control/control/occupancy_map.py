import numpy as np
import matplotlib.pyplot as plt
from .router import *
from dora_msgs.msg import Map


class OccupancyMap:
    """
    Represents an occupancy map based on obstacle coordinates and resolution.
    nb: The coords are in the form [x, y].T or (y, x)

    Attributes:
    - offset: numpy.ndarray - The offset of the occupancy map.
    - pointclouds: list - The coordinates of the obstacles.
    - resolution: float - The resolution of the occupancy map.
    """

    def __init__(
        self,
        offset: np.ndarray,
        pointclouds: list["PointCloud"],
        resolution: float = 0.05,
    ):
        """
        Initialises an occupancy map based on the given obstacle coordinates and resolution.

        Params:
            offset (np.ndarray): The offset of the occupancy map, given (x, y).
            pointclouds (list[np.ndarray]): The coordinates of the obstacles, given [ox, oy].T.
            resolution (float, optional): Resolution of the occupancy map. Defaults to 0.02.

        Returns:
            None
        """
        # explanatory
        self.offset = offset
        self.resolution = resolution
        self.pointclouds = (pointclouds
                            if not isinstance(pointclouds, PointCloud) else
                            [pointclouds])
        if len(self.pointclouds) == 0:
            self.min = np.array([0, 0])
            self.max = np.array([0, 0])
            self.shape = np.array([0, 0])
            self.map = np.array([[]])
            return
        # Gets the minimum and maximum x and y coordinates of the obstacles
        mins = np.array([c.min for c in self.pointclouds])
        maxs = np.array([c.max for c in self.pointclouds])

        self.min = np.array([min(mins[:, 0]), min(mins[:, 1])])
        self.max = np.array([max(maxs[:, 0]), max(maxs[:, 1])])

        # Calculate the shape of the occupancy map
        self.shape = np.round((self.max - self.min) / resolution).astype(int)

        # Generate the occupancy map with probability 0.5 in each cell
        self.map = np.ones(self.shape) * 0.5

    def generate(self, fuzz: bool = True) -> "OccupancyMap":
        """
        Generates an occupancy map based on the given obstacle coordinates.

        Params:
            fuzz (bool): Whether to apply a fuzzy filter to the occupancy map. Default is True.

        Returns:
            map (numpy.ndarray): Occupancy map representing the environment, where 0.0 represents free area and 1.0 represents occupied area.
        """

        # Draw Empty Space
        for cloud in self.pointclouds:
            o = self.translate(cloud.origin)  # Normalise the origin
            for p in cloud.cloud():
                i = self.translate(p)  # Normalise the point
                # Draw ray between origin and point
                line = bresenham(o, i)
                for pl in line:
                    try:
                        self.map[int(pl[0])][int(pl[1])] = 0  # free area 0.0
                    except IndexError:
                        pass

        # Draw on Obstacles
        wallThickness = 3
        for cloud in self.pointclouds:
            for p in cloud.objectCloud:
                i = self.translate(p, True)

                for w in range(wallThickness):
                    try:
                        prob = 1
                        # extend the occupied area
                        self.map[i[0] + w][i[1]] = prob
                        # extend the occupied area
                        self.map[i[0]][i[1] + w] = prob
                        # extend the occupied area
                        self.map[i[0] + w][i[1] + w] = prob
                    except IndexError:
                        pass

        # Apply Fuzzy Filter
        self.map = man_fuzz(self.map) if fuzz else self.map

        return self

    def merge(self, others: "OccupancyMap") -> "OccupancyMap":
        """
        Merges the current OccupancyMap with another OccupancyMap.

        Params:
            other (OccupancyMap): The OccupancyMap to merge with.

        Returns:
            OccupancyMap: The merged OccupancyMap.
        """
        if isinstance(others, OccupancyMap):
            others = [others]

        for other in others:
            newPointcloud = self.pointclouds

            # Normalise the other pointclouds
            for cloud in other.pointclouds:
                cloud.transform(cloud.origin - self.offset)
                # TODO: find efficient method of removing duplicates
                # cloud.removeDuplicates(newPointcloud)

                if cloud.isEmpty():
                    continue
                newPointcloud.append(cloud)

        # rerolls the offset so it doesn't get rolled twice
        self.__init__(self.offset, newPointcloud)
        return self

    def sampleCoord(self,
                    coord: np.ndarray,
                    yx=False,
                    mean=False,
                    n=3) -> np.ndarray:
        """
        Sample the occupancy map at a given coordinate.

        Params:
            coord (np.ndarray): The coordinate to sample.
            yx (bool, optional): If True, the coordinate is treated as (y, x) instead of (x, y). Defaults to False.

        Returns:
            np.ndarray: The sampled value, nan if the coordinate is out of bounds.
        """
        try:
            coord = self.translate(coord)
            if not mean:
                return self.map[coord[0], coord[1]]
            else:
                return (1 if 1 in self.map[coord[0] - n:coord[0] + n,
                                           coord[1] - n:coord[1] + n] else
                        np.mean(self.map[coord[0] - n:coord[0] + n,
                                         coord[1] - n:coord[1] + n]))
        except IndexError:
            return 1

    def translate(self, coord: np.ndarray) -> np.ndarray:
        """
        Translates a coordinate to a grid index.

        Params:
            coord (float): The coordinate value to be translated.
            
        Returns:
            int: The grid index corresponding to the translated coordinate.
        """
        return np.round(((coord - self.offset) - self.min) /
                            self.resolution).astype(int)

    def normalise(self) -> None:
        """
        Normalizes the pointclouds by applying the offset.
        """
        for c in self.pointclouds:
            c.transform(self.offset)
        self.offset = np.array([0, 0])

    def to_msg(self) -> "Map":
        """
        Converts the OccupancyMap to a ROS message.

        Returns:
            Map: The ROS message representing the OccupancyMap.
        """
        msg = Map()
        msg.offset.x = self.offset[0]
        msg.offset.y = self.offset[1]
        msg.resolution = self.resolution
        msg.clouds = [c.toMsg() for c in self.pointclouds]
        return msg
    
    def from_msg(msg: "Map") -> "OccupancyMap":
        """
        Converts a ROS message to an OccupancyMap.

        Params:
            msg (Map): The ROS message to be converted.

        Returns:
            OccupancyMap: The OccupancyMap representing the ROS message.
        """
        pos = np.array([msg.offset.x, msg.offset.y])
        rot = msg.offset.rot
        clouds = [PointCloud.fromMsg(c) for c in msg.clouds]
        res = msg.resolution
        return OccupancyMap(pos, clouds, res)
    
    
    def show(
        self,
        raycast: bool = False,
        region: np.ndarray = None,
        save: bool = False,
        path: str = "",
    ) -> None:
        """
        Display the occupancy map using matplotlib.

        Params:
            raycast (bool, optional): If True, displays the occupancy map with raycast visualization. Defaults to False.
            region (np.ndarray, optional): The region to be plotted in the raycast visualization. Defaults to None.
        """
        plt.figure(1, figsize=(4, 4))
        if not raycast:
            plt.imshow(self.map, cmap="PiYG_r")
            plt.clim(0, 1)
            plt.gca().set_xticks(np.arange(-0.5, self.shape[1], 1), minor=True)
            plt.gca().set_yticks(np.arange(-0.5, self.shape[0], 1), minor=True)
            plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
            plt.colorbar()
        else:
            if not region is None:
                plt.plot(region[:, 0], region[:, 1], "bo-")

            for cloud in self.pointclouds:
                if len(cloud.objectCloud) == 0:
                    continue

                c = cloud.objectCloud + self.offset
                plt.plot(
                    [c[:, 0], self.offset[0] + np.zeros(np.size(c[:, 0]))],
                    [c[:, 1], self.offset[1] + np.zeros(np.size(c[:, 1]))],
                    "ro-",
                )
            plt.axis("equal")
            plt.plot(self.offset[1], self.offset[0], "ob")
            plt.gca().set_aspect("equal", "box")
            bottom, top = plt.ylim()  # return the current y-lim
            # rescale y axis, to match the grid orientation
            plt.ylim((top, bottom))
            plt.grid(True)
        if save:
            plt.savefig(path)
            plt.close()
        else:
            plt.show()