import numpy as np
from matplotlib import pyplot as plt
from collections import deque
import simulator as simulate

# ! Library Methods


def man_fuzz(grid: np.ndarray) -> np.ndarray:
    """
    Applies the Manhattan Fuzz algorithm to the given grid.

    The Manhattan Fuzz algorithm calculates the mean value of each cell in the grid
    by considering its neighboring cells in the Manhattan distance.

    Params:
        grid (np.ndarray): The input grid.

    Returns:
        np.ndarray: The grid with the Manhattan Fuzz algorithm applied.
    """
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] == 1:
                continue
            try:
                grid[i, j] = np.mean([grid[i, j], grid[i - 1, j], grid[i + 1, j], grid[i, j - 1], grid[i, j + 1]])
            except IndexError:
                pass
    return grid


def bresenham(start: np.ndarray, end: np.ndarray, res: float=1) -> np.ndarray[np.float64]:
    """
    Generate a Bresenham line between two points in a grid.

    Params:
        start (np.ndarray): The starting point of the line.
        end (np.ndarray): The ending point of the line.
        res (float, optional): The resolution of the grid. Defaults to 1.

    Returns:
        np.ndarray: An array of points representing the Bresenham line.
    """
    
    direction = end - start
    length = int(np.linalg.norm(direction) / res) + 1 
    result = np.zeros((length + 1, 2))

    for i in range(0, length + 1):
        result[i] = (start + (i / length) * direction).round(int(-np.log10(res)) if res < 1 else 0) // res
    
    return result


def floodfill(start: np.ndarray, occupancy_map: np.ndarray) -> np.ndarray[np.float64]:
    """
    Perform flood fill algorithm on an occupancy grid starting from a given point.

    Parameters:
    start (np.ndarray): The starting point for the flood fill algorithm.
    occupancy_grid (np.ndarray): The occupancy grid on which the flood fill algorithm is performed.

    Returns:
    np.ndarray[np.float64]: The result of the flood fill algorithm.
    """
    grid = np.copy(occupancy_map)
    q = deque()
    q.append(start)
    while q:
        current = q.pop()
        grid[current[0], current[1]] = 0
        for (i, j) in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                try:
                    if grid[current[0] + i, current[1] + j] != 0:
                        q.append(np.array([current[0] + i, current[1] + j]))
                except IndexError:
                    pass

    return grid


class OccupancyMap:
    
    EXTEND_AREA = 1.0     
    
    def __init__(self, offset: np.ndarray, pointclouds: list[np.ndarray], origins: list[np.ndarray]=None, xy_resolution: float=0.05):
            """
            Initializes an occupancy map based on the given obstacle coordinates and resolution.

            Params:
                offset (np.ndarray): The offset of the occupancy map, given (x, y).
                pointclouds (list[np.ndarray]): The coordinates of the obstacles, given [ox, oy].T.
                xy_resolution (float, optional): Resolution of the occupancy map. Defaults to 0.02.

            Returns:
                None
            """
            
            # explanatory
            self.offset = np.roll(offset,1)
            self.xy_resolution = xy_resolution 
            self.pointclouds = pointclouds if len(pointclouds) > 0 else [pointclouds]
            self.origins = origins if origins else [self.offset]
            if len(self.origins) != len(self.pointclouds):
                raise ValueError("The number of origins must match the number of pointclouds.")
            
            # Gets the minimum and maximum x and y coordinates of the obstacles
            try:
                mins = np.array([[min(c[:,0]) - self.EXTEND_AREA * 0.5,
                                    min(c[:,1]) - self.EXTEND_AREA * 0.5] 
                                    for c in self.pointclouds])
                maxs = np.array([[max(c[:,0]) + self.EXTEND_AREA * 0.5,
                                   max(c[:,1]) + self.EXTEND_AREA * 0.5] 
                                  for c in self.pointclouds])
            except Exception:
                mins = np.array([[0,0]])
                maxs = np.array([[0,0]])

            self.min = np.array([min(mins[:,0]), min(mins[:,1])])
            self.max = np.array([max(maxs[:,0]), max(maxs[:,1])])
            
            # Gets a point in the center of each point cloud
            self.centers = np.array([np.round((max - min) / (2*xy_resolution)).astype(int) for min, max in zip(mins, maxs)])
            
            # Calculate the shape of the occupancy map
            self.shape = np.round((self.max - self.min) / xy_resolution, 3).astype(int)
            # Generate the occupancy map with probability 0.5 in each cell
            self.occupancy_map = np.ones(self.shape) * 0.5


    def generate(self, floodfill: bool = False, fuzz: bool = True) -> "OccupancyMap":
            """
            Generates an occupancy map based on the given obstacle coordinates.

            Params:
            - fuzz (bool): Whether to apply a fuzzy filter to the occupancy map. Default is False.
            
            Returns:
            - occupancy_map (numpy.ndarray): Occupancy map representing the environment, where 0.0 represents free area and 1.0 represents occupied area.
            """
            if floodfill:
                for origin, cloud in zip(self.origin, self.pointclouds):
                    prev = cloud[-1]
                    prev = np.round(((prev - self.min) / self.xy_resolution), 0).astype(int)
                    cloud_map = np.ones(self.shape)
                    
                    for p in cloud:
                        # x, y coordinates of the the occupied area
                        i = np.round(((p - self.min) / self.xy_resolution), 0).astype(int)
                        line = bresenham(prev, i)

                        for p_l in line:
                            try:
                                cloud_map[int(p_l[0])][int(p_l[1])] = 0  # free area 0.0
                            except IndexError:
                                pass
                        
                        prev = i
                        
                    # Flood Fill
                    cloud_map = floodfill(self.translate(origin, True), cloud_map)
                    self.occupancy_map = np.multiply(self.occupancy_map, cloud_map)
            else:
                for origin, cloud in zip(self.origins, self.pointclouds):
                    o = self.translate(origin, True)    # Normalise the origin
                    for p in cloud:
                        # x, y coordinates of the the occupied area
                        i = np.round(((p - self.min) / self.xy_resolution), 0).astype(int)
                        line = bresenham(o, i)

                        for p_l in line:
                            try:
                                self.occupancy_map[int(p_l[0])][int(p_l[1])] = 0  # free area 0.0
                            except IndexError:
                                pass
                        
                        prev = i
                        
            # Draw on Obstacles
            for cloud in self.pointclouds:
                    for (x, y) in cloud:
                        ix = int(round((x - self.min[0]) / self.xy_resolution))
                        iy = int(round((y - self.min[1]) / self.xy_resolution))
                        try:
                            self.occupancy_map[ix][iy] = 1.0  # occupied area 1.0
                            self.occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
                            self.occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
                            self.occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
                        except IndexError:
                            pass
                
            # Apply Fuzzy Filter
            self.occupancy_map = man_fuzz(self.occupancy_map) if fuzz else self.occupancy_map

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

                new_origins = self.origins
                new_pointcloud = self.pointclouds

                f = lambda p: any((np.abs(c - p) < 0.001).any() for c in new_pointcloud)
                
                # Normalise the other pointclouds
                for origin, cloud in zip(other.origins, other.pointclouds):

                    c = cloud + other.offset - self.offset if cloud != [] else []
                    filter_arr = [not f(p) for p in c]
                    c = c[filter_arr]
                    
                    if len(c) <= 2:
                        continue
                    
                    new_pointcloud.append(c)
                    new_origins.append(origin)
            
            # Maybe try merging pointclouds with similar origins (some tolerance)
            
            # rerolls the offset so it doesn't get rolled twice
            self.__init__(np.roll(self.offset,1), new_pointcloud, new_origins)
            return self
    
    
    def sample_coord(self, coord: np.ndarray, yx=False) -> np.ndarray:
            """
            Sample the occupancy map at a given coordinate.

            Params:
                coord (np.ndarray): The coordinate to sample.
                yx (bool, optional): If True, the coordinate is treated as (y, x) instead of (x, y). Defaults to False.
                
            Returns:
                np.ndarray: The sampled value, nan if the coordinate is out of bounds.
            """
            try:
                coord = self.translate(coord, yx)
                return self.occupancy_map[coord[0], coord[1]]
            except IndexError:
                return np.array(np.nan, np.nan)
    
    
    def translate(self, coord: np.ndarray, yx=False) -> np.ndarray:
        """
        Translates a coordinate to a grid index.

        Params:
            coord (float): The coordinate value to be translated.
            yx (bool, optional): If True, the coordinate is treated as (y, x) instead of (x, y). Defaults to False.

        Returns:
            int: The grid index corresponding to the translated coordinate.
        """
        if yx:
            return np.round(((coord - self.offset) - self.min) / self.xy_resolution , 0).astype(int)
        else:
            return np.round(((coord - self.offset) - self.min) / self.xy_resolution , 0).astype(int)
    
    
    def normalise(self) -> None:
        for c in self.pointclouds:
            c += self.offset
        self.offset = np.array([0, 0])
            
    
    def show(self, raycast: bool=False, region: np.ndarray=None) -> None:
        """
        Display the occupancy map using matplotlib.
        """
        plt.figure(1, figsize=(4, 4))
        if not raycast:
            plt.imshow(self.occupancy_map, cmap="PiYG_r")
            plt.clim(0, 1)
            plt.gca().set_xticks(np.arange(-.5, self.shape[1], 1), minor=True)
            plt.gca().set_yticks(np.arange(-.5, self.shape[0], 1), minor=True)
            plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
            plt.colorbar()
        else:
            if not region is None:
                plt.plot(region[:,0], region[:,1], "bo-")
            
            for cloud in self.pointclouds:
                c = cloud + self.offset
                plt.plot([c[:, 1], self.offset[1] + np.zeros(np.size(c[:, 1]))], [c[:, 0], self.offset[0] + np.zeros(np.size(c[:, 0]))], "ro-")
            plt.axis("equal")
            plt.plot(self.offset[1], self.offset[0], "ob")
            plt.gca().set_aspect("equal", "box")
            bottom, top = plt.ylim()  # return the current y-lim
            plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
            plt.grid(True)
        plt.show()

    
    # ! Static Methods
    def LiDARtoCloud(scan) -> np.ndarray:
        # TODO: Change how pointclouds are handled.
        scan = np.array([p for p in scan if np.isfinite(p).all()])
        if len(scan) == 0:
            return []
        ox = scan[:,1] * np.cos(scan[:,0])
        oy = scan[:,1] * np.sin(scan[:,0])
        return np.array([ox, oy]).T


# TODO: Add support for empty reads, i.e. an empty read means the robot has no obstacles in all directions

# ! TEST CODE

def test_1():
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    res = 1
    max_dist = 10
    controller = simulate.Controller(np.array([0, 0]), 0, region, res, max_scan_dist=max_dist)

    cloud = OccupancyMap.LiDARtoCloud(controller.getLiDARScan())
    m = OccupancyMap(controller.pos, [cloud])
    m.generate()
    m.show()
    
    for dist in np.arange(1, 5, 1):
        for angle in range(0, 360, 10):
            controller.forward(dist)
            controller.turn(angle, deg=True)
            cloud = OccupancyMap.LiDARtoCloud(controller.getLiDARScan())
            m.merge(OccupancyMap(controller.pos, [cloud]))
    m.generate()
    m.show(raycast=True)
    m.show()

test_1()