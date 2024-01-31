import numpy as np
from matplotlib import pyplot as plt
from collections import deque


# ! Library Methods

def man_fuzz(grid: np.ndarray) -> np.ndarray:
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

    Args:
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
    
    def __init__(self, offset: np.ndarray, pointclouds: list[np.ndarray], xy_resolution: float=0.05):
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
            self.pointclouds = pointclouds
            # Gets the minimum and maximum x and y coordinates of the obstacles
            self.mins = np.array([[min(c[:,0]) - self.EXTEND_AREA * 0.5,
                                   min(c[:,1]) - self.EXTEND_AREA * 0.5] 
                                  for c in pointclouds])
            self.maxs = np.array([[max(c[:,0]) + self.EXTEND_AREA * 0.5,
                                   max(c[:,1]) + self.EXTEND_AREA * 0.5] 
                                  for c in pointclouds])
            # Gets a point in the center of each point cloud
            self.centers = np.array([np.round((max - min) / (2*xy_resolution)).astype(int) for min, max in zip(self.mins, self.maxs)])
            # Calculate the shape of the occupancy map
            self.shape = np.round(np.array([((self.maxs[:,0].max() - self.mins[:,0].min())),
                          ((self.maxs[:,1].max() - self.mins[:,1].min()))]) / xy_resolution).astype(int)
            # Generate the occupancy map with probability 0.5 in each cell
            self.occupancy_map = np.ones(self.shape) * 0.5


    def generate(self, fuzz: bool = False):
            """
            Generates an occupancy map based on the given obstacle coordinates.

            Params:
            - fuzz (bool): Whether to apply a fuzzy filter to the occupancy map. Default is False.
            
            Returns:
            - occupancy_map (numpy.ndarray): Occupancy map representing the environment, where 0.0 represents free area and 1.0 represents occupied area.
            """
        
            # Find the bounding box of the occupied area
            for min, center, cloud in zip(self.mins, self.centers, self.pointclouds):
                prev = cloud[-1]
                prev = np.round(((prev - min) / self.xy_resolution), 0).astype(int)
                cloud_map = np.ones(self.shape)
                
                for p in cloud:
                    # x, y coordinates of the the occupied area
                    i = np.round(((p - min) / self.xy_resolution), 0).astype(int)
                    line = bresenham(prev, i)

                    for p_l in line:
                        try:
                            cloud_map[int(p_l[0])][int(p_l[1])] = 0  # free area 0.0
                        except IndexError:
                            pass
                    
                    prev = i
                    
                # Flood Fill
                cloud_map = floodfill(center, cloud_map)
                self.occupancy_map = np.multiply(self.occupancy_map, cloud_map)

            # Draw on Obstacles
            for min, cloud in zip(self.mins, self.pointclouds):
                for (x, y) in cloud:
                    ix = int(round((x - min[0]) / self.xy_resolution))
                    iy = int(round((y - min[1]) / self.xy_resolution))
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


    def merge(self, other: "OccupancyMap"):
            """
            Merges the current OccupancyMap with another OccupancyMap.

            Params:
                other (OccupancyMap): The OccupancyMap to merge with.

            Returns:
                OccupancyMap: The merged OccupancyMap.
            """
            
            # Convert to normal list to allow appending
            new_pointcloud = self.pointclouds

            f = lambda p: any([p in c for c in new_pointcloud])
            
            # Normalise the other pointclouds
            for cloud in other.pointclouds:
                c = cloud + other.offset - self.offset
                filter_arr = [not f(p) for p in c]
                new_pointcloud.append(c[filter_arr])

            
            # rerolls the offset so it doesn't get rolled twice
            return OccupancyMap(np.roll(self.offset,1), new_pointcloud)
    
    
    def sample_coord(self, coord: np.ndarray) -> np.ndarray:
            """
            Sample the occupancy map at a given coordinate.

            Params:
                coord (np.ndarray): The coordinate to sample.

            Returns:
                np.ndarray: The sampled value, nan if the coordinate is out of bounds.
            """
            coord = np.round((((np.roll(coord,1)) - self.offset) - self.mins[0]) / self.xy_resolution , 0).astype(int)
            try:
                return self.occupancy_map[coord[0], coord[1]]
            except IndexError:
                return np.array(np.nan, np.nan)
    
    
    def normalise(self):
        for c in self.pointclouds:
            c += self.offset
        self.offset = np.array([0, 0])
            
    
    def show(self, raycast: bool=False, region: np.ndarray=None):
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
        

# ! Simulation Methods

def isBetween(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
    """
    Check if point c lies between points a and b.

    Params:
    a (numpy.ndarray): The first point.
    b (numpy.ndarray): The second point.
    c (numpy.ndarray): The point to check.

    Returns:
    bool: True if c lies between a and b, False otherwise.
    """
    return np.linalg.norm(a - c) + np.linalg.norm(b - c) - np.linalg.norm(a - b) < 0.00001


def pointIntersect(p1: np.ndarray, d1: np.ndarray, p2: np.ndarray, d2: np.ndarray) -> np.ndarray:
    """
    Calculate the intersection point between two lines defined by a point and a direction vector.

    Params:
    p1 (numpy.ndarray): The starting point of the first line.
    d1 (numpy.ndarray): The direction vector of the first line.
    p2 (numpy.ndarray): The starting point of the second line.
    d2 (numpy.ndarray): The direction vector of the second line.

    Returns:
    numpy.ndarray: The intersection point of the two lines.
    nb: this return nan if the lines are parallel
    """
    if d2[0] == 0:
        t = (p2[0] - p1[0]) / d1[0]
    else:
        t = ((p2[1] - p1[1]) + (d2[1]/d2[0])*(p1[0] - p2[0])) / (d1[1] - (d2[1]*d1[0]) / d2[0])
        
    p = p1 + t * d1
    
    if any(np.isnan(p)) or any(np.isinf(p)):
        return np.array([np.nan, np.nan])

    return p


def lidarRay(start: np.ndarray, angle: float, bounds: np.ndarray[np.ndarray]) -> float:
    """
    Calculates the minimum distance from a starting point to a boundary region
    along a given angle using LiDAR.

    Params:
        start (np.ndarray): The starting point coordinates.
        angle (float): The angle in radians.
        bounds (np.ndarray): The boundary region coordinates.

    Returns:
        float: The minimum distance from the starting point to the boundary region.
        nb: return nan if there is no intersection
    """
    
    dir_lidar = np.round(np.array([np.sin(angle), np.cos(angle)]), 3)
    min_dist = np.inf

    for bound in bounds:
        for i in range(len(bound)):
            # Get a bounding line of the boundary region
            p1 = bound[i]
            dir = np.round(bound[i+1] - p1 if i < len(bound) - 1 else bound[0] - p1, 3)
            
            #  Get intersection point between the LiDAR ray and the bounding line
            p = pointIntersect(start, dir_lidar, p1, dir)
            
            # Checks if the intersection point is valid (pointing right way, not nan, not too far away,...)
            if np.isnan(p).any() or  \
                not isBetween(p1, p1+dir, p) or \
                np.dot(dir_lidar, p - start) < 0 or \
                np.linalg.norm(p - start) >= min_dist:
                continue
            
            # Update minimum distance
            min_dist = np.linalg.norm(p - start)
    
    return min_dist if min_dist != np.inf else np.nan


def lidarCircleScan(start: np.ndarray, bounds: np.ndarray[np.ndarray], res: int = 1):
    """
    Perform a circular lidar scan around a given start point within the specified bounds.

    Params:
        start (np.ndarray): The starting point of the lidar scan.
        bounds (np.ndarray): The bounds within which the lidar scan is performed.
        res (int, optional): The resolution of the lidar scan in degrees. Defaults to 1.

    Returns:
        np.ndarray: An array containing the lidar scan results.
    """
    # Init
    scan = []
    angles = [np.deg2rad(i) for i in np.arange(0, 360, res)]
    
    # Get lidar scan results for each angle
    for angle in angles:
        p = np.array([angle, lidarRay(start, angle, bounds)])
        if not np.isnan(p).any():   
            scan.append(p)
            
    return np.array(scan)


# ! TEST CODE

def test_1():
    # ! TEST CODE

    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    res = 5
    # 1
    scan_center = np.array([2, 3.5])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m = OccupancyMap(scan_center, cloud)
    m.generate()
    m.show()
    m.show(raycast=True)

    # 2
    scan_center = np.array([3.6, 2.4])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m2 = OccupancyMap(scan_center, cloud)
    m2.generate()
    m2.show()
    m2.show(raycast=True)
    
    # 3
    scan_center = np.array([0, 0])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m3 = OccupancyMap(scan_center, cloud)
    m3.generate()
    m3.show()
    m3.show(raycast=True)
    
    # 4
    scan_center = np.array([-1, 3.5])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m4 = OccupancyMap(scan_center, cloud)
    m4.generate()
    m4.show()
    m4.show(raycast=True)
     
    # 5
    m5 = m4.merge(m3).merge(m2).merge(m)
    m5.generate()
    m5.show(raycast=False, region=region[0])

def test_2(points):
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    res = 5
    # 1
    scan_center = np.array([2, 3.5])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m = OccupancyMap(scan_center, cloud)
    m.generate()

    # 2
    scan_center = np.array([3.6, 2.4])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m2 = OccupancyMap(scan_center, cloud)
    m2.generate()
    
    # 3
    scan_center = np.array([0, 0])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m3 = OccupancyMap(scan_center, cloud)
    m3.generate()
    
    # 4
    scan_center = np.array([-1, 3.5])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m4 = OccupancyMap(scan_center, cloud)
    m4.generate()
     
    # 5
    m5 = m4.merge(m3).merge(m2).merge(m)
    m5.generate()
    print([m5.sample_coord(np.array(point)) for point in points])
    
    
def test_norm():
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    res = 5
    # 1
    scan_center = np.array([2, 3.5])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m = OccupancyMap(scan_center, cloud)
    m.generate()

    # 2
    scan_center = np.array([3.6, 2.4])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m2 = OccupancyMap(scan_center, cloud)
    m2.generate()
    
    # 3
    scan_center = np.array([0, 0])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m3 = OccupancyMap(scan_center, cloud)
    m3.generate()
    
    # 4
    scan_center = np.array([-1, 3.5])
    scan = lidarCircleScan(scan_center, region, res=res)
    ox = scan[:,1] * np.cos(scan[:,0])
    oy = scan[:,1] * np.sin(scan[:,0])
    cloud = [np.array([ox, oy]).T]
    m4 = OccupancyMap(scan_center, cloud)
    m4.generate()
     
    # 5
    m5 = m4.merge(m3).merge(m2).merge(m)
    m5.generate()
    m5.normalise()
    m5.show()
    m5.show(raycast=True)
    