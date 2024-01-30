import numpy as np
from matplotlib import pyplot as plt
from collections import deque


# ! Library Methods

def man_fuzz(grid: np.ndarray) -> np.ndarray:
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
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
    
    q = deque()
    q.append(start)
    while q:
        current = q.pop()
        occupancy_map[current[0], current[1]] = 0
        for (i, j) in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                try:
                    if occupancy_map[current[0] + i, current[1] + j] == 0.5:
                        q.append(np.array([current[0] + i, current[1] + j]))
                except IndexError:
                    pass

    return occupancy_map


class OccupancyMap:
    
    EXTEND_AREA = 1.0
    
    def __init__(self, offset: np.ndarray, ox: np.ndarray, oy: np.ndarray, xy_resolution: float=0.1):
        """
        Initializes an occupancy map based on the given obstacle coordinates and resolution.

        Args:
            ox (np.ndarray): Array of x-coordinates of obstacles.
            oy (np.ndarray): Array of y-coordinates of obstacles.
            xy_resolution (float, optional): Resolution of the occupancy map. Defaults to 0.02.
            EXTEND_AREA (float, optional): Area to extend around the obstacles. Defaults to 1.0.

        Returns:
            tuple: A tuple containing the following elements:
                - occupancy_map (np.ndarray): The initialized occupancy map.
                - min_x (float): Minimum x-coordinate of the map.
                - max_x (float): Maximum x-coordinate of the map.
                - min_y (float): Minimum y-coordinate of the map.
                - max_y (float): Maximum y-coordinate of the map.
                - x_w (int): Width of the occupancy map in cells.
                - y_w (int): Height of the occupancy map in cells.
                - center_x (int): X-coordinate of the center of the map.
                - center_y (int): Y-coordinate of the center of the map.
        """
        self.offset = offset
        self.ox = ox
        self.oy = oy
        self.min = np.array([round(min(ox) - self.EXTEND_AREA / 2.0), round(min(oy) - self.EXTEND_AREA / 2.0)])
        self.max = np.array([round(max(ox) + self.EXTEND_AREA / 2.0), round(max(oy) + self.EXTEND_AREA / 2.0)])
        self.shape = np.array([int(round((self.max[0] - self.min[0]) / xy_resolution)),
                         int(round((self.max[1] - self.min[1]) / xy_resolution))])  
        self.occupancy_map = np.ones(self.shape+1) / 2  
        self.center = np.array([int(round(-self.min[0] / xy_resolution)), 
                                int(round(-self.min[1] / xy_resolution))])
        self.xy_resolution = xy_resolution 


    def generate(self, fuzz: bool = False):
        """
        Generates an occupancy map based on the given obstacle coordinates.

        Parameters:
        - fuzz (bool): Whether to apply a fuzzy filter to the occupancy map. Default is False.
        
        Returns:
        - occupancy_map (numpy.ndarray): Occupancy map representing the environment, where 0.0 represents free area and 1.0 represents occupied area.
        """
        
        prev_ix, prev_iy = self.center[0] - 1, self.center[1]
    
        # Find the bounding box of the occupied area
        for (x, y) in zip(ox, oy):
            # x, y coordinates of the the occupied area
            ix = int(round((x - self.min[0]) / self.xy_resolution))
            iy = int(round((y - self.min[1]) / self.xy_resolution))
            
            free_area = bresenham(np.array([prev_ix, prev_iy]), np.array([ix, iy]))

            for fa in free_area:
                self.occupancy_map[int(fa[0])][int(fa[1])] = 0  # free area 0.0
            
            prev_ix = ix
            prev_iy = iy
        
        # Flood Fill
        self.occupancy_map = floodfill(np.array([self.center[0], self.center[1]]), self.occupancy_map)
        
        # Draw on Obstacles
        for (x, y) in zip(ox, oy):
                ix = int(round((x - self.min[0]) / self.xy_resolution))
                iy = int(round((y - self.min[1]) / self.xy_resolution))
                self.occupancy_map[ix][iy] = 1.0  # occupied area 1.0
                self.occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
                self.occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
                self.occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
        
        # Apply Fuzzy Filter
        self.occupancy_map = man_fuzz(self.occupancy_map) if fuzz else self.occupancy_map

        return self


    def merge(self, other: "OccupancyMap", output_resolution: float = 0.1):
        # TODO: Implement merge method
        pass
    
    
    def show(self):
        """
        Display the occupancy map using matplotlib.
        """
        plt.figure(1, figsize=(4, 4))
        plt.imshow(self.occupancy_map, cmap="PiYG_r")
        plt.clim(0, 1)
        plt.gca().set_xticks(np.arange(-.5, self.shape[1], 1), minor=True)
        plt.gca().set_yticks(np.arange(-.5, self.shape[0], 1), minor=True)
        plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
        plt.colorbar()
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


def lidarRay(start: np.ndarray, angle: float, bounds: np.ndarray) -> float:
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

    for i in range(len(bounds)):
        # Get a bounding line of the boundary region
        p1 = bounds[i]
        dir = np.round(bounds[i+1] - p1 if i < len(bounds) - 1 else bounds[0] - p1, 3)
        
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


def lidarCircleScan(start: np.ndarray, bounds: np.ndarray, res: int = 1):
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

region = np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]])

scan_center = np.array([0, 0])
scan = lidarCircleScan(scan_center, region, res=1)
ox = scan[:,1] * np.cos(scan[:,0])
oy = scan[:,1] * np.sin(scan[:,0])

map = OccupancyMap(scan_center, ox, oy)
map.generate()
map.show()