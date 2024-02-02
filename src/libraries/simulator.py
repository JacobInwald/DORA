import numpy as np
import matplotlib.pyplot as plt
import shutil
import glob
from natsort import natsorted
from PIL import Image

# ! Library Methods

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
    return np.linalg.norm(a - c) + np.linalg.norm(b - c) - np.linalg.norm(a - b) < 0.0001


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


def lineIntersectPolygon(p: np.ndarray, d: np.ndarray, bounds: np.ndarray) -> np.ndarray:
    """
    Calculate the intersection point between a line and a polygon.

    Params:
    p (numpy.ndarray): The starting point of the line.
    d (numpy.ndarray): The direction vector of the line.
    bounds (numpy.ndarray): The polygon to check for intersection.

    Returns:
    numpy.ndarray: The intersection point of the line and the polygon.
    """
    intersects = []
    
    for bound in bounds:
        for i in range(len(bound)):
            # Get a bounding line of the boundary region
            p_bound = bound[i]
            d_bound = np.round(bound[i+1] - p_bound if i < len(bound)-1 else bound[0] - p_bound, 3)
            
            #  Get intersection point between the LiDAR ray and the bounding line
            intersect = pointIntersect(p, d, p_bound, d_bound)
            
            # Checks if the intersection point is valid (pointing right way, not nan, not too far away,...)
            if np.isnan(intersect).any() or  \
                not isBetween(p_bound, p_bound+d_bound, intersect) or \
                np.dot(d, intersect - p) < 0 or \
                np.linalg.norm(intersect - p) > np.linalg.norm(d):
                continue
            
            # Update minimum distance
            intersects.append(intersect)
       

    return intersects


def lidarRay(start: np.ndarray, angle: float, bounds: np.ndarray[np.ndarray], noise=True, max_dist=np.inf) -> float:
    """
    Calculates the minimum distance from a starting point to a boundary region
    along a given angle using LiDAR.

    Params:
        start (np.ndarray): The starting point coordinates.
        angle (float): The angle in radians.
        bounds (np.ndarray): The boundary region coordinates.

    Returns:
        float: The minimum distance from the starting point to the boundary region.
        nb: return inf if there is no intersection
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
            
    noise = np.random.normal(0, 0.01) if noise else 0
    return min_dist + noise if min_dist < max_dist else np.inf


def folderToGIF(dir: str):
    # filepaths
    fp_in = f"{dir}/*.png"
    fp_out = f"{dir.split('/')[-1]}.gif"

    frames = natsorted([file for file in glob.glob(fp_in)])
    frames = [Image.open(frame) for frame in frames]
    shutil.rmtree(dir)
    frame_one = frames[0]
    frame_one.save(fp_out, format="GIF", append_images=frames,
               save_all=True, duration=100, loop=0)


# ! Controller Simulation

class Controller:
    
    def __init__(self, pos: np.ndarray, rot: float, map: list[np.ndarray], scan_res: float = 1, noise: bool = True, max_scan_dist: float = 2):
        """
        Initialize the controller.

        Params:
            pos (np.ndarray): The initial position of the robot.
            rot (float): The initial rotation of the robot in radians.
            map (list[np.ndarray]): The map of the environment.
            noise (bool, optional): Whether to add noise to the LiDAR scan results. Defaults to True.
            max_scan_dist (float, optional): The maximum distance to scan with the LiDAR. Defaults to 2.
        """
        self.pos = np.array(pos).astype(float)
        self.rot = rot
        self.map = map
        self.scan_res = scan_res
        self.noise = noise
        self.max_scan_dist = max_scan_dist
        

    def forward(self, dist: float):
        """
        Move the robot forward by a given distance.

        Params:
            dist (float): The distance to move the robot forward.
        """
        move_dir = np.array([np.cos(self.rot), np.sin(self.rot)]) * dist

        if not np.isnan(lineIntersectPolygon(self.pos, move_dir, self.map)).all():
            print("Collision detected, cannot move forward.")
            return False
        
        self.pos += move_dir
        return True

    
    def turn(self, angle: float, deg: bool = False):
        """
        Turn the robot by a given angle.

        Params:
            angle (float): The angle to turn the robot by in radians.
        """
        if deg:
            angle = np.deg2rad(angle)
        self.rot += angle % (2 * np.pi)

    
    def getLiDARScan(self):
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
        angles = [np.deg2rad(i) for i in np.arange(0, 360, self.scan_res)]
        
        # Get lidar scan results for each angle
        scan = [np.array([a, lidarRay(self.pos, a, self.map, self.noise, self.max_scan_dist)]) for a in angles]
                
        return np.array(scan)


    def show(self, save:bool = False, path:str = ""):
        plt.figure(1, figsize=(4, 4))
        for region in self.map:
            r = np.append(region, [region[0]], axis=0)
            plt.plot(r[:,0], r[:,1], "bo-")
        
        plt.arrow(self.pos[0], self.pos[1], 0.15 * np.cos(self.rot), 0.15 * np.sin(self.rot), head_width=0.15, head_length=0.15, fc="r", ec="r")
        plt.axis("equal")
        plt.gca().set_aspect("equal", "box")
        bottom, top = plt.ylim()  # return the current y-lim
        plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
        if save:
            plt.savefig(path)
            plt.close()
        else:
            plt.show()