import rclpy
from simulation_ros import LiDARPublisher, GPSPublisher, MoveSubscriber
import numpy as np

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
    return (
        np.linalg.norm(a - c) + np.linalg.norm(b - c) -
        np.linalg.norm(a - b) < 0.0001
    )


def pointIntersect(
    p1: np.ndarray, d1: np.ndarray, p2: np.ndarray, d2: np.ndarray
) -> np.ndarray:
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
        t = ((p2[1] - p1[1]) + (d2[1] / d2[0]) * (p1[0] - p2[0])) / (
            d1[1] - (d2[1] * d1[0]) / d2[0]
        )

    p = p1 + t * d1

    if any(np.isnan(p)) or any(np.isinf(p)):
        return np.array([np.nan, np.nan])

    return p


def lineIntersectPolygon(
    p: np.ndarray, d: np.ndarray, bounds: np.ndarray
) -> np.ndarray:
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
            d_bound = np.round(
                bound[i + 1] -
                p_bound if i < len(bound) - 1 else bound[0] - p_bound, 3
            )

            #  Get intersection point between the LiDAR ray and the bounding line
            intersect = pointIntersect(p, d, p_bound, d_bound)

            # Checks if the intersection point is valid (pointing right way, not nan, not too far away,...)
            if (
                np.isnan(intersect).any()
                or not isBetween(p_bound, p_bound + d_bound, intersect)
                or np.dot(d, intersect - p) < 0
                or np.linalg.norm(intersect - p) > np.linalg.norm(d)
            ):
                continue

            # Update minimum distance
            intersects.append(intersect)

    return intersects


# ! Simulation Classes
    
class Map:
    
    def __init__(self, regions) -> None:
        self.regions = regions
    
    
    def line_intersects_region(self, p1, p2):
        """
        Check if a line intersects with any of the regions in the map.
        """
        return lineIntersectPolygon(p1, p2, self.regions)
    
    def cast_ray(self,
        start: np.ndarray,
        angle: float,
        noise=True,
        max_dist=np.inf,
    ) -> float:
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

        dir_lidar = np.round(np.array([np.cos(angle), np.sin(angle)]), 3)
        min_dist = np.inf

        for bound in self.regions:
            for i in range(len(bound)):
                # Get a bounding line of the boundary region
                p1 = bound[i]
                dir = np.round(
                    bound[i + 1] - p1 if i < len(bound) - 1 else bound[0] - p1, 3
                )

                #  Get intersection point between the LiDAR ray and the bounding line
                p = pointIntersect(start, dir_lidar, p1, dir)

                # Checks if the intersection point is valid (pointing right way, not nan, not too far away,...)
                if (
                    np.isnan(p).any()
                    or not isBetween(p1, p1 + dir, p)
                    or np.dot(dir_lidar, p - start) < 0
                    or np.linalg.norm(p - start) >= min_dist
                ):
                    continue

                # Update minimum distance
                min_dist = np.linalg.norm(p - start)

        noise = np.random.normal(0, 0.01) if noise else 0
        return min_dist + noise if min_dist < max_dist else np.inf
          
class Simulation:
    
    def __init__(self, pos, region) -> None:
        # Lidar
        self.lidar_publisher = LiDARPublisher(self)
        # GPS
        self.gps_publisher = GPSPublisher(self)
        # Map
        self.map = Map(region)
        # Move Subscriber
        self.move_subscriber = MoveSubscriber(self)
        # Position
        self.pos = pos
        # Rotation
        self.rot = 0

    def forward(self, dist: float):
        """
        Move the robot forward by a given distance.

        Params:
            dist (float): The distance to move the robot forward.
        """
        move_dir = np.array([np.cos(self.rot), np.sin(self.rot)]) * dist

        if len(self.map.line_intersects_region(self.pos, move_dir)) > 0:
            print("Collision detected, cannot move forward.")
            return False

        self.pos += move_dir
        print(f"DORA is at {self.pos} with angle of {np.rad2deg(self.rot)}")
        return True

    def turn(self, angle: float, deg: bool = True):
        """
        Turn the robot by a given angle.

        Params:
            angle (float): The angle to turn the robot by in radians.
            deg (bool, optional): Whether the angle is given in degrees. Defaults to False.
        """
        if deg:
            angle = np.deg2rad(angle)
        self.rot = (self.rot + angle) % (2 * np.pi)

    def spin(self):
        """
        Spins the robot.
        """
        while(True):
            rclpy.spin_once(self.lidar_publisher, timeout_sec=0.01)
            rclpy.spin_once(self.gps_publisher, timeout_sec=0.01)
            rclpy.spin_once(self.move_subscriber, timeout_sec=0.01)
        rclpy.shutdown()
    
    
    def destroy(self):
        self.lidar_publisher.destroy()
        self.gps_publisher.destroy()
        rclpy.shutdown()


    def get_pos(self):
        return self.pos
    
    def get_rot(self):
        return self.rot
    
    def get_map(self):
        return self.map

def spin():
    rclpy.init()
    region = [np.array([[-2, 4], [3, 4], [2,2], [4, 3], [4, 0], [4, 0], [2, -1], [-2, 0]]), \
                    np.array([[-1,3],[-1,2.5],[-1.5,3]])]
    res = 5    # Resolution of LIDAR scans
    max_dist = 1.5 # max distance of LIDAR scans
    sim = Simulation(np.array([0.0, 0.0]), region)
    sim.spin()
    
    sim.destroy()

spin()
