from rclpy.node import Node
import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from std_msgs.msg import _header
import numpy as np
from enum import Enum
import threading
import asyncio
import rclpy
from rclpy.node import Node

# ! Library Methods

async def spin_node(node: Node):
    cancel = node.create_guard_condition(lambda: None)
    def _spin(node: Node,
              future: asyncio.Future,
              event_loop: asyncio.AbstractEventLoop):
        while not future.cancelled():
            rclpy.spin_once(node)
        if not future.cancelled():
            event_loop.call_soon_threadsafe(future.set_result, None)
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(target=_spin, args=(node, spin_task, event_loop))
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)

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


# ! Publishers

class LiDARPublisher(Node):
    """
    Represents the LiDAR.
    """

    def __init__(self, simulation, noise=True, max_scan_dist=1.5, scan_res=10) -> None:
        super().__init__("lidar_publisher")
        self.publisher_ = self.create_publisher(LaserScan, "lidar", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish)
        self.i = 0
        self.simulation = simulation
        self.noise = noise
        self.max_scan_dist = max_scan_dist
        self.scan_res = scan_res
        
    def publish(self) -> None:
        """
        Publishes the scan to the map.
        """
        pos = self.simulation.get_pos()
        map = self.simulation.get_map()
        scan = self.getLiDARScan(pos, map, self.noise, self.max_scan_dist, self.scan_res)
        # Generate the LiDAR scan data
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar"
        msg.angle_min = float(0)
        msg.angle_max = float(2 * np.pi)
        msg.angle_increment = float(np.deg2rad(self.scan_res))
        msg.ranges = [float(reading[1]) for reading in scan]
        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: LiDAR scan %d" % self.i)
        self.i += 1
    
    def getLiDARScan(self, pos, map, noise, max_scan_dist, scan_res) -> np.ndarray:
        """
        Perform a circular LiDAR scan around a given start point within the specified bounds.

        Returns:
            np.ndarray: An array containing the LiDAR scan results.
        """
        angles = [np.deg2rad(i) for i in np.arange(0, 360, scan_res)]
        scan = [
                [a, map.cast_ray(pos, a, noise, max_scan_dist)]
            for a in angles
        ]
        return scan
    
    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)  
    
    
    def destroy(self):
        self.destroy_node()


class GPSData(Header):
    
    def __init__(self, pos, rot) -> None:
        super().__init__()
        self.pos = pos
        self.rot = rot

class GPSPublisher(Node):
    """
    Represents the GPS.
    """

    def __init__(self, simulation) -> None:
        super().__init__("gps_publisher")
        self.publisher_ = self.create_publisher(Point, "gps", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish)
        self.i = 0
        self.simulation = simulation

    def publish(self) -> None:
        """
        Publishes the GPS data to the map.
        """
        pos = self.simulation.get_pos()
        rot = self.simulation.get_rot()
        # Generate the GPS data
        msg = Point()
        msg.x = float(pos[0])
        msg.y = float(pos[1])
        msg.z = float(rot)
        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: GPS %d" % self.i)
        self.i += 1
    
    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)  
    
    
    def destroy(self):
        self.destroy_node()


class MoveType(Enum):
    FORWARD = 0
    BACKWARD = 1
    TURN = 2
    PASS = 3

class MoveMsg(Header):
    
    def __init__(self, type: MoveType, arg) -> None:
        super().__init__()
        self.type = type
        self.arg = arg

class MoveSubscriber(Node):
    """
    Receives movement commands from the robot.
    """
    
    def __init__(self, simulation) -> None:
        super().__init__("move_subscriber")
        self.subscription = self.create_subscription(Point, "move", self.callback, 10)
        self.i = 0
        self.simulation = simulation

    def callback(self, msg: Point) -> None:
        """
        Receives the movement commands from the robot.
        """
        if int(msg.x) == 0:
            self.simulation.forward(msg.y)
        elif int(msg.x) == 1:
            self.simulation.forward(-msg.y)
        elif int(msg.x) == 2:
            self.simulation.turn(msg.y)
        elif int(msg.x) == 3:
            pass
        self.get_logger().info(f"Heard: Move {msg.x} {msg.y}")
        self.i += 1
    
    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)  
    
    
    def destroy(self):
        self.destroy_node()

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
        print(f"DORA is at {self.pos} with angle of {self.rot}")
        return True

    def turn(self, angle: float, deg: bool = False):
        """
        Turn the robot by a given angle.

        Params:
            angle (float): The angle to turn the robot by in radians.
            deg (bool, optional): Whether the angle is given in degrees. Defaults to False.
        """
        if deg:
            angle = np.deg2rad(angle)
        self.rot = (self.rot + angle) % (2 * np.pi)

    def norm_spin(self):
        """
        Spins the robot.
        """
        while(True):
            rclpy.spin_once(self.lidar_publisher)
            rclpy.spin_once(self.gps_publisher)
            rclpy.spin_once(self.move_subscriber)
        rclpy.shutdown()
    
    def spin(self) -> None:
        """
        Spins the robot.
        """
        for i in range(10000):
            for n in [self.lidar_publisher, self.move_subscriber, self.gps_publisher]:
                asyncio.get_event_loop().run_until_complete(self.run(n))    
        asyncio.get_event_loop().close()
        rclpy.shutdown()
        
    async def run(self, node) -> None:
        """
        """    
        spin_task = asyncio.get_event_loop().create_task(spin_node(node))
        sleep_task = asyncio.get_event_loop().create_task(asyncio.sleep(1.0))

        # concurrently execute both tasks
        await asyncio.wait([spin_task, sleep_task], return_when=asyncio.FIRST_COMPLETED)

        # cancel tasks
        if spin_task.cancel():
            await spin_task
        if sleep_task.cancel():
            await sleep_task
    
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
    sim.norm_spin()
    
    sim.destroy()
    