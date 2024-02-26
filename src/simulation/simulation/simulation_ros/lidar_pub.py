import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


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

