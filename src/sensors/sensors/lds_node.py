import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from dora_msgs.msg import Map, Pose
from dora_srvs.srv import LdsCmd
from control.occupancy_map import OccupancyMap
from control.point_cloud import PointCloud


class LdsNode(Node):
    """
    Represents the LDS (Laser Distance Sensor).

    TODO: Implement LDS node (Jacob)
    Subscribes to the Turtlebot3 LDS-01 to get pointcloud.
    Publishes occupancy map of environment.

    The control flow will be as follows:
     - The LDS node will subscribe to the LDS
     - The listen to the 'lds_scan' client call
     - Calculates the occupancy map from the last laser scan when triggered by hardware
     - Optional: shift pointcloud by time difference using odometry data from hardware
     - Publishes the occupancy map and return True to client

    Attributes:
    - last_scan: numpy.ndarray - The most recent laser scan
    """

    def __init__(self, reference_map: OccupancyMap = None):
        super().__init__('lds_node')
        lds_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.lds_sub_ = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, lds_qos)
        self.gps_pub_ = self.create_publisher(Pose, '/pose', 10)
        self.gps_timer = self.create_timer(1, self.pose_publish)
        self.scan_srv_ = self.create_service(
            LdsCmd, '/lds_service', self.scan_callback)

        # Variables
        self.max_range = 4.5
        self.last_scan = None
        self.last_cloud = None
        if reference_map is None:
            self.reference_map = OccupancyMap.load('reference_map.npz')
        else:
            self.reference_map = reference_map
        self.pose = None
        self.processing_pose = False

    def pose_publish(self):
        if self.last_cloud is None:
            return False
        # Localise Cloud and Publish Pose
        self.processing_pose = True
        pose, acc = self.reference_map.localise_cloud(self.last_cloud)
        if acc > 0.75:
            self.get_logger().info(f'DORA pose: {pose}, Certainty: {100*acc}%')
            self.pose = pose
            self.gps_pub_.publish(pose.to_msg())
            self.processing_pose = False
            return True
        else:
            self.get_logger().error(
                f'Localisation accuracy of {100*acc}% too low, not publishing pose.')
            self.processing_pose = False
            return False

    def lds_callback(self, msg: 'LaserScan'):
        """
        Store last laser scan

        Args:
            msg: message received
        """

        # Read Scan
        res = msg.angle_increment
        start = msg.angle_min
        scan = []
        a = start
        for i in range(len(msg.ranges)):
            scan.append([a, msg.ranges[i]])
            a += res

        # Create PointCloud
        self.last_scan = scan
        self.last_cloud = PointCloud(scan, self.pos, self.max_range)
        self.get_logger().info(f'Heard scan')
        return True

# Entry Point


def main():
    rclpy.init()
    lds_node = LdsNode()
    rclpy.spin(lds_node)
    lds_node.destroy_node()
    rclpy.shutdown()
