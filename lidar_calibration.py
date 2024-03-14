# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from sensor_msgs.msg import LaserScan 
from src.control.control.point_cloud import PointCloud
from src.control.control.occupancy_map import OccupancyMap
import matplotlib.pyplot as plt
import numpy as np
import cv2

# class LdsNode(Node):
#     """
#     Represents the LDS (Laser Distance Sensor).

#     TODO: Implement LDS node (Jacob)
#     Subscribes to the Turtlebot3 LDS-01 to get pointcloud.
#     Publishes occupancy map of environment.

#     The control flow will be as follows:
#      - The LDS node will subscribe to the LDS
#      - The listen to the 'lds_scan' client call
#      - Calculates the occupancy map from the last laser scan when triggered by hardware
#      - Optional: shift pointcloud by time difference using odometry data from hardware
#      - Publishes the occupancy map and return True to client

#     Attributes:
#     - last_scan: numpy.ndarray - The most recent laser scan
#     """

#     def __init__(self):
#         super().__init__('lds_node')
#         lds_qos = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=1
#         )
#         self.lds_sub_ = self.create_subscription(LaserScan, '/scan', self.lds_callback, lds_qos)
#         self.last_scan = None
#         self.running = False
#         self.timer = self.create_timer(2, self.timer_callback)
#         self.i = 1
        
#     def timer_callback(self):
#         if self.running:
#             return
#         self.running = True
#         self.get_logger().info('Timer callback')
#         if self.last_scan is None:
#             self.get_logger().info('Timer callback: no scan received')
#             return
#         self.get_logger().info(f'Last scan: {self.last_scan.header.frame_id} at {self.last_scan.header.stamp.sec}s{self.last_scan.header.stamp.nanosec}')
#         max_range = 4.5
#         ranges = []
#         for i in range(len(self.last_scan.ranges)):
#             if self.last_scan.ranges[i] == float('inf') \
#                 or self.last_scan.ranges[i] <= 0.01 \
#                     or self.last_scan.ranges[i] > max_range:
#                 ranges.append(np.inf)
#             else:
#                 ranges.append(self.last_scan.ranges[i])
#         angles = []
#         res = self.last_scan.angle_increment
#         start = self.last_scan.angle_min
#         a = start
#         for i in range(len(self.last_scan.ranges)):
#             angles.append(a)
#             a += res
#         scan = [i for i in zip(angles, ranges)]
        
#         cloud = PointCloud(scan, (0, 0), max_range, rot=0, res=0.01)
#         cv2.imshow('LDS', cloud.generate())
#         k = cv2.waitKey(1000)
#         if k == ord('s'):
#             np.save(f'data/maps/lds_scan_{self.i}.npy', np.array(scan))
#             self.i+=1
#         self.running=False
#         self.get_logger().info(f'Timer callback: scan received with max_range: {max_range}')
        
#     def lds_callback(self, msg: 'LaserScan'):
#         """
#         Store last laser scan

#         Args:
#             msg: message received
#         """

#         self.last_scan = msg
#         header = msg.header
#         self.get_logger().info(f'Heard: LDS scan {header.frame_id} at {header.stamp.sec}s{header.stamp.nanosec}')


# def main()
#     rclpy.init()
#     lds_node = LdsNode()
#     rclpy.spin(lds_node)
#     lds_node.destroy_node()
#     rclpy.shutdown()

def stitch_scans():
    scans= []
    for i in range(12):
        index = i+1
        scans.append(np.load(f'data/maps/lds_scan_{index}.npy'))
    
    
    cloud_1 = PointCloud(scans[0], (-0.97, -0.35), 4.5, rot=0, res=0.05)
    occ = OccupancyMap((0, 0), cloud_1, resolution=0.01)
    
    cloud_2 = PointCloud(scans[1], (-2.3, -0.35), 4.5, rot= 9*((2*np.pi)/360), res=0.05)

    occ.merge_cloud_into_map(cloud_2)
    
    cloud_3 = PointCloud(scans[2], (-2.79, -1.59), 4.5, rot=0, res=0.01)
    occ.merge_cloud_into_map(cloud_3)
    
    cloud_4 = PointCloud(scans[3], (-2.81, -2.86), 4.5, rot=0, res=0.01)
    occ.merge_cloud_into_map(cloud_4)
    
    cloud_7 = PointCloud(scans[6], (-1.86, -4.04), 4.5, rot=0, res=0.01)
    occ.merge_cloud_into_map(cloud_7)
    
    cloud_8 = PointCloud(scans[7], (-0.36, -3.96), 4.5, rot=0, res=0.01)
    occ.merge_cloud_into_map(cloud_8)
    
    cloud_9 = PointCloud(scans[8], (-0.36, -2.67), 4.5, rot=0, res=0.01)
    occ.merge_cloud_into_map(cloud_9)
    
    cloud_10 = PointCloud(scans[9], (-0.36, -1.39), 4.5, rot=0, res=0.01)
    occ.merge_cloud_into_map(cloud_10)
    
    cloud_11 = PointCloud(scans[10], (-1.47, -1.7), 4.5, rot=0, res=0.01)
    occ.merge_cloud_into_map(cloud_11)
    
    cloud_12 = PointCloud(scans[11], (-1.53, -2.91), 4.5, rot=0, res=0.01)
    occ.merge_cloud_into_map(cloud_12)
    
    plt.imshow(occ.map)
    plt.show()
    
stitch_scans()
