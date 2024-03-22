# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from sensor_msgs.msg import LaserScan
# from src.control.control.point_cloud import PointCloud
# from src.control.control.occupancy_map import OccupancyMap
# from src.control.control.utils import *
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
#         self.lds_sub_ = self.create_subscription(
#             LaserScan, '/scan', self.lds_callback, lds_qos)
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
#         self.get_logger().info(
#             f'Last scan: {self.last_scan.header.frame_id} at {self.last_scan.header.stamp.sec}s{self.last_scan.header.stamp.nanosec}')
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
#             self.i += 1
#         self.running = False
#         self.get_logger().info(
#             f'Timer callback: scan received with max_range: {max_range}')

#     def lds_callback(self, msg: 'LaserScan'):
#         """
#         Store last laser scan

#         Args:
#             msg: message received
#         """

#         self.last_scan = msg
#         header = msg.header
#         self.get_logger().info(
#             f'Heard: LDS scan {header.frame_id} at {header.stamp.sec}s{header.stamp.nanosec}')


# def main():
#     rclpy.init()
#     lds_node = LdsNode()
#     rclpy.spin(lds_node)
#     lds_node.destroy_node()
#     rclpy.shutdown()


def stitch_scans():
    scans = []
    for i in range(12):
        index = i+1
        scans.append(np.load(f'data/maps/lds_scan_{index}.npy'))

    pose1 = (3.97, -1.05)
    pose2 = (3.91, -2.39)
    pose3 = (2.68, -2.81)
    pose4 = (1.42, -2.91)
    pose7 = (0.19, -1.93)
    pose8 = (0.25, -0.44)
    pose9 = (1.61, -0.44)
    pose10 = (2.95, -0.45)
    pose11 = (2.6, -1.37)
    pose12 = (1.35, -1.6)

    cloud_1 = PointCloud(scans[0], pose1, 4.5, rot=0)
    cloud_2 = PointCloud(scans[1], pose2, 4.5, rot=0.22004082)
    cloud_3 = PointCloud(scans[2], pose3, 4.5, rot=0)
    cloud_4 = PointCloud(scans[3], pose4, 4.5, rot=0)
    cloud_7 = PointCloud(scans[6], pose7, 4.5, rot=0)
    cloud_8 = PointCloud(scans[7], pose8, 4.5, rot=0.01946999)
    cloud_9 = PointCloud(scans[8], pose9, 4.5, rot=0)
    cloud_10 = PointCloud(scans[9], pose10, 4.5, rot=0)
    cloud_11 = PointCloud(scans[10], pose11, 4.5, rot=0.01972879)
    cloud_12 = PointCloud(scans[11], pose12, 4.5, rot=0)

    # occ = OccupancyMap((0, 0), cloud_1, resolution=0.01)
    # occ.merge_cloud_into_map(cloud_2)
    # occ.merge_cloud_into_map(cloud_3)
    # occ.merge_cloud_into_map(cloud_4)
    # occ.merge_cloud_into_map(cloud_7)
    # occ.merge_cloud_into_map(cloud_8)
    # occ.merge_cloud_into_map(cloud_9)
    # occ.merge_cloud_into_map(cloud_10)
    # occ.merge_cloud_into_map(cloud_11)
    # occ.merge_cloud_into_map(cloud_12)
    # occ.fuzz_map(2)
    # occ.save('new_ref')
    # plt.imshow(occ.map)
    # plt.show()

    occ = OccupancyMap.load('data/maps/new_ref.npz')
    occ.change_res(0.01)

    pose1 = occ.localise_cloud(cloud_1)
    print(f'Pose 1: {pose1}')
    pose2 = occ.localise_cloud(cloud_2)
    print(f'Pose 2: {pose2}')
    pose3 = occ.localise_cloud(cloud_3)
    print(f'Pose 3: {pose3}')
    pose4 = occ.localise_cloud(cloud_4)
    print(f'Pose 4: {pose4}')
    pose7 = occ.localise_cloud(cloud_7)
    print(f'Pose 7: {pose7}')
    pose8 = occ.localise_cloud(cloud_8)
    print(f'Pose 8: {pose8}')
    pose9 = occ.localise_cloud(cloud_9)
    print(f'Pose 9: {pose9}')
    pose10 = occ.localise_cloud(cloud_10)
    print(f'Pose 10: {pose10}')
    pose11 = occ.localise_cloud(cloud_11)
    print(f'Pose 11: {pose11}')
    pose12 = occ.localise_cloud(cloud_12)
    print(f'Pose 12: {pose12}')


# stitch_scans()
ys = [2750, 2100, 1450, 790]
xs = [360, 270, 180, 90]

plt.plot(xs, ys)
plt.show()
