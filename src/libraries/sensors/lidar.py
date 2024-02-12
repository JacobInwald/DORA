import roslibpy as rospy
from sensor_msgs.msg import LaserScan


class LiDAR:
    """
    Represents the LiDAR.

    TODO: Implement the LiDAR class (Jacob?)
    This is going to subscribe to the lidar in ROS to get the lidar pointcloud.
    There should be a call back function that automatically pre-processes the lidar.

    The control flow will be as follows:
     - The lidar sensor will subscribe to the lidar
     - The lidar will publish the image
     - The lidar sensor will pre-process the image
     - The lidar sensor will update its current frame attribute

    Attributes:
    - cur_scan: numpy.ndarray - The current scan of the camera.
    """

    def __init__(self):
        self.min_range = 0.0
        self.cur_scan = None

    def preprocess(self, scan):
        """
        Prepares the scan for use in the network.
        """
        return scan

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("lidar", LaserScan, self.callback)
        rospy.spin()

    def callback(self, msg):
        # This assumes a LaserScan Class
        res = msg.angle_increment
        start = msg.angle_min
        end = msg.angle_max
        scan = []

        for a in range(start, end, res):
            scan.append([a, msg.ranges[a]])

        self.cur_scan = self.preprocess(scan)

        rospy.loginfo(rospy.get_caller_id() + "I heard %s", scan)
