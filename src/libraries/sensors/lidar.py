import roslibpy as rospy
from std_msgs.msg import LaserScan


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
     - Continue...

    Attributes:
    - currentScan: numpy.ndarray - The current scan of the camera.
    """

    def __init__(self):
        self.currentScan = None

    def preprocess(self):
        """
        Updates the position of the robot based on the GPS sensor, normalise the rotation of the scan
        """

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("lidar", LaserScan, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
