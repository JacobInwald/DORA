import rclpy
from rclpy.node import Node
from dora_msgs.msg import Toy, Pose
from dora_srvs.srv import SweeperCommand


class Controller(Node):
    """
    Controller for the robot movements (driving and SWEEPER).

    Subscribes to sensor messages.
    Publishes movement messages.
    """

    def __init__(self):
        super().__init__('controller')
        self.toy_sub_ = self.create_subscription([Toy], 'toys', self.toy_callback, 10)
        self.gps_sub_ = self.create_subscription(Pose, 'gps', self.gps_callback, 10)
        self.sweeper_cli_ = self.create_client(SweeperCommand, 'sweeper')

    def toy_callback(self, msg):
        """
        Get relative location of toys and calculate map of toys?

        Args:
            msg: list of Toy message

        Returns:
            map: map of toys
        """

    def gps_callback(self, msg):
        """
        Update self pose

        Args:
            msg: Pose message from GPS
        """