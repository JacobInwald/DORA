import rclpy
from rclpy.node import Node
from dora_msgs.msg import Toy, Pose
from dora_srvs.srv import SweeperCommand, WheelsCommand


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
        self.wheels_cli_ = self.create_client(WheelsCommand, 'wheels')
        self.sweeper_cli_ = self.create_client(SweeperCommand, 'sweeper')

    def toy_callback(self, msg: [Toy]):
        """
        Update list of toy positions

        Args:
            msg: list of Toy message
        """

    def gps_callback(self, msg: Pose):
        """
        Update self pose

        Args:
            msg: Pose message from GPS
        """