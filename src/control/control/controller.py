import rclpy
from rclpy.node import Node
from dora_msgs.msg import Toy, Pose
from dora_srvs.srv import JobCommand, LdsCommand, SweeperCommand, WheelsCommand
from router import Router
from job import DoraJob



class Controller(Node):
    """
    Controller for the robot movements (driving and SWEEPER).

    Subscribes to sensor messages.
    Publishes movement messages.
    """

    def __init__(self):
        super().__init__('controller')
        self.service_ = self.create_service(JobCommand, 'job', self.switch)
        self.toy_sub_ = self.create_subscription([Toy], 'toys', self.toy_callback, 10)
        self.gps_sub_ = self.create_subscription(Pose, 'gps', self.gps_callback, 10)
        self.lds_cli_ = self.create_client(LdsCommand, 'lds_service')
        self.wheels_cli_ = self.create_client(WheelsCommand, 'wheels')
        self.sweeper_cli_ = self.create_client(SweeperCommand, 'sweeper')
        self.cli_node_ = Node()
        self.router = Router()
        self.state = DoraJob.SCAN

    def switch(self, msg):
        if msg.job == DoraJob.SCAN:
            return self.scan_request()
        elif msg.job == DoraJob.NAV_TOY:
            return self.navigate_to_toy()
        elif msg.job == DoraJob.RETRIEVE:
            return self.retrieve_request()
        elif msg.job == DoraJob.NAV_STORAGE:
            return self.navigate_to_storage()
        elif msg.job == DoraJob.UNLOAD:
            return self.unload_request()
        return False

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

    def scan_request(self):
        lds_command = LdsCommand()
        lds_command.scan = True
        future = self.lds_cli_.call_async(lds_command)
        rclpy.spin_until_future_complete(self.cli_node_, future)
        return future.result()

    def retrieve_request(self):
        pass

    def unload_request(self):
        pass

    def navigate_to_toy(self):
        pass

    def navigate_to_storage(self):
        pass
