import math
import numpy as np
import rclpy
from rclpy.node import Node
from dora_msgs.msg import Toy, Pose
from dora_srvs.srv import JobCmd, LdsCmd, SweeperCmd, WheelsCmd
from router import Router
from occupancy_map import OccupancyMap
from job import DoraJob
from move_type import MoveType


class Controller(Node):
    """
    Controller for the robot movements (driving and SWEEPER).

    Subscribes to sensor messages.
    Publishes movement messages.
    """

    def __init__(self):
        super().__init__('controller')
        self.service_ = self.create_service(JobCmd, 'job', self.switch)
        self.toy_sub_ = self.create_subscription([Toy], 'toys', self.toy_callback, 10)
        self.gps_sub_ = self.create_subscription(Pose, 'gps', self.gps_callback, 10)
        self.cli_node_ = Node()
        self.move_cli_ = self.cli_node_.create_client(tuple, 'move distance and rotation')
        self.lds_cli_ = self.cli_node_.create_client(LdsCmd, 'lds_service')
        self.wheels_cli_ = self.cli_node_.create_client(WheelsCmd, 'wheels')
        self.sweeper_cli_ = self.cli_node_.create_client(SweeperCmd, 'sweeper')
        self.router = Router()
        self.map = OccupancyMap()
        self.toy = None

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
        lds_cmd = LdsCmd()
        lds_cmd.scan = True
        future = self.lds_cli_.call_async(lds_cmd)
        rclpy.spin_until_future_complete(self.cli_node_, future)
        return future.result()

    def retrieve_request(self) -> bool:
        """
        Send service request for retrieving toy, similar to scan_request

        Returns:
            job status
        """
        pass

    def unload_request(self) -> bool:
        """
        Send service request for unloading toy, similar to scan_request

        Returns:
            job status
        """
        pass

    def navigate_to_toy(self) -> DoraJob:
        """
        Calculate route to toy using router.
        Convert route to robot movements, send service call to actuators.

        Returns:
            job status
        """
        (route, self.toy) = self.router.next_retrieve_pt(self.router, self.map, self.toy_sub_)
        success = self.navigate(self, route)
        if success:
            return DoraJob.NAV_TOY
        else:
            return None

    def navigate_to_storage(self) -> DoraJob:
        route = self.unload_pt(self.router, self.map, self.toy)
        success = self.navigate(self, route)
        if success:
            return DoraJob.NAV_STORAGE
        else:
            return None

    def navigate(self, route: np.ndarray) -> bool:
        """
        Publish format is a tuple contains MoveType and value (distance to move / angle to rotate)

        Assume the gps will update the current position update automatically at a constant rate If not,
        need to add timer in gps_node to call back often
        """
        for aim_point in route:
            self.go_to_next_point(self, aim_point)
            if self.gps_sub_.x == aim_point.x & self.gps_sub_.y == aim_point.y:
                # Make sure robot reach the destination every time
                continue
            else:
                self.go_to_next_point(aim_point)
        return True

    def go_to_next_point(self, aim_point):
        x_distance = aim_point.x - self.gps_sub_.x
        y_distance = aim_point.y - self.gps_sub_.y
        end_rotation = math.atan2(y_distance, x_distance)
        rotate = end_rotation - self.gps_sub_.rot
        action = (MoveType.TURN, rotate)

        # TODO: Add client part to build connection to send and get response from the robot. The method will end after get response
        # Wait for response from robot of completing action as well

        self.get_logger().info(f'Move command：turn for {rotate} angle')

        distance = math.sqrt(x_distance ^ 2 + y_distance ^ 2)
        action = (MoveType.MOVE, distance)

        self.get_logger().info(f'Move command：move for {distance} meters')

    # Wait for some time for gps update?
