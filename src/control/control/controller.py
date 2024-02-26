import math
import numpy as np
import rclpy
from rclpy.node import Node
from dora_msgs.msg import Toy, Pose
from dora_srvs.srv import JobCmd, LdsCmd, SweeperCmd, WheelsCmd
from router import Router
from occupancy_map import OccupancyMap
from job import DoraJob
from actuators.wheels import WheelsMove


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
        self.pose = None
        self.close_thres = 3

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
        self.pose = Pose

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

    def navigate_to_toy(self) -> bool:
        """
        Calculate route to toy using router.
        Convert route to robot movements, send service call to actuators.

        Returns:
            job status
        """
        (route, self.toy) = self.router.next_retrieve_pt(self.router, self.map, self.toy_sub_)
        success = self.navigate(self, route)
        return success

    def navigate_to_storage(self) -> bool:
        route = self.unload_pt(self.router, self.map, self.toy)
        success = self.navigate(self, route)
        return success

    def navigate(self, route: np.ndarray) -> bool:
        """
        Navigate through route.
        Translate route to robot (wheels) moves
        Call wheels service for every move.

        Returns:
            job status
        """
        for aim_point in route:
            while not self.close_to(aim_point, self.pose):
                x_distance = aim_point.x - self.pose.x
                y_distance = aim_point.y - self.pose.y
                angle = math.atan2(y_distance, x_distance)

                rotation = angle - self.pose.rot
                wheels_rot_cmd = WheelsCmd.Request()
                wheels_rot_cmd.type = WheelsMove.TURN
                wheels_rot_cmd.magnitude = rotation
                future = self.wheels_cli_.call_async(wheels_rot_cmd)
                rclpy.spin_until_future_complete(self.cli_node_, future)

                distance = math.sqrt(x_distance ^ 2 + y_distance ^ 2)
                wheels_dist_cmd = WheelsCmd.Request()
                wheels_dist_cmd.type = WheelsMove.FORWARD
                wheels_dist_cmd.magnitude = distance
                future = self.wheels_cli_.call_async(wheels_dist_cmd)
                rclpy.spin_until_future_complete(self.cli_node_, future)
        return True

    def close_to(self, src, dst):
        dx = dst.x - src.x
        dy = dst.y - dst.x
        return math.sqrt(dx^2 + dy^2) < self.close_thres

