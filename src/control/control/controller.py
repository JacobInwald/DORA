import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from dora_msgs.msg import Toy, Pose, Toys, Map
from dora_srvs.srv import JobCmd, LdsCmd, SweeperCmd, WheelsCmd
from .router import Router
from .occupancy_map import OccupancyMap
from .job import DoraJob
from actuators.wheels import WheelsMove


class Controller(Node):
    """
    Controller for the robot movements (driving and SWEEPER).

    Subscribes to sensor messages and map messages.
    Publishes movement messages.
    """

    def __init__(self):
        super().__init__('controller')

        toy_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.running = False
        self.service_ = self.create_service(JobCmd, '/job', self.switch)
        self.toy_sub_ = self.create_subscription(
            Toys, '/toys', self.toy_callback, toy_qos)
        self.pose_sub_ = self.create_subscription(
            Pose, '/pose', self.pose_callback, 10)
        self.cli_node_ = Node('control_client')
        self.lds_cli_ = self.cli_node_.create_client(LdsCmd, '/lds_service')
        self.wheels_cli_ = self.cli_node_.create_client(WheelsCmd, '/wheels')
        self.sweeper_cli_ = self.cli_node_.create_client(
            SweeperCmd, '/sweeper')
        self.router = Router()
        self.toy = None
        self.pose = None
        self.map = OccupancyMap.load('reference_map.npz')
        self.close_thres = 0.02

    def switch(self, msg, response):
        self.get_logger().info(f'Received job request: {msg.job}')
        response.status = False
        if msg.job == 0:  # SCAN
            response.status = self.scan_request()
        elif msg.job == 1:  # NAV_TOY
            response.status = self.navigate_to_toy()
        elif msg.job == 2:  # RETRIEVE
            response.status = self.retrieve_request()
        elif msg.job == 3:  # NAV_STORE
            response.status = self.navigate_to_storage()
        elif msg.job == 4:  # UNLOAD
            response.status = self.unload_request()
        elif msg.job == 5:  # DEMO
            response.status = self.demo()
        return response

    def toy_callback(self, msg: Toys):
        """
        Update list of toy positions

        Args:
            msg: list of Toy message
        """

    def pose_callback(self, msg: Pose):
        """
        Update self pose

        Args:
            msg: Pose message from GPS
        """
        self.pose = (msg.x, msg.y, msg.rot)
        self.get_logger().info(f'Heard pose: {self.pose}')

        self.demo()

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

    def demo(self):
        """
        Run demo job
        """
        self.get_logger().info('Running demo task')
        if self.running:
            return
        self.running = True

        while self.pose is None:
            pass
        cur_pos = np.array(self.pose[0:2])
        next_retrieve_pt = np.array([2.3, 2])
        route = self.router.route(cur_pos, next_retrieve_pt, self.map)
        status = self.navigate(route)
        return status

    def navigate_to_toy(self) -> bool:
        """
        Calculate route to toy using router.
        Convert route to robot movements, send service call to actuators.

        Returns:
            job status
        """
        cur_pos = np.array([self.pose.x, self.pose.y])
        next_retrieve_pt, self.toy = self.router.next_retrieve_pt(
            self.map, self.toy_sub_, cur_pos)
        route = self.router.route(cur_pos, next_retrieve_pt, self.map)
        status = self.navigate(route)
        return status

    def navigate_to_storage(self) -> bool:
        cur_pos = np.array([self.pose.x, self.pose.y])
        next_unload_pt = self.router.next_unload_pt(
            self.map, self.toy, cur_pos)
        route = self.router.route(cur_pos, next_unload_pt, self.map)
        status = self.navigate(route)
        return status

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
                x_distance = aim_point[0] - self.pose[0]
                y_distance = aim_point[1] - self.pose[1]
                angle = math.atan2(y_distance, x_distance)

                rotation = angle - self.pose[2]
                wheels_rot_cmd = WheelsCmd.Request()
                wheels_rot_cmd.type = 1  # TURN
                wheels_rot_cmd.magnitude = rotation
                future = self.wheels_cli_.call_async(wheels_rot_cmd)
                rclpy.spin_until_future_complete(self.cli_node_, future)

                distance = math.sqrt(x_distance ** 2 + y_distance ** 2)
                wheels_dist_cmd = WheelsCmd.Request()
                wheels_dist_cmd.type = 0  # FORWARD
                wheels_dist_cmd.magnitude = distance
                future = self.wheels_cli_.call_async(wheels_dist_cmd)
                rclpy.spin_until_future_complete(self.cli_node_, future)
        return self.close_to(route[-1], self.pose)

    def close_to(self, src: np.ndarray, dst: Pose):
        """
        :param src: np.array([x,y]) represents coordinate
        :param dst: Pose represent current coordinate
        """
        dx = dst[0] - src[0]
        dy = dst[1] - src[1]
        return math.sqrt(dx ** 2 + dy ** 2) < self.close_thres


# Entry Point
def main():
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
