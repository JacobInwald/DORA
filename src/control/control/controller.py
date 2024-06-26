import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from dora_msgs.msg import Toy, Pose, Toys, Map
from dora_srvs.srv import JobCmd, LdsCmd, SweeperCmd, WheelsCmd
from actuators.wheels import WheelsMove
from actuators.sweeper import SweeperMove
from .router import Router
from .occupancy_map import OccupancyMap
from .job import DoraJob


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
        self.toy_sub_ = self.create_subscription(
            Toys, '/toys', self.toy_callback, toy_qos)
        self.pose_sub_ = self.create_subscription(
            Pose, '/pose', self.pose_callback, 10)
        # Initilaise clients
        self.cli_node_ = Node('control_client')
        self.lds_cli_ = self.cli_node_.create_client(LdsCmd, '/lds_service')
        self.wheels_cli_ = self.cli_node_.create_client(WheelsCmd, '/wheels')
        self.sweeper_cli_ = self.cli_node_.create_client(
            SweeperCmd, '/sweeper')

        # Wait for the services to be ready
        while not self.lds_cli_.service_is_ready() or \
            not self.wheels_cli_.service_is_ready() or \
                not self.sweeper_cli_.service_is_ready():
            self.get_logger().info('Waiting for services to be ready ...')
            sleep(1)

        self.router = Router()
        self.toy = None
        self.pose = None
        self.map = OccupancyMap.load('reference_map.npz')
        self.close_thres = 0.1

        self.drop_off_0 = np.array([0, 0])
        self.drop_off_1 = np.array([0, 0])
        self.drop_off_2 = np.array([0, 0])

        self.service_ = self.create_service(JobCmd, '/job', self.switch)

    def switch(self, msg, response):
        self.get_logger().info(f'Received job request: {msg.job}')
        response.status = False
        if msg.job == DoraJob.SCAN.value:
            response.status = self.scan_request()
        elif msg.job == DoraJob.NAV_TOY.value:
            response.status = self.navigate_to_toy()
        elif msg.job == DoraJob.RETRIEVE.value:
            response.status = self.retrieve_request()
        elif msg.job == DoraJob.NAV_STORAGE.value:
            response.status = self.navigate_to_storage()
        elif msg.job == DoraJob.UNLOAD.value:
            response.status = self.unload_request()
        elif msg.job == DoraJob.DEMO.value:
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

    def localise_request(self):
        self.get_logger().info('Request pose from LDS ...')
        pose_cmd = LdsCmd.Request()
        pose_cmd.localise = True
        future = self.lds_cli_.call_async(pose_cmd)
        rclpy.spin_until_future_complete(self.cli_node_, future)
        self.get_logger().info(
            f'Heard Pose: {(future.result().x, future.result().y, future.result().rot)}')
        if future.result().status:
            self.pose = np.array([future.result().x, future.result().y, future.result().rot])
        else:
            self.pose = None
          
    def turn_request(self, angle: float) -> bool:
        """
        Send service request for turning robot

        Args:
            angle: angle to turn

        Returns:
            job status
        """
        wheels_rot_cmd = WheelsCmd.Request()
        wheels_rot_cmd.type = WheelsMove.TURN.value
        wheels_rot_cmd.magnitude = angle
        future = self.wheels_cli_.call_async(wheels_rot_cmd)
        rclpy.spin_until_future_complete(self.cli_node_, future)

        return future.result().status

    def move_request(self, distance: float) -> bool:
        """
        Send service request for moving robot

        Args:
            distance: distance to move

        Returns:
            job status
        """
        wheels_dist_cmd = WheelsCmd.Request()
        wheels_dist_cmd.type = WheelsMove.FORWARD.value
        wheels_dist_cmd.magnitude = distance
        future = self.wheels_cli_.call_async(wheels_dist_cmd)
        rclpy.spin_until_future_complete(self.cli_node_, future)
        return future.result().status

    def retrieve_request(self) -> bool:
        """
        Send service request for retrieving toy, similar to scan_request

        Returns:
            job status
        """
        sweeper_cmd = SweeperCmd.Request()
        sweeper_cmd.type = SweeperMove.RETRIEVE.value
        future = self.sweeper_cli_.call_async(sweeper_cmd)
        rclpy.spin_until_future_complete(self.cli_node_, future)
        return future.result().status

    def unload_request(self) -> bool:
        """
        Send service request for unloading toy, similar to scan_request

        Returns:
            job status
        """
        sweeper_cmd = SweeperCmd.Request()
        sweeper_cmd.type = SweeperMove.UNLOAD.value
        future = self.sweeper_cli_.call_async(sweeper_cmd)
        rclpy.spin_until_future_complete(self.cli_node_, future)
        return future.result().status

    def stop_sweeper_request(self) -> bool:
        """
        Send service request for retrieving toy, similar to scan_request

        Returns:
            job status
        """
        sweeper_cmd = SweeperCmd.Request()
        sweeper_cmd.type = SweeperMove.STOP.value
        future = self.sweeper_cli_.call_async(sweeper_cmd)
        rclpy.spin_until_future_complete(self.cli_node_, future)
        return future.result().status

    def close_to(self, src: np.ndarray, dst: np.ndarray):
        """
        :param src: np.array([x,y]) represents coordinate
        :param dst: Pose represent current coordinate
        """
        return np.linalg.norm(src[:2] - dst[:2]) < self.close_thres

    def navigate(self, route: np.ndarray) -> bool:
        """
        Navigate through route.
        Translate route to robot (wheels) moves
        Call wheels service for every move.

        Returns:
            job status
        """
        i = 0
        for pt in route:
            self.get_logger().info(f'Move ({i}) - Navigating to: {pt}')
            if not self.navigate_to_pt(pt):
                return False
        return True

    def navigate_to_pt(self, pt: np.ndarray) -> bool:
        """
        Navigate to a point

        Args:
            pt: point to navigate to

        Returns:
            job status
        """
        tries = 8
        for i in range(tries):
            self.pose = None
            while self.pose is None:
                self.localise_request()

            if self.close_to(pt, self.pose):
                return True

            self.get_logger().info(f'Navigate: {self.pose} -> {pt}')

            # Attempt turn
            if not self.rotate(pt):
                break

            dir = pt - self.pose[:2]
            dst = np.linalg.norm(dir)
            self.move_request(dst)
        
        while self.pose is None:
                self.localise_request()
        return self.close_to(pt, self.pose)

    # JOBS
    def calibrate_wheels(self):
        """
        Calibrate wheels
        """
        self.get_logger().info('Running calibration ...')
        pose_ = None
        while pose_ is None:
            self.localise_request()
            pose_ = np.copy(self.pose)
        results = {}
        for i in range(30):
            t = (i+51)*10
            self.turn_request(float(t))

            self.pose = None
            while self.pose is None:
                self.localise_request()
            results[t] = (self.pose[2] - pose_[2]) % (2 * np.pi)
            pose_ = np.copy(self.pose)

        self.get_logger().info(f'Calibration results: {results}')
        from matplotlib import pyplot as plt
        res_y = np.array(list(results.values()))
        res_x = np.array(list(results.keys()))
        np.savez('calibration_turn_1.npz', x=res_x, y=res_y)
        plt.plot(res_x, res_y)
        plt.show()

    def demo(self):
        """
        Run demo job
        """
        self.get_logger().info('Running demo task ...')
        self.pose = None
        while self.pose is None:
            self.localise_request()
        next_retrieve_pt = np.array(self.pose[:2]) + np.array([0, -1])
        next_retrieve_pt = np.array([1.65, -1.375])
        route = self.router.route(self.pose[:2], next_retrieve_pt, self.map)
        self.get_logger().info(f'Following path: {route}')
        if self.navigate(route):
            toy_position = np.array(self.pose[:2]) + np.array([1, -1])
            if self.rotate(toy_position):
                self.get_logger().info(f'Retreiving toy')
                return self.retrieve_request()
        return False

    def navigate_to_toy(self) -> bool:
        """
        Calculate route to toy using router.
        Convert route to robot movements, send service call to actuators.

        Returns:
            job status
        """
        cur_pos = np.array([self.pose[:2]])
        next_retrieve_pt, self.toy = self.router.next_retrieve_pt(
            self.map, self.toy_sub_, cur_pos)
        route = self.router.route(cur_pos, next_retrieve_pt, self.map)
        status = self.navigate(route)
        return status

    def navigate_to_storage(self) -> bool:
        cur_pos = np.array([self.pose[:2]])
        next_unload_pt = self.router.next_unload_pt(
            self.map, self.toy)
        route = self.router.route(cur_pos, next_unload_pt, self.map)
        status = self.navigate(route)
        return status
    
    def rotate(self, target, n=10) -> bool:
        for i in range(n):
            angle = self.calculate_angle(target)
            rotation = angle - self.pose[2]
            self.turn_request(rotation)
            # Update pose
            self.pose = None
            while self.pose is None:
                self.localise_request()
            if abs(self.pose[2] - angle) < 0.05:
                return True
        return False
    
    def calculate_angle(self, target) -> float:
        dir = target - self.pose[:2]
        dst = np.linalg.norm(dir)
        angle = np.arccos(np.dot(dir / dst, np.array([0.0, -1.0])))
        if dir[0] < 0:
            angle *= -1
        return angle


# Entry Point
def main():
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
