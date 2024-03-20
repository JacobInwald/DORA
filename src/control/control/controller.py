import math
import numpy as np
from time import sleep
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
        self.close_thres = 0.02
        
        self.drop_off_0 = np.array([0, 0])
        self.drop_off_1 = np.array([0, 0])
        self.drop_off_2 = np.array([0, 0])
        
        self.service_ = self.create_service(JobCmd, '/job', self.switch)

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

    def localise_request(self):
        self.get_logger().info('Request pose from LDS ...')
        pose_cmd = LdsCmd.Request()
        pose_cmd.localise = True 
        future = self.lds_cli_.call_async(pose_cmd)
        rclpy.spin_until_future_complete(self.cli_node_, future)
        if future.result().status:
            return np.array([future.result().x, future.result().y, future.result().rot])
        else:
            return None

    def turn_request(self, angle: float) -> bool:
        """
        Send service request for turning robot

        Args:
            angle: angle to turn

        Returns:
            job status
        """
        wheels_rot_cmd = WheelsCmd.Request()
        wheels_rot_cmd.type = 1  # TURN
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
        wheels_dist_cmd.type = 0  # FORWARD
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
        pass

    def unload_request(self) -> bool:
        """
        Send service request for unloading toy, similar to scan_request

        Returns:
            job status
        """
        pass
    
    def close_to(self, src: np.ndarray, dst: np.ndarray):
        """
        :param src: np.array([x,y]) represents coordinate
        :param dst: Pose represent current coordinate
        """
        return np.linalg.norm(src, dst) < self.close_thres
    
    def navigate(self, route: np.ndarray) -> bool:
        """
        Navigate through route.
        Translate route to robot (wheels) moves
        Call wheels service for every move.

        Returns:
            job status
        """
        for pt in route:
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
        tries = 3
        for i in range(tries):
            cur_pose = None
            while cur_pose is None:
                cur_pose = self.localise_request()
            
            if self.close_to(pt, cur_pose):
                return True
            
            self.get_logger().info(f'Navigate: {cur_pose} -> {pt}')
            
            x_dis = pt[0] - cur_pose[0]
            y_dis = pt[1] - cur_pose[1]
            angle = np.arctan2(y_dis, x_dis) + (np.pi / 2)
            rotation = angle - cur_pose[2]
            distance = np.sqrt(x_dis ** 2 + y_dis ** 2)
            
            self.turn_request(rotation)
            self.move_request(distance)
            
        return self.close_to(pt, cur_pose)

    # JOBS

    def demo(self):
        """
        Run demo job
        """
        self.get_logger().info('Running demo task ...')
        pose = None
        while pose is None:
            pose = self.localise_request()
        cur_pos = np.array(pose[0:2])
        
        next_retrieve_pt = cur_pos + np.array([0, -1])
        return self.navigate_to_pt(next_retrieve_pt)

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



# Entry Point
def main():
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
