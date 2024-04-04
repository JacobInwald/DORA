import rclpy
import time
from rclpy.node import Node
from dora_srvs.srv import SweeperCmd
from enum import Enum
from .motors import Motors


class SweeperMove(Enum):
    STOP = 0
    RETRIEVE = 1
    UNLOAD = 2


class Sweeper(Node):
    """
    Represents the SWEEPER arm.

    Subscribes to arm movement commands.
    Move the SWEEPER arm according to the command.
    """

    def __init__(self):
        super().__init__('sweeper')
        self.service_ = self.create_service(SweeperCmd, '/sweeper', self.callback)
        self.mc = Motors()
        self.motor_id_left = 1
        self.motor_id_right = 0
        # The port that your motor is plugged in to

    def callback(self, msg, resp):
        type_ = msg.type
        self.get_logger().info(f'Received cmd {SweeperMove(type_).name}')

        if type_ == SweeperMove.RETRIEVE.value:
            self.get_logger().info('Rotate inwards')
            resp.status = self.retrieve()
        elif type_ == SweeperMove.UNLOAD.value:
            self.get_logger().info('Rotate outwards')
            resp.status = self.unload()
        elif type_ == SweeperMove.STOP.value:
            self.get_logger().info('Stop rotating')
            resp.status = self.stop()
        else:
            resp.status = False
        return resp

    def retrieve(self):
        """
        Control the SWEEPER spinner to retrieve toy by rotating inwards

        Returns:
            status: whether the move was executed
        """
        speed_inwards_left = -35
        speed_inwards_right = -50
        self.mc.move_motor(self.motor_id_left, speed_inwards_left)
        self.mc.move_motor(self.motor_id_right, speed_inwards_right)
        time.sleep(5)
        return self.stop()
        return True
        # Encoder board can be fragile - always use a try/except loop

    def unload(self):
        """
        Control the SWEEPER spinner to unload toy by rotating outwards

        Returns:
            status: whether the move was executed
        """
        speed_outwards = 50
        # Speed for left and right motors need to be different for rotating outwards to avoid blocking each other
        self.mc.move_motor(self.motor_id_left, speed_outwards)
        self.mc.move_motor(self.motor_id_right, speed_outwards)
        time.sleep(5)
        return self.stop()

    def stop(self):
        self.mc.stop_motors()
        return True


def main():
    rclpy.init()
    sweeper = Sweeper()
    rclpy.spin(sweeper)
    sweeper.destroy_node()
    rclpy.shutdown()
