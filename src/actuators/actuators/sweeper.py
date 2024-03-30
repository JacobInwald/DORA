import rclpy
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

    mc = Motors()
    motor_id_left = 1
    motor_id_right = 0
    # The port that your motor is plugged in to

    speed_inwards = 50
    speed_outwards_left = -35
    speed_outwards_right = -50
    # Should be different speed for left and right motors when rotating outwards so that they will not block each other

    def retrieve(self):
        """
        Control the SWEEPER spinner to retrieve toy by rotating inwards

        Returns:
            status: whether the move was executed
        """
        self.mc.move_motor(self.motor_id_left, self.speed_inwards)
        self.mc.move_motor(self.motor_id_right, self.speed_inwards)
        # Encoder board can be fragile - always use a try/except loop

    def unload(self):
        """
        Control the SWEEPER spinner to unload toy by rotating outwards

        Returns:
            status: whether the move was executed
        """
        self.mc.move_motor(self.motor_id_left, self.speed_outwards_left)
        self.mc.move_motor(self.motor_id_right, self.speed_outwards_right)

    def stop(self):
        self.mc.stop_motors()
        return True

    def __init__(self):
        super().__init__('sweeper')
        self.service_ = self.create_service(SweeperCmd, '/sweeper', self.callback)

    def callback(self, msg, resp):
        type_ = msg.type
        self.get_logger().info(f'Received cmd of type {type_}')

        if type_ == 1:
            self.get_logger().info('Rotate inwards')
            self.retrieve()
        elif type_ == 2:
            self.get_logger().info('Rotate outwards')
            self.unload()
        elif type_ == 0:
            self.get_logger().info('Stop rotating')
            self.stop()
        resp.status = True
        return resp


def main():
    rclpy.init()
    sweeper = Sweeper()
    rclpy.spin(sweeper)
    sweeper.destroy_node()
    rclpy.shutdown()