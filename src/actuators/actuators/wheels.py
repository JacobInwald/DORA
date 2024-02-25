from enum import Enum
import rclpy
from rclpy.node import Node
from dorasrvs.srv import WheelsCommand


class WheelsMove(Enum):
    FORWARD = 0
    TURN = 1


class Wheels(Node):
    """
    Represents the wheels.

    Wait for controller wheel movement service call.
    Control wheels to move robot according to service message.
    """
    def __init__(self):
        super.__init__('wheels')
        self.service_ = self.create_service(WheelsCommand, 'wheels', self.callback)

    def callback(self, msg: WheelsCommand):
        if msg.type == WheelsMove.FORWARD:
            return self.forward(msg.magnitude)
        elif msg.type == WheelsMove.TURN:
            return self.turn(msg.magnitude)
        return False

    def forward(self, dist: float):
        pass

    def turn(self, angle: float):
        pass