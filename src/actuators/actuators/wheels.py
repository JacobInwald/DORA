from enum import Enum
import rclpy
from rclpy.node import Node
from dora_srvs.srv import WheelsCmd


class WheelsMove(Enum):
    FORWARD = 0
    TURN = 1


MOTOR_PINS = {
    "left": {
        "step": 15,
        "dir": 14,
        "enable": 18
    },
    "right": {
        "step": 2,
        "dir": 3,
        "enable": 4
    }
}
STEPS_TO_DISTANCE = 1000
STEPS_PER_DEGREE = 100
SPEED = 1


class Wheels(Node):
    """
    Represents the wheels.

    Wait for controller wheel movement service call.
    Control wheels to move robot according to service message.
    """

    def __init__(self):
        super().__init__('wheels')
        self.service_ = self.create_service(
            WheelsCmd, '/wheels', self.callback)

    def callback(self, msg: WheelsCmd):
        if msg.type == WheelsMove.FORWARD:
            return self.forward(msg.magnitude)
        elif msg.type == WheelsMove.TURN:
            return self.turn(msg.magnitude)
        return False

    def forward(self, dist: float):
        pass

    def turn(self, angle: float):
        pass


def main():
    rclpy.init()
    wheels = Wheels()
    rclpy.spin(wheels)
    wheels.destroy_node()
    rclpy.shutdown()
