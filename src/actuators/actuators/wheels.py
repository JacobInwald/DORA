from enum import Enum
import rclpy
from rclpy.node import Node
from dora_srvs.srv import WheelsCmd
from stepper import Stepper


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
        super.__init__('wheels')
        self.service_ = self.create_service(WheelsCmd, 'wheels', self.callback)

        # initialize the stepper motors
        self.left_motor = Stepper(MOTOR_PINS["left"].values())
        self.right_motor = Stepper(MOTOR_PINS["right"].values())

    def callback(self, msg: WheelsCmd):
        if msg.type == WheelsMove.FORWARD:
            return self.forward(msg.magnitude)
        elif msg.type == WheelsMove.TURN:
            return self.turn(msg.magnitude)
        return False

    def forward(self, dist: float):
        self.left_motor.step(dist * STEPS_TO_DISTANCE, "right", SPEED)
        self.right_motor.step(dist * STEPS_TO_DISTANCE, "left", SPEED)

    def turn(self, angle: float):
        if angle < 0:
            self.left_motor.step(abs(angle) * STEPS_PER_DEGREE, "left", SPEED)
            self.right_motor.step(abs(angle) * STEPS_PER_DEGREE, "left", SPEED)
        else:
            self.left_motor.step(angle * STEPS_PER_DEGREE, "right", SPEED)
            self.right_motor.step(
                angle * STEPS_PER_DEGREE, "right", SPEED)
