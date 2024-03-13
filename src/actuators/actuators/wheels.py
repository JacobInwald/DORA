from enum import Enum
import rclpy
from rclpy.node import Node
from dora_srvs.srv import WheelsCmd
import serial


class WheelsMove(Enum):
    FORWARD = 0
    TURN = 1


ARDUINO_PORT = '/dev/ttyACM0'
ARDUINO_BAUDRATE = 9600


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
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE)
        except serial.SerialException:
            raise Exception("Arduino not found")

    def callback(self, msg: WheelsCmd):
        if msg.type == WheelsMove.FORWARD:
            return self.forward(msg.magnitude)
        elif msg.type == WheelsMove.TURN:
            return self.turn(msg.magnitude)
        return False

    def forward(self, dist: float):
        forward = dist > 0
        time = self.convert_dist_to_time(abs(dist))
        self.arduino.write(
            f"{'forward' if forward else 'backward'}.{time}-".encode())

    def turn(self, angle: float):
        right = angle > 0
        time = self.convert_angle_to_time(abs(angle))
        self.arduino.write(
            f"{'right' if right else 'left'}.{time}-".encode())

    def convert_dist_to_time(self, dist: float) -> int:
        """
        Convert distance to time for the Arduino (in integer milliseconds).
        1 meter = ~1000 milliseconds.
        """
        return int(dist)

    def convert_angle_to_time(self, angle: float) -> int:
        """
        Convert angle to time for the Arduino (in integer milliseconds).
        360 degrees = ~1300 milliseconds.
        """
        return int(angle / 360 * 1300)


def main():
    rclpy.init()
    wheels = Wheels()
    rclpy.spin(wheels)
    wheels.destroy_node()
    rclpy.shutdown()
