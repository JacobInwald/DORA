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
        self.arduino.write(
            f"{'foward' if dist > 0 else 'backward'}.{abs(dist)}-".encode())

    def turn(self, angle: float):
        self.arduino.write(
            f"{'right' if angle > 0 else 'left'}.{abs(angle)}-".encode())


def main():
    rclpy.init()
    wheels = Wheels()
    rclpy.spin(wheels)
    wheels.destroy_node()
    rclpy.shutdown()
