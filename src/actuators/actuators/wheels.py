import serial
import numpy as np
import rclpy
from rclpy.node import Node
from dora_srvs.srv import WheelsCmd
from enum import Enum
from time import sleep


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

    def callback(self, msg, resp):
        type_ = msg.type
        magnitude = msg.magnitude
        self.get_logger().info(
            f'Received cmd {WheelsMove(type_).name} with magnitude {magnitude}')

        if type_ == WheelsMove.FORWARD.value:
            self.forward(magnitude)
        elif type_ == WheelsMove.TURN.value:
            self.turn(magnitude)
        self.get_logger().info('End move')
        resp.status = True
        return resp

    def forward(self, dist: float):
        forward = dist > 0
        time = self.convert_dist_to_time(abs(dist))
        self.get_logger().info(
            f'Start move, forward: {forward}, time: {time}, dist: {dist}')
        self.arduino.write(
            f"{'forward' if forward else 'backward'}.{time}-".encode())
        sleep((time/1000) + 0.5)

    def turn(self, angle: float):
        right = angle > 0
        time = self.convert_angle_to_time(abs(angle))
        self.get_logger().info(
            f'Start move, right: {right}, time: {time}, angle: {angle}')
        self.arduino.write(
            f"{'right' if right else 'left'}.{time}-".encode())
        sleep((time/1000) + 0.5)
        # self.arduino.write(
        #     f"right.{int(angle)}-".encode())
        # sleep(angle/1000 + 0.5)

    def convert_dist_to_time(self, dist: float) -> int:
        """
        Convert distance to time for the Arduino (in integer milliseconds).
        1 meter = ~1000 milliseconds.
        """
        t = dist * 2100
        return int(t)

    def convert_angle_to_time(self, angle: float) -> int:
        """
        Convert angle to time for the Arduino (in integer milliseconds).
        360 degrees = ~1300 milliseconds.
        """
        l = np.array([37.19598879, -158.43157906, 795.7760816, 42.09105822])
        if angle < np.deg2rad(60):
            t = np.polyval(l, angle)
        else:
            t = (angle / (2*np.pi)) * 2750
        return int(t)


def main():
    rclpy.init()
    wheels = Wheels()
    rclpy.spin(wheels)
    wheels.destroy_node()
    rclpy.shutdown()
