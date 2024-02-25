import rclpy
from rclpy.node import Node
from dorasrvs.srv import WheelsCommand


class Wheels(Node):
    """
    Represents the wheels.

    Wait for controller wheel movement service call.
    Control wheels to move robot according to service message.
    """
    def __init__(self):
        super.__init__('wheels')
        self.service_ = self.create_service(WheelsCommand, 'wheels', self.callback)

    def callback(self, msg):
        if msg.type ==