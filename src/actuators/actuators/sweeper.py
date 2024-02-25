imort rclpy
from rclpy.node import Node
from dorasrvs.srv import SweeperCommand


class Sweeper(Node):
    """
    Represents the SWEEPER arm.

    Subscribes to arm movement commands.
    Move the SWEEPER arm according to the command.
    """

    def __init__(self):
        super.__init__('sweeper')
        self.service_ = self.create_service(SweeperCommand, 'sweeper_service', self.callback)

    def callback(self, msg):
        return True

    def forward(self, dist: float):
        pass

    def turn(self, angle: float):
        pass
