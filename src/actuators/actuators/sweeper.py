import rclpy
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
        self.service_ = self.create_service(SweeperCommand, 'sweeper', self.callback)

    def callback(self, msg):
        if msg.move == 0:
            status = self.retrieve()
        elif msg.move == 1:
            status = self.unload()
        else:
            status = False
        return status

    def retrieve(self):
        """
        Control the SWEEPER arm to retrieve toy

        Returns:
            status: whether the move was executed
        """
        return True

    def unload(self):
        """
        Control the SWEEPER arm to unload toy

        Returns:
            status: whether the move was executed
        """
        return False
