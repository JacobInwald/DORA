import rclpy
from rclpy.node import Node
from dora_srvs.srv import LoopCmd


class Dora(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('dora')
        self.client_ = self.create_client(LoopCmd, '/loop')
        while not self.client_.service_is_ready():
            pass
        loop_cmd = LoopCmd.Request()
        loop_cmd.start = True
        future = self.client_.call_async(loop_cmd)
        rclpy.spin_until_future_complete(self, future)
        self.destroy_node()
