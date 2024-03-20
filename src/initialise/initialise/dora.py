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
        self.client_.call_async(loop_cmd)
        rclpy.spin_once(self, timeout_sec=0)
        self.destroy_node()
