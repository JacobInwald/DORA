import rclpy
from rclpy.node import Node
from dora_srvs.srv import JobCmd, LoopCmd
from control.job import DoraJob


class Loop(Node):
    """
    The control loop.
    """

    def __init__(self):
        super().__init__('loop')
        self.client_ = self.create_client(JobCmd, 'job')
        self.job = DoraJob.SCAN
        self.job_cmd = JobCmd.Request()
        self.service_ = self.create_service(LoopCmd, 'loop', self.run)

    def run(self):
        while True:
            self.job_cmd.job = self.job
            future = self.lds_cli_.call_async(self.job_cmd)
            rclpy.spin_until_future_complete(self.cli_node_, future)
            if future.result():
                self.job = (self.job + 1) % len(DoraJob)

# Entry Point
def main():
    rclpy.init()
    loop = Loop()
    rclpy.spin(loop)
    loop.destroy_node()
    rclpy.shutdown()