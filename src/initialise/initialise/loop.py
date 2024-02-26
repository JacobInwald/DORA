import rclpy
from rclpy.node import Node
from dora_srvs.srv import JobCmd
from control.control.job import DoraJob


class Loop(Node):
    """
    The control loop.
    """

    def __init__(self):
        self.client_ = self.create_client(JobCmd, 'job')
        self.job = DoraJob.SCAN
        self.job_cmd = JobCmd()

    def run(self):
        while True:
            self.job_cmd.job = self.job
            future = self.lds_cli_.call_async(self.job_cmd)
            rclpy.spin_until_future_complete(self.cli_node_, future)
            if future.result():
                self.job = (self.job + 1) % len(DoraJob)
