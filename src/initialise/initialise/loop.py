import rclpy
from rclpy.node import Node
from dora_srvs.srv import JobCmd, LoopCmd
from control.job import DoraJob
from time import sleep

class Loop(Node):
    """
    The control loop.
    """

    def __init__(self):
        super().__init__('loop')
        
        self.cli_node_ = Node('loop_client')
        self.job_cli_ = self.cli_node_.create_client(JobCmd, '/job')
        while not self.job_cli_.service_is_ready():
            self.get_logger().info('Waiting for controller to be ready ...')
            sleep(1)
            
        # Variables
        self.job = 5  # DEMO
        self.job_cmd = JobCmd.Request()
        # Run loop
        self.loop_cli = self.create_service(
            LoopCmd, '/loop', self.run)

    def run(self, request, response):
        self.get_logger().info('Heard loop request, starting control loop ...')
        while self.job != DoraJob.SCAN:
            job_cmd = JobCmd.Request()
            job_cmd.job = self.job
            self.get_logger().info(f'Sent job request: id = {self.job}')
            future = self.job_cli_.call_async(job_cmd)
            rclpy.spin_until_future_complete(self.cli_node_, future)
            if future.result():
                self.job = (self.job + 1) % len(DoraJob)
        response.status = False
        return response
# Entry Point


def main():
    rclpy.init()
    loop = Loop()
    rclpy.spin(loop)
    loop.destroy_node()
    rclpy.shutdown()
