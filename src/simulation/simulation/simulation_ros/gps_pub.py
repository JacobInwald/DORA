import rclpy
from rclpy.node import Node
from dora_msgs.msg import Pose

class GPSPublisher(Node):
    """
    Represents the GPS.
    """

    def __init__(self, simulation) -> None:
        super().__init__("gps_publisher")
        self.publisher_ = self.create_publisher(Pose, "gps", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish)
        self.i = 0
        self.simulation = simulation

    def publish(self) -> None:
        """
        Publishes the GPS data to the map.
        """
        pos = self.simulation.get_pos()
        rot = self.simulation.get_rot()
        # Generate the GPS data
        msg = Pose()
        msg.x = float(pos[0])
        msg.y = float(pos[1])
        msg.rot = float(rot)
        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: GPS {self.i}: {pos} {rot}")
        self.i += 1
    
    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)  
    
    
    def destroy(self):
        self.destroy_node()

