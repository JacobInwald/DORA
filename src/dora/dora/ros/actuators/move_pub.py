from enum import Enum
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Point

# TODO: Create our own message type for this

class MoveType(Enum):
    FORWARD = 0
    BACKWARD = 1
    TURN = 2
    PASS = 3

class MovePublisher(Node):
    """
    Sends movement commands to the robot.
    """
    
    def __init__(self) -> None:
        super().__init__("move_publisher")
        self.publisher_ = self.create_publisher(Point, "move", 10)
        self.i = 0
        
    def publish(self, move_type=MoveType.PASS, args=0.0) -> None:
        """
        Publishes the GPS data to the map.
        """
        msg = Point()
        msg.x = float(move_type.value)
        msg.y = float(args)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: Move {self.i}: {msg.x} {msg.y}")
        self.i += 1
        self.spin_once()
    
    def spin(self):
        rclpy.spin(self)
    
    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)
    
    def destroy(self):
        self.destroy_node()
