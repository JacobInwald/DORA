from enum import Enum
from rclpy.node import Node
import rclpy
from dora_msgs.msg import Move


class MoveType(Enum):
    FORWARD = 0
    BACKWARD = 1
    TURN = 2
    PASS = 3

class MoveSubscriber(Node):
    """
    Sends movement commands to the robot.
    """
    
    def __init__(self) -> None:
        super().__init__("move_publisher")
        self.publisher_ = self.create_publisher(Move, "move", 10)
        self.i = 0
        
    def publish(self, move_type=MoveType.PASS, args=0.0) -> None:
        """
        Publishes the GPS data to the map.
        """
        msg = Move()
        msg.type = int(move_type.value)
        msg.arg_1 = float(args)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: Move {self.i}: {msg.type} {msg.arg_1}")
        self.i += 1
        self.spin_once()
    
    def spin(self):
        rclpy.spin(self)
    
    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)
    
    def destroy(self):
        self.destroy_node()
