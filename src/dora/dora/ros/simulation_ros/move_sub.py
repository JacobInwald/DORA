import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


# class MoveType(Enum):
#     FORWARD = 0
#     BACKWARD = 1
#     TURN = 2
#     PASS = 3

class MoveSubscriber(Node):
    """
    Receives movement commands from the robot.
    """
    
    def __init__(self, simulation) -> None:
        super().__init__("move_subscriber")
        self.subscription = self.create_subscription(Point, "move", self.callback, 10)
        self.i = 0
        self.simulation = simulation

    def callback(self, msg: Point) -> None:
        """
        Receives the movement commands from the robot.
        """
        if int(msg.x) == 0:
            self.simulation.forward(msg.y)
        elif int(msg.x) == 1:
            self.simulation.forward(-msg.y)
        elif int(msg.x) == 2:
            self.simulation.turn(msg.y, True)
        elif int(msg.x) == 3:
            pass
        self.get_logger().info(f"Heard: Move number {self.i}: {msg.x} {msg.y}")
        self.i += 1
    
    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self, timeout_sec=0.01):
        rclpy.spin_once(self, timeout_sec=timeout_sec)
    
    
    def destroy(self):
        self.destroy_node()
