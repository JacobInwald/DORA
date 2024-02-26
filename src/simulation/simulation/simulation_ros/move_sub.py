import rclpy
from rclpy.node import Node
from dora_msgs.msg import Move


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
        self.subscription = self.create_subscription(Move, "move", self.callback, 10)
        self.i = 0
        self.simulation = simulation

    def callback(self, msg: Move) -> None:
        """
        Receives the movement commands from the robot.
        """
        if int(msg.type) == 0:
            self.simulation.forward(msg.arg_1)
        elif int(msg.type) == 1:
            self.simulation.forward(-msg.arg_1)
        elif int(msg.type) == 2:
            self.simulation.turn(msg.arg_1, True)
        elif int(msg.type) == 3:
            pass
        self.get_logger().info(f"Heard: Move number {self.i}: {msg.type} {msg.arg_1}")
        self.i += 1
    
    def spin(self):
        rclpy.spin(self)
    
    async def spin_once(self, timeout_sec=0.01):
        rclpy.spin_once(self, timeout_sec=timeout_sec)
    
    
    def destroy(self):
        self.destroy_node()
