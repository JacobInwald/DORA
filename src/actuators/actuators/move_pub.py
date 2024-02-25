from enum import Enum
from rclpy.node import Node
import rclpy
from dora_msgs.msg import Move
from geometry_msg.msg import Twist


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
        self.publisher_ = self.create_publisher(Move, "move", 10)
        self.i = 0

        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

    def publish(self, move_type=MoveType.PASS, args=0.0) -> None:
        """
        Publishes the movement data to the map.
        """
        msg = Move()
        msg.type = int(move_type.value)
        msg.arg_1 = float(args)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: Move {self.i}: {msg.type} {msg.arg_1}")
        self.i += 1
        self.spin_once()



    # Can delete it if no necessary
    def move(self, speed: float, direction: float) -> None:
        """
        Moves the robot in a given direction at a given speed.
        """

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = direction # Angular speed

    self.cmd_vel_pub.publish(twist)  # Tell robot to rotate
    rclpy.sleep(1)  # Wait for rotate, the duration needs to be tested

    max_speed = 1.0  # Should have a max speed to prevent speed too high, the value wait for testing

    twist = Twist()
    twist.linear.x = min(speed, max_speed)
    twist.angular.z = 0.0

    while rclpy.ok():  # Keep moving forward until program shutdown
        self.cmd_vel_pub.publish(twist)
        rclpy.sleep(0.1)


    # Might should have a time or distance to stop moving

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
