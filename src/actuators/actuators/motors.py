import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from stepper import Stepper

MOTOR_PINS = {
    "left": {
        "step": 15,
        "dir": 14,
        "enable": 18
    },
    "right": {
        "step": 2,
        "dir": 3,
        "enable": 4
    }
}


class MotorsSubscriber(Node):

    def __init__(self):
        super().__init__('motors_subscriber')
        self.subscription = self.create_subscription(
            String,
            'move',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # initialize the stepper motors
        self.left_motor = Stepper(MOTOR_PINS["left"].values())
        self.right_motor = Stepper(MOTOR_PINS["right"].values())

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        if msg.data not in ["forward", "backward", "left", "right"]:
            self.get_logger().info('Invalid command')
            return

        right_motor_dir = "right" if msg.data == "forward" or msg.data == "left" else "left"
        left_motor_dir = "right" if msg.data == "forward" or msg.data == "right" else "left"

        STEP_AMOUNT = 1000
        SPEED = 1

        # move the motors
        self.right_motor.step(STEP_AMOUNT, right_motor_dir, SPEED, False)
        self.left_motor.step(STEP_AMOUNT, left_motor_dir, SPEED, False)


def main(args=None):
    print("Starting motors node")

    rclpy.init(args=args)

    motors_subscriber = MotorsSubscriber()

    rclpy.spin(motors_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motors_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
