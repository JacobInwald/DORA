from sensors.camera import Camera
from sensors.lidar import LiDAR
from sensors.gps import GPS
from actuators.move_pub import MovePublisher, MoveType
import rclpy


class ROSInterface:
    
    def __init__(self) -> None:
        rclpy.init()
        self.gps = GPS()
        self.lidar = LiDAR()
        self.camera = Camera()
        self.move_publisher = MovePublisher()
        self.move_dist = 0.05

    # ! METHODS
    # TODO: Implement these methods. They are the basic movement and sensor methods.
    # TODO: Use the simulator class as a base for how the output (if any) should look like.
    # TODO: Most of these methods can't really be implemented without an idea of what method of hardware control we are using.
    # TODO: For now, just implement whatever you can and research what the best approach is
    # TODO: (Wanting)

    def forward(self, move_dist) -> None:
        """
        Moves the robot in a given direction at a given speed.
        """
        self.move_publisher.publish(MoveType.FORWARD, move_dist)
        
    def backward(self, move_dist) -> None:
        """
        Moves the robot in a given direction at a given speed.
        """
        self.move_publisher.publish(MoveType.BACKWARD, -move_dist)
        
    def turn(self, angle) -> None:
        """
        Turns the robot by a given angle.
        """
        self.move_publisher.publish(MoveType.TURN, angle)

    def update_sensors(self) -> None:
        """
        Spins the robot once.
        """
        rclpy.spin_once(self.lidar)
        rclpy.spin_once(self.gps)
    
    def destroy(self) -> None:
        """
        Destroys the robot.
        """
        self.move_publisher.destroy()
        self.lidar.destroy()
        self.gps.destroy()
        # self.camera.destroy()
        rclpy.shutdown()
    
    def __str__(self) -> str:
        return f"DORA at {self.gps.pos}, with angle of {self.gps.pos}"

    