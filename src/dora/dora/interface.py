from dora_software_ros.camera_sub import *
from dora_software_ros.gps_sub import *
from dora_software_ros.lidar_sub import *
from dora_software_ros.move_pub import *
import rclpy


class ROSInterface:
    
    def __init__(self) -> None:
        rclpy.init()
        self.gps = GPSSubscriber()
        self.lidar = LidarSubscriber()
        self.camera = CameraSubscriber()
        self.move_publisher = MovePublisher()
        self.move_dist = 0.05

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

    