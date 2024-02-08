import numpy as np
from sensors.camera import Camera
from sensors.lidar import LiDAR
from sensors.gps import GPS


class ROSInterface:
    def __init__(self) -> None:
        self.gps = GPS()
        self.lidar = LiDAR()
        self.camera = Camera()

    # ! METHODS
    # TODO: Implement these methods. They are the basic movement and sensor methods.
    # TODO: Use the simulator class as a base for how the output (if any) should look like.
    # TODO: Most of these methods can't really be implemented without an idea of what method of hardware control we are using.
    # TODO: For now, just implement whatever you can and research what the best approach is
    # TODO: (Wanting)

    def move(self, speed: float, direction: float) -> None:
        """
        Moves the robot in a given direction at a given speed.
        """
        pass

    def __str__(self) -> str:
        return f"DORA at {self.gps.pos}, with angle of {self.gps.pos}"
