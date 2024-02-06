import numpy as np


class Controller:
 
    def __init__(self, pos: np.ndarray, rot: float) -> None:
        self.pos: np.ndarray = np.zeros(2)
        self.rot: float = rot
        
    # ! METHODS
    # TODO: Implement these methods. They are the basic movement and sensor methods.
    # TODO: Use the simulator class as a base for how the output (if any) should look like.
    # TODO: Most of these methods can't really be implemented without an idea of what method of hardware control we are using.
    # TODO: For now, just implement whatever you can and research what the best approach is
    # TODO: (Wanting)
    
    # Movement
    
    def forward(self, dist: float) -> None:
        self.pos += dist * np.array(np.cos(self.rot), np.sin(self.rot))
    
    
    def backward(self, dist: float) -> None:
        self.pos -= dist * np.array(np.cos(self.rot), np.sin(self.rot))
    
    
    def turn(self, angle: float) -> None:
        self.rot += angle
    
    # Sensors
    
    def scan(self) -> (float, float):
        """
        Retrieves the LiDAR data in polar coordinates.
        """
        return (10, self.theta)
    
    
    def localise(self) -> None:
        """
        Localises the robot using the LiDAR data.
        """
        pass
    # ...
    
    
    # ! GETTERS AND SETTERS
    
    def get_pos(self) -> np.ndarray:
        return self.pos
    
    def get_rot(self) -> float:
        return self.rot
    
    # ! INBUILT METHODS
    
    def __str__(self) -> str:
        return f"DORA at {self.pos}, with angle of {self.rot}"
     
     
controller = Controller(np.array([0, 0]), 0)
controller.forward(10)
controller.turn(np.pi/2)