import numpy as np


class DORA:
 
    def __init__(self, pos: np.ndarray, rot: float) -> None:
        self.pos: np.ndarray = np.zeros(2)
        self.rot: float = rot
        
    # ! METHODS
    # TODO: Implement these methods.
    
    # Movement
    
    def forward(self, dist: float) -> None:
        self.pos += dist * np.array(np.cos(self.rot), np.sin(self.rot))
    
    
    def backward(self, dist: float) -> None:
        self.pos -= dist * np.array(np.cos(self.rot), np.sin(self.rot))
    
    
    def turn(self, angle: float) -> None:
        self.rot += angle
    
    # Sensors
    
    def getLiDAR(self) -> (float, float):
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
     