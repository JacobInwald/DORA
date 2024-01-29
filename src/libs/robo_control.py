import numpy as np


class Position:
    """
    Represents a position in a 2D coordinate system.
    """

    def __init__(self, x: float, y: float) -> None:
        self.x: float = x
        self.y: float = y
    
    def euclid_dist(self, other: 'Position') -> float:
        """
        Calculate the Euclidean distance between two positions.

        Parameters:
            other (Position): The other position to calculate the distance to.

        Returns:
            float: The Euclidean distance between the two positions.
        """
        return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y))    
        
    # ! SETTERS AND GETTERS
        
    def set(self, x: float, y: float) -> None:
        self.x = x
        self.y = y
        
    def get(self) -> (float, float):
        return (self.x, self.y)    
        
    # ! INBUILT METHODS
        
    def __str__(self) -> str:
        return f"({self.x}, {self.y})"
    
    def __add__(self, other: object) -> 'Position':
        if other.isinstance(Position):
            return Position(self.x + other.x, self.y + other.y)
        elif other.isinstance(float):
            return Position(self.x + other, self.y + other)
        raise Exception("Cannot add Position to non-numerical type.")
    
    def __sizeof__(self) -> int:
        return np.sqrt(self.x ** 2 + self.y ** 2)
    
    def __eq__(self, __value: object) -> bool:
        if __value.isinstance(Position):
            return self.x == __value.x and self.y == __value.y    
        else:
            return False


class Rotation:
    """
    Represents a rotation angle in degrees.
    """

    def __init__(self, theta: float) -> None:
        self.theta: float = theta
                
    # ! SETTERS AND GETTERS
            
    def set(self, theta: float) -> None:
        self.theta = theta
            
    def get(self) -> float:
        return self.theta
        
    # ! INBUILT METHODS
        
    def __str__(self) -> str:
        return f"{self.theta} degrees"
        
    def __add__(self, other) -> 'Rotation':
        return Rotation(self.theta + other.theta)
        
    def __sizeof__(self) -> float:
        return self.theta
        
    def __eq__(self, __value: object) -> bool:
        if __value.isinstance(Rotation):
            return self.theta == __value.theta
        else:
            return False


class DORA:
 
    def __init__(self, pos: 'Position', rot: 'Rotation') -> None:
        self.pos: 'Position' = pos
        self.rot: 'Rotation' = rot
        
    # ! METHODS
    # TODO: Implement these methods.
    
    # Movement
    
    def forward(self, dist: float) -> None:
        self.pos.x += dist * np.cos(self.rot.theta)
        self.pos.y += dist * np.sin(self.rot.theta)
    
    
    def backward(self, dist: float) -> None:
        self.pos.x -= dist * np.cos(self.rot.theta)
        self.pos.y -= dist * np.sin(self.rot.theta)
    
    
    def turn(self, angle: float) -> None:
        self.rot.theta += angle
    
    # Sensors
    
    def getLiDAR(self) -> float:
        pass
     
    # ...
    
    
    # ! GETTERS AND SETTERS
    
    def get_pos(self) -> 'Position':
        return self.pos
    
    def get_rot(self) -> 'Rotation':
        return self.rot
    
    # ! INBUILT METHODS
    
    def __str__(self) -> str:
        return f"DORA at {self.pos}, with angle of {self.rot}"
     