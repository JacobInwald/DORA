import numpy as np


class GPS:
    """
    Represents a GPS sensor. (Overhead camera)

    Attributes:
    - pos: numpy.ndarray - The current position of the robot.
    TODO: Implement the GPS sensor class (Wilfredo)
    """

    def __init__(self):
        pass

    def locate(self, controller: "simulate.Controller"):
        """
        Updates the position of the robot based on the GPS sensor.
        """
        img = controller.getOverheadImage()
        return self.getPosInImage(img), self.getRotInImage(img)

    def getPosInImage(self, img: np.ndarray) -> np.ndarray:
        """
        Returns the position of the robot in the given image.

        Params:
            img (np.ndarray): The image to search for the robot.

        Returns:
            np.ndarray: The position of the robot in the image.

        TODO: Implement the logic to find the position of the robot in the image. (Wilfredo)
        """
        pass

    def getRotInImage(self, img: np.ndarray) -> float:
        """
        Returns the position of the robot in the given image.

        Params:
            img (np.ndarray): The image to search for the robot.

        Returns:
            np.ndarray: The position of the robot in the image.

        TODO: Implement the logic to find the position of the robot in the image. (Wilfredo)
        """
        pass