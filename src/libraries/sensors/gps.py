import numpy as np


class GPS:
    """
    Represents a GPS sensor. (Overhead camera)

    TODO: Implement the GPS sensor class (Wilfredo)
    This is going to subscribe to the Overhead camera in ROS to get the image to process.
    There should be a call back function that automatically updates the position and rotation
    of the robot

    The control flow will be as follows:
     - The GPS sensor will subscribe to the overhead camera
     - The overhead camera will publish the image
     - The GPS sensor will process the image to find the position and rotation of the robot
     - The GPS sensor will update the position and rotation of the robot
     - Continue...

    Attributes:
    - pos: numpy.ndarray - The current position of the robot.
        - 1 is 1 meter in this scale
    - rot: float - The current rotation of the robot. ()
        - 0 is facing the positive x-axis
    """

    def __init__(self):
        self.pos = np.zeros(2)
        self.rot = 0

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

    def getRotInImage(self, img: np.ndarray) -> float:
        """
        Returns the position of the robot in the given image.

        Params:
            img (np.ndarray): The image to search for the robot.

        Returns:
            np.ndarray: The position of the robot in the image.

        TODO: Implement the logic to find the position of the robot in the image. (Wilfredo)
        """
