class Camera:
    """
    Represents the front-facing camera.

    TODO: Implement the Camera class (Ardith)
    This is going to subscribe to the camera in ROS to get the image to process.
    There should be a call back function that automatically pre-processes the image.

    The control flow will be as follows:
     - The camera sensor will subscribe to the camera
     - The overhead camera will publish the image
     - The camera sensor will pre-process the image
     - The camera sensor will update its current frame attribute
     - Continue...

    Attributes:
    - currentFrame: numpy.ndarray - The current frame of the camera.
    """

    def __init__(self):
        self.currentFrame = None
        pass

    def preprocess(self):
        """
        Updates the position of the robot based on the GPS sensor.
        """
        pass
