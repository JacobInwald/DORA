import numpy as np


def msg_to_np(msg):
    """
    Converts ROS2 Image message to numpy array.

    Args:
        msg: ROS2 sensor_msgs.msg.Image

    Returns:
        np array of the image
    """
    height = msg.height
    width = msg.width
    channel = msg.step // msg.width
    return np.reshape(msg.data, (height, width, channel))
