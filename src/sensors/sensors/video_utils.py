import numpy as np
import cv2
import queue
import threading
from sensor_msgs.msg import Image


class VideoCapture:
    """
    Buffer-less video capture.
    """

    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.q = queue.Queue()
        self.ret = True
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                self.ret = False
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()  # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        return self.ret, self.q.get()

    def set(self, *args):
        self.cap.set(*args)

    def isOpened(self):
        return self.cap.isOpened()


def cv2_to_msg(frame, header):
    msg = Image()
    msg.header = header
    msg.height = np.shape(frame)[0]
    msg.width = np.shape(frame)[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = np.shape(frame)[2] * np.shape(frame)[1]
    msg.data = np.array(frame).tobytes()
    return msg
