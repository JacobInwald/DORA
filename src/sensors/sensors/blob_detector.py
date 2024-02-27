import numpy as np
import cv2

class BlobDetector:
    def __init__(self):

        self.x_axis = np.array([1, 0, 0])
        self.y_axis = np.array([0, 1, 0])
        self.z_axis = np.array([0, 0, 1])

        # Last known configuration of the system. Initially empty
        # index 0: ja1, index 1: ja3, index2: ja4
        self.last_known_ja = [0,0,0]

        # number of times we have not allowed the angle to change
        # index 0: ja1, index1: ja3, and index 2: ja4
        self.buffer_graph_smoothing = [0, 0, 0]

        # Colour Ranges to be used for thresholding
        # RED
        self.RED_BLOB_HSV_COLOR_RANGE_BELOW = (0,110,172)
        self.RED_BLOB_HSV_COLOR_RANGE_UPPER = (6,255,255)
        # GREEN
        self.GREEN_BLOB_HSV_COLOR_RANGE_BELOW = (76,70,110)
        self.GREEN_BLOB_HSV_COLOR_RANGE_UPPER = (85,255,255)
        # BLUE
        self.BLUE_BLOB_HSV_COLOR_RANGE_UPPER = (118,255,255)
        self.BLUE_BLOB_HSV_COLOR_RANGE_BELOW = (110,148,160)
        # YELLOW
        self.YELLOW_BLOB_HSV_COLOR_RANGE_BELOW = (20,100,100)
        self.YELLOW_BLOB_HSV_COLOR_RANGE_UPPER = (30,255,255)
        # WHITE
        self.WHITE_BLOB_HSV_COLOR_RANGE_BELOW = (0,0,255)
        self.WHITE_BLOB_HSV_COLOR_RANGE_UPPER = (255,255,255)
      # Returns the position and visibility of a given coloured blob from given two images
    def detect_color(self, image, color):
        color_range_upper = 0
        color_range_below = 0
        if (color == "red"):
            color_range_upper = self.RED_BLOB_HSV_COLOR_RANGE_UPPER
            color_range_below = self.RED_BLOB_HSV_COLOR_RANGE_BELOW
        elif (color == "blue"):
            color_range_upper = self.BLUE_BLOB_HSV_COLOR_RANGE_UPPER
            color_range_below = self.BLUE_BLOB_HSV_COLOR_RANGE_BELOW
        elif(color == "yellow"):
            color_range_upper = self.YELLOW_BLOB_HSV_COLOR_RANGE_UPPER
            color_range_below = self.YELLOW_BLOB_HSV_COLOR_RANGE_BELOW
        elif (color == 'green'):
            color_range_upper = self.GREEN_BLOB_HSV_COLOR_RANGE_UPPER
            color_range_below = self.GREEN_BLOB_HSV_COLOR_RANGE_BELOW
        elif (color == 'white'):
            color_range_upper = self.WHITE_BLOB_HSV_COLOR_RANGE_UPPER
            color_range_below = self.WHITE_BLOB_HSV_COLOR_RANGE_BELOW

    
        (x, y), small_area_yz = self.find_moments(image, color_range_below, color_range_upper)

        return np.array([x, y])
    

    # Calculates and returns the position and visibility of a coloured blob in a given image based on the given threshold
    # Makes use of hue, saturation, and value in hsv space to calculate required properties.
    # Appropriate values of hue, saturation, and value for different coloured blobs are determined using the 
    # python file hsv_color_finder.py in this package.
    def find_moments(self, image, color_range_below, color_range_upper):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yz_mask = cv2.inRange(hsv_image, color_range_below, color_range_upper)
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        yz_mask = cv2.dilate(yz_mask, kernel, iterations=3)
        # cv2.imshow('hsv' + str(color_range_below[0]), yz_mask)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # Obtain the moments of the binary image
        M = cv2.moments(yz_mask)
        # Calculate pixel coordinates for the centre of the blob
        m10 = M['m10']
        m00 = M['m00']
        m01 = M['m01']
        # small area determines if this blob is visible
        small_area = False
        if m00 < 1000000:

            small_area = True

        if m00 == 0:
            m00 = 0.0000001
        return ((int(m10 / m00), int(m01 / m00)), small_area)
