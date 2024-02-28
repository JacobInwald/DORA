import rclpy 
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Point
from dora_msgs.msg import Pose
import copy


class GpsNode(Node):
    """
    Represents a GPS (Overhead camera).

    TODO: Implement GPS node (Wilfredo?)
    Subscribes to the overhead camera.
    Publishes global location of robot.

    The control flow will be as follows:
     - The GPS node will subscribe to the overhead camera
     - Calculate the current global position and rotation of the robot
     - Publish Pose of the robot with topic name "gps"

    Attributes:
    - pos: numpy.ndarray - The current position of the robot.
        - 1 is 1 meter in this scale
    - rot: float - The current rotation of the robot. ()
        - 0 is facing the positive x-axis
    """

    def __init__(self):
        super().__init__('gps')
        self.LAT_MIN = 0.0
        self.LAT_MAX = 1.2631578947
        self.LONG_MIN = 0.0
        self.LONG_MAX = 1.0
        self.IMAGE_Y = 950
        self.IMAGE_X = 1200

        self.robotPosition = None
        self._processor = ImageProcessor()

        self.gps_pub = self.create_publisher(Pose, '/gps', 10)
        self.pos_pub = self.create_publisher(Point, '/robot_position', 10)
        self.bench2_pub = self.create_publisher(Point, '/bench2_position_longlat', 10)
        self.bench3_pub = self.create_publisher(Point, '/bench3_position_longlat', 10)
        self.pos_pub_longlat = self.create_publisher(Point, '/robot_position_longlat', 10)
        return 
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(1/30, self.process_video)

    def process_video(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture video frame')
            return

        frame = cv2.resize(frame, None, fx=1, fy=1, interpolation=cv2.INTER_AREA)
        cv2.imwrite('camera_input.jpg', frame)

        robot_position, angle, position_yellow, position_green = self._processor.runProcessor(frame)
        position_in_point_robot = Point(x=float(robot_position[1]), y=float(robot_position[0]), z=angle)
        position_in_point_bench2 = Point(x=float(position_yellow[1]), y=float(position_yellow[0]), z=0)
        position_in_point_bench3 = Point(x=float(position_green[1]), y=float(position_green[0]), z=0)

        self.pos_pub.publish(position_in_point_robot)
        self.pos_pub_longlat.publish(self.convert_to_longlat(position_in_point_robot))
        self.bench2_pub.publish(self.convert_to_longlat(position_in_point_bench2))
        self.bench3_pub.publish(self.convert_to_longlat(position_in_point_bench3))

    def convert_to_longlat(self, position_in_point):
        change_in_Long = self.LONG_MAX - self.LONG_MIN
        change_in_lat = self.LAT_MAX - self.LAT_MIN
        long_conversion_constant = change_in_Long / self.IMAGE_Y
        lat_conversion_constant = change_in_lat / self.IMAGE_X

        point_to_convert = copy.deepcopy(position_in_point)
        point_to_convert.x = long_conversion_constant * point_to_convert.x
        point_to_convert.y = lat_conversion_constant * point_to_convert.y

        return point_to_convert

class blob_detector:
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
        # Obtain the moments of the binary image
        M = cv2.moments(yz_mask)
        # Calculate pixel coordinates for the centre of the blob
        m10 = M['m10']
        m00 = M['m00']
        m01 = M['m01']
        # small area determines if this blob is visible
        small_area = False
        if m00 < 1000000:
            # print("Small moment!")
            small_area = True

        if m00 == 0:
            m00 = 0.000001

        return (int(m10 / m00), int(m01 / m00)), small_area


class ImageProcessor:

    def __init__(self):
        self.calibratred = False
        self.position_estimator = blob_detector()
        self.i = 0
        return

    def runProcessor(self, frame):
        return self.__imageSegmentation(frame)

    # Perform image processing
    def __imageSegmentation(self, image):

        img = image
        cv2.imwrite(("img_" +str(self.i)+ '.jpg'), img)
        self.i+=1
        print(img.shape)  # Print image shape
        cv2.imshow("original", img)
       


        # get original feed dimentions
        width, height = img.shape[:2]
    
        # Cropping images
        cropped_image1 = img[0:int(width / 2), 0:int(height / 2)]
        cropped_image2 = img[int(width / 2):width, 0:int(height / 2)]
        cropped_image3 = img[0:int(width / 2), int(height / 2):height]
        cropped_image4 = img[int(width / 2):width, int(height / 2):height]
        
        images = [cropped_image1, cropped_image2, cropped_image3, cropped_image4]

        vis = self.__imageStitch(images)
        # print(vis.shape)

        pos = self.position_estimator.detect_color(vis, 'red')
        pos_blue = self.position_estimator.detect_color(vis, 'blue')
        pos_yellow = self.position_estimator.detect_color(vis, 'yellow')
        pos_green = self.position_estimator.detect_color(vis, 'green')
        angle = self.calculate_angle(pos, pos_blue)

        if pos[0] < 5 and pos[1] < 5:
            angle = -999
            pos[0] = -999
            pos[1] = -999
        elif pos_blue[0] < 5 and pos_blue[1] < 5:
            angle = -999

        
        # print(pos)
        # cv2.imshow('gps', vis)
        self.draw_robot_pos(vis, pos, pos_blue, angle)
        # print(vis.shape)
        cv2.waitKey(4)

        return pos, angle, pos_yellow, pos_green
    
    def calculate_angle(self, pos_red, pos_blue):
        red_x = pos_red[0]
        red_y = pos_red[1]

        blue_x = pos_blue[0]
        blue_y = pos_blue[1]
        

        vector_1 = [1, 0]
        vector_2 = pos_blue - pos_red

        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_2, unit_vector_1)
        angle = np.arccos(dot_product)

        if red_y < blue_y:
            angle = 2*np.pi - angle
        return np.rad2deg(angle)
 
    def draw_robot_pos(self, img, pos1, pos2, angle):
        image_with_centers = cv2.circle(img, (int(pos1[0]), int(pos1[1])), 10, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(pos2[0]), int(pos2[1])), 10, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.line(image_with_centers, (pos1[0], pos1[1]), (pos2[0], pos2[1]), (255, 255, 255), 2)
        image_with_centers = cv2.putText(image_with_centers, 'Angle: ' + str(round(angle, 2)), (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        image_with_centers = cv2.putText(image_with_centers, 'Position: ' + str(round(pos1[0], 2)) + ', ' + str(round(pos1[1], 2)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow('image with centers', image_with_centers)
    

    def robot_present(self, pos):
        return pos[0] != 0 or pos[1] != 0
    
    def __imageStitch(self, images):

        #top two
        h1, w1 = images[0].shape[:2]
        h2, w2 = images[2].shape[:2]

        #create empty matrix
        vis = np.zeros((h1+h2, w1+w2,3), np.uint8)

        #top left
        vis[5:h1+5, 10:w1] = images[0][:600, 10:960]

        #top right
        vis[:h1, w1-70:w1+w2-160] = images[2][:600, 90:960]

        #bottom left
        vis[h1-70:h1+h2-110, :w1-30] = images[1][40:600, 30:960]

        #bottom right
        vis[h1-70:h1+h2-100, w1-140:w1+w2-150] = images[3][30:600, 10:960]

        # cv2.imshow("combined4",vis)
        # cv2.waitKey(0)

        return vis[100:1050, 450:1650]

    def __colourSpaceCoordinate(self, image):

        red_u = (20, 20, 256)
        red_l = (0, 0, 100)
        climits = [[red_l, red_u]]

        masks = [cv2.inRange(image, climit[0], climit[1]) for climit in climits]
        maskJs = [cv2.cvtColor(mask, cv2.COLOR_BGR2RGB) for mask in masks]

        frames = [(image & maskJ) for maskJ in maskJs]

        gray_frames = [cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) for frame in frames]

        jThreshes = [cv2.threshold(gray_frame, 1, 255, cv2.THRESH_BINARY) for gray_frame in gray_frames]

        jcontours = [cv2.findContours(jthresh[1], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) for jthresh in jThreshes]

        cords = []
        radiuslist = []
        for jcontour in jcontours:
            # print(jcontour)
            try:
                Gradius = 0
                (Gx, Gy), Gradius = cv2.minEnclosingCircle(self.mergeContors(jcontour[0]))
                radiuslist.append(Gradius)
                # print(Gradius)
                if Gradius < 2:  # Filter out single pixel showing
                    cords.append([-1, -1])
                else:
                    cords.append([Gx, Gy])

            except:
                cords.append([-1, -1])
                radiuslist.append(0)

        contourDic = {"Red": {'x': cords[3][0], 'y': cords[3][1]}}

        im_copy = image.copy()

        for i in range(len(cords)):
            cv2.circle(im_copy, (int(cords[i][0]), int(cords[i][1])), 2, (255, 255, 255), -1)
            cv2.putText(im_copy, list(contourDic.keys())[i], (int(cords[i][0]) - 50, int(cords[i][1]) - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.circle(im_copy, (int(cords[i][0]), int(cords[i][1])), int(radiuslist[i]), (0, 255, 0), 1)

        return contourDic, im_copy

    def __mergeContors(self, ctrs):
        list_of_pts = []
        for c in ctrs:
            for e in c:
                list_of_pts.append(e)
        ctr = np.array(list_of_pts).reshape((-1, 1, 2)).astype(np.int32)
        ctr = cv2.convexHull(ctr)
        return ctr

    def sdpPixelToDegrees(self):
        return
    

def main():
    rclpy.init()
    node = GpsNode()
    rclpy.spin(node)
    rclpy.destroy_node(node)
    rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     gps_simulator = GPSSimulator()

#     try:
#         rclpy.spin(gps_simulator)
#     except KeyboardInterrupt:
#         gps_simulator.get_logger().info('GPS Simulator stopped cleanly')
#     except BaseException as e:
#         # Log the exception with its traceback without using `exc_info`
#         gps_simulator.get_logger().error(f'Exception in GPS Simulator: {e}')
#         import traceback
#         gps_simulator.get_logger().error(traceback.format_exc())
#     finally:
#         gps_simulator.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()
