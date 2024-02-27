import cv2
import numpy as np
from .blob_detector import BlobDetector

class ImageProcessor:
    """
    Processes the image given
    """


    def __init__(self):
        self.calibratred = False
        self.position_estimator = BlobDetector()
        self.i = 0

    def runProcessor(self, frame):
        return self.__imageSegmentation(frame)

    # Perform image processing
    def __imageSegmentation(self, img):       

        # get original feed dimentions
        width, height = img.shape[:2]
    
        # Cropping images
        cropped_image1 = img[0:int(width / 2), 0:int(height / 2)]
        cropped_image2 = img[int(width / 2):width, 0:int(height / 2)]
        cropped_image3 = img[0:int(width / 2), int(height / 2):height]
        cropped_image4 = img[int(width / 2):width, int(height / 2):height]
        
        images = [cropped_image1, cropped_image2, cropped_image3, cropped_image4]
        
        

        vis = self.__imageStitch(images)

        pos = self.position_estimator.detect_color(vis, 'red')
        pos_blue = self.position_estimator.detect_color(vis, 'blue')
        pos_yellow = self.position_estimator.detect_color(vis, 'yellow')
        pos_green = self.position_estimator.detect_color(vis, 'green')
        angle = self.calculate_angle(pos, pos_blue)

        if (pos[0] < 5 and pos[1] < 5):
            angle = -999
            pos[0] = -999
            pos[1] = -999
        elif (pos_blue[0] < 5 and pos_blue[1] < 5):
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
    
