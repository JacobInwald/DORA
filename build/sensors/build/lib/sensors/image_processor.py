import numpy as np
import os
import cv2
from .blob_detector import BlobDetector

class ImageProcessor:

    def __init__(self):
        self.calibratred = False
        self.position_estimator = BlobDetector()
        self.i = 0
        self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = None, None, None, None, None
        
        self.tl_mtx = np.load('data/calibration/top_left/tl_mtx.npy')
        self.tl_dist = np.load('data/calibration/top_left/tl_dist.npy')
        self.tr_mtx = np.load('data/calibration/top_right/tr_mtx.npy')
        self.tr_dist = np.load('data/calibration/top_right/tr_dist.npy')
        self.bl_mtx = np.load('data/calibration/bottom_left/bl_mtx.npy')
        self.bl_dist = np.load('data/calibration/bottom_left/bl_dist.npy')

    def runProcessor(self, img):

        # get original feed dimentions
        w, h = img.shape[:2]
    
        # Cropping images
        tl = img[0:int(w/2), 0:int(h/2)]
        tr = img[0:int(w/2), int(h/2):h]
        bl = img[int(w/2):w, 0:int(h/2)]
        br = img[int(w/2):w, int(h/2):h]
        
        # undistort
        #  posiiton of cameras may be wrong check!!!
        tl = self.undistort(tl, 0) # top-left
        tr = self.undistort(tr, 1) # top-right
        bl = self.undistort(bl, 2) # bottom-left
        br = self.undistort(br, 3) # bottom-right

        img = self.imageStitch([tl, tr, bl, br])

        # TODO: remove when working
        cv2.imshow("image", img)
        cv2.waitKey(100)


        pos_red = self.position_estimator.detect_color(img, 'red')
        pos_blue = self.position_estimator.detect_color(img, 'blue')
        angle = self.calculate_angle(pos_red, pos_blue)

        if pos_red[0] == 0 and pos_red[1] == 0:
            raise Exception("could not find red")
        if pos_blue[0] == 0 and pos_blue[1] == 0:
            raise Exception("could not find blue")

        return pos_red, np.deg2rad(angle)
    
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
 
    def undistort(self, img, id):
        
        if id == 0:
            matrix = self.tl_mtx
            distortion = self.tl_dist
        elif id == 1:
            matrix = self.tr_mtx
            distortion = self.tr_dist
        elif id == 2:
            matrix = self.bl_mtx
            distortion = self.bl_dist
        else: 
            matrix = self.tl_mtx
            distortion = self.tl_dist
        matrix = self.tl_mtx
        distortion = self.tl_dist    
        w, h = img.shape[:2]
        
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(matrix, distortion, (w,h), 1, (w,h))
        dst = cv2.undistort(img, matrix, distortion, None, matrix)
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]
        
        return dst
        
        # # TODO: test different mtx1, dist1 value to get a better image output
        # h,  w = img.shape[:2]
        # mtx1 = np.array([[100, 0.000000, w/2], [0.000000, 100, h/2], [0.000000, 0.000000, 1.000000]], dtype=np.float32)
        # if id == 0: # top-left
        #     dist1 = np.float32([-0.01, -0.02, -0.01, 0, 0])
        # elif id == 1: # top-right
        #     dist1 = np.float32([-0.01, -0.02, -0.01, 0.02, 0])
        # elif id == 2: # bottom-left
        #     dist1 = np.float32([-0.01, -0.02, 0.01, -0.01, 0])
        # else: # id = 3 #bottom-right
        #     dist1 = np.float32([-0.01, -0.02, 0.01, 0, 0])    
        # dst = cv2.undistort(img, mtx1, dist1)
        
        # # TODO: work out best crop distance
        # crop = 20
        # x,y,w,h = (crop, 0, w - 2*crop, h)
        # dst = dst[y:y+h, x:x+w]

        return dst

    def draw_robot_pos(self, img, pos1, pos2, angle):
        image_with_centers = cv2.circle(img, (int(pos1[0]), int(pos1[1])), 10, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(pos2[0]), int(pos2[1])), 10, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.line(image_with_centers, (pos1[0], pos1[1]), (pos2[0], pos2[1]), (255, 255, 255), 2)
        image_with_centers = cv2.putText(image_with_centers, 'Angle: ' + str(round(angle, 2)), (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        image_with_centers = cv2.putText(image_with_centers, 'Position: ' + str(round(pos1[0], 2)) + ', ' + str(round(pos1[1], 2)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow('image with centers', image_with_centers)
    
    def robot_present(self, pos):
        return pos[0] != 0 or pos[1] != 0
    
    def imageStitch(self, images):
        # Initialise
        tl, tr, bl, br = images
        tr_h, tr_w = tr.shape[:2]
        br_h, br_w = br.shape[:2]
        tl_h, tl_w = tl.shape[:2]
        bl_h, bl_w = bl.shape[:2]
        
        # right
        tr = tr[10:tr_h-20, 0:tr_w]
        br = br[0:br_h-10, 0:br_w]
        right = cv2.vconcat([tr, br])
        # left
        tl = tl[0:tl_h-30, 0:tl_w-10]
        bl = bl[10:bl_h, 10:bl_w]
        left = cv2.vconcat([tl, bl])
        
        l_h, l_w = left.shape[:2]
        left = left[0:l_h, 0:l_w-40]
        
        img = cv2.hconcat([left, right])
        h, w = img.shape[:2]
        img = img[40:h-25, 150:w-30]
        return img

    