# Import required modules 
import cv2 
import numpy as np 
import os 
import glob 
import cv2
import numpy as np
from pathlib import Path

# Define the dimensions of checkerboard 
CHECKERBOARD = (5, 8) 


def split_images(save=False, corner_spec=None):
    images = glob.glob(PATH + '*.png') 
    print(images)
    i = 0
    for filename in images: 
        image = cv2.imread(filename) 
        h, w = image.shape[:2]
        tl = image[0:int(h/2), 0:int(w/2)]
        tr = image[0:int(h/2), int(w/2):w]
        bl = image[int(h/2):h, 0:int(w/2)]
        br = image[int(h/2):h, int(w/2):w]
        if save:
            if not corner_spec:
                cv2.imwrite(filename + 'tl/' + f'{i}.png', tl)
                cv2.imwrite(filename + 'tr/' + f'{i}.png', tr)
                cv2.imwrite(filename + 'bl/' + f'{i}.png', bl)
                cv2.imwrite(filename + 'br/' + f'{i}.png', br)
            elif corner_spec == 'tl':
                cv2.imwrite(filename, tl)
            elif corner_spec == 'tr':
                cv2.imwrite(filename, tr)
            elif corner_spec == 'bl':
                cv2.imwrite(filename, bl)
            elif corner_spec == 'br':
                cv2.imwrite(filename, br)
        i += 1

def calibrate(path, corner='tl'):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[1],0:CHECKERBOARD[0]].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob(path + '/*.png')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)
    cv2.destroyAllWindows()


    # Perform camera calibration by 
    # passing the value of above found out 3D points (threedpoints) 
    # and its corresponding pixel coordinates of the 
    # detected corners (twodpoints) 
    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


    # Displaying required output 
    print(" Camera matrix:") 
    print(matrix) 

    print("\n Distortion coefficient:") 
    print(distortion) 

    print("\n Rotation Vectors:") 
    print(r_vecs) 

    print("\n Translation Vectors:") 
    print(t_vecs) 
    
    np.save(f'{corner}_mtx', matrix)
    np.save(f'{corner}_dist', distortion)
    # np.save('r_vecs', r_vecs)
    # np.save('t_vecs', t_vecs)
    
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], r_vecs[i], t_vecs[i], matrix, distortion)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )
    return matrix, distortion, r_vecs, t_vecs

def undistort(path, corner):
    matrix, distortion, r_vecs, t_vecs = calibrate(path, corner)
    images = glob.glob(path + '/*.png') 
    for filename in images: 
        image = cv2.imread(filename)
        h, w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(matrix, distortion, (w,h), 1, (w,h))
        dst = cv2.undistort(image, matrix, distortion, None, matrix)
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]
        cv2.imshow('img', dst)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def take_photos(path, corner, i=0):
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        h, w = frame.shape[:2]
        
        if corner == 'tl':
            frame = frame[0:int(h/2), 0:int(w/2)]
        elif corner == 'tr':
            frame = frame[0:int(h/2), int(w/2):w]
        elif corner == 'bl':
            frame = frame[int(h/2):h, 0:int(w/2)]
        elif corner == 'br':
            frame = frame[int(h/2):h, int(w/2):w]
        
        
        cv2.imshow('frame', frame)
        k = cv2.waitKey(0)
        
        if k == ord('s'):
            cv2.imwrite(path + f'/{i}.png', frame)
            i += 1
        elif k == ord('c'):
            continue
        elif k == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

PATH = "data/calibration/images/"
corner = 'tr'
path = PATH + corner
take_photos(path, corner, 1 )
# calibrate(path, corner)
undistort(path, corner)