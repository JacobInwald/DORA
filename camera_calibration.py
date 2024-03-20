# Import required modules 
import cv2 
import numpy as np 
import os 
import glob 

# Define the dimensions of checkerboard 
CHECKERBOARD = (7, 7     ) 


def calibrate(path, corner='tl', save=True):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    calibration_flags = cv2.fisheye.CALIB_FIX_SKEW + cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC 
    # + cv2.fisheye.CALIB_CHECK_COND

    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    _img_shape = None
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = []
    N_OK = 0
    images = glob.glob(path + '/*.png')
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue
        if _img_shape == None:
            _img_shape = img.shape[:2]
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),criteria)
            imgpoints.append(corners)
            N_OK = len(objpoints)
            
            if 'corners' not in fname:
                cv2.imwrite(f'{fname[:-4]}_corners.png', img)
                
        if 'corners' not in fname:
            os.remove(fname)
            
    cv2.destroyAllWindows()
    print(f'Found {N_OK} valid images for calibration')
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    rms, _, _, _, _ = \
        cv2.fisheye.calibrate(
            objpoints,
            imgpoints,
            gray.shape[::-1],
            K,
            D,
            rvecs,
            tvecs,
            calibration_flags,
            (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
        
    print("K=np.array(" + str(K.tolist()) + ")")
    print("D=np.array(" + str(D.tolist()) + ")")
    print("DIM=" + str(img[::-1]))
    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    if save:
        np.save(f'data/calibration/{corner}/{corner}_mtx', matrix)
        np.save(f'data/calibration/{corner}/{corner}_dist', distortion)
        np.save(f'data/calibration/{corner}/{corner}_K', K)
        np.save(f'data/calibration/{corner}/{corner}_D', D)
    
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], r_vecs[i], t_vecs[i], matrix, distortion)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )
    return matrix, distortion, r_vecs, t_vecs


def take_photos(path, corner, i=0, save=False):
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
        if not save:
            k = cv2.waitKey(0)
            if k == ord('s'):
                cv2.imwrite(path + f'/{i}.png', frame)
                i += 1
            elif k == ord('q'):
                break
        else:
            k = cv2.waitKey(1)
            if k == ord('q'):
                break
            cv2.imwrite(path + f'/{i}.png', frame)
            i+=1

    cap.release()
    cv2.destroyAllWindows()

def test():
    cap = cv2.VideoCapture(0)
    
    
    
    tl_K = np.load('data/calibration/tl/tl_K.npy')
    tl_D = np.load('data/calibration/tl/tl_D.npy')
    tr_K = np.load('data/calibration/tr/tr_K.npy')
    tr_D = np.load('data/calibration/tr/tr_D.npy')
    bl_K = np.load('data/calibration/bl/bl_K.npy')
    bl_D = np.load('data/calibration/bl/bl_D.npy')
    br_K = np.load('data/calibration/br/br_K.npy')
    br_D = np.load('data/calibration/br/br_D.npy')
    
    # h_tl = np.load('data/calibration/tl/tl_h.npy')
    
    k = 0
    while k != ord('q'):
        ret, image = cap.read()
        h, w = image.shape[:2]
        tl = test_undistort(image[0:int(h/2), 0:int(w/2)], tl_K, tl_D)
        tr = test_undistort(image[0:int(h/2), int(w/2):w], tr_K, tr_D)
        bl = test_undistort(image[int(h/2):h, 0:int(w/2)], bl_K, bl_D)
        br = test_undistort(image[int(h/2):h, int(w/2):w], br_K, br_D)
        
        img = image_stitch([tl, tr, bl, br])
        cv2.imshow('img', img)
        k = cv2.waitKey(50)
        
        
    cap.release()
    cv2.destroyAllWindows()
        

def image_stitch(images):
    # Initialise
    tl, tr, bl, br = images
    tr_h, tr_w = tr.shape[:2]
    br_h, br_w = br.shape[:2]
    tl_h, tl_w = tl.shape[:2]
    bl_h, bl_w = bl.shape[:2]
    
    # top
    tl = tl[0:tl_h, 0:tl_w]
    tr = tr[0:tr_h, 0:tr_w]
    top = cv2.hconcat([tl, tr])

    # bottom
    bl = bl[0:bl_h, 0:bl_w]
    br = br[0:br_h, 0:br_w]
    bottom = cv2.hconcat([bl, br])
    
    top_h, top_w = top.shape[:2]
    bottom_h, bottom_w = bottom.shape[:2]
    
    top = top[0:top_h, 0:top_w]
    bottom = bottom[0:bottom_h, 0:bottom_w]
    img = cv2.vconcat([top, bottom])
    h, w = img.shape[:2]
    # img = img[40:h-25, 150:w-30]
    return img

def test_undistort(img, K, D, balance=0.0, dim2=(960, 600), dim3=(960, 600)):    
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort  
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, dim1, np.eye(3), balance=balance)
    dst = cv2.fisheye.undistortImage(img, K, D, K, new_K, dim1)
    return dst


corners = ['bl']
# , 'tr', 'bl', 'br']
for corner in corners:
    path = f"data/calibration/images/{corner}"
    # take_photos(path, corner, 1, True)
    calibrate(path, corner, save=True)

test()