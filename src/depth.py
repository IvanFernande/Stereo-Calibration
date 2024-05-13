import cv2
import numpy as np
import time
from matplotlib import pyplot as plt

# loads calibration data

cameraMatrix1=np.load('cameraMatrix1.npy')
distCoeffs1=np.load('distCoeffs1.npy')
cameraMatrix2=np.load('cameraMatrix2.npy')
distCoeffs2=np.load('distCoeffs2.npy')
R=np.load('R.npy')
T=np.load('T.npy')


print("Matriz de la cámara izquierda:", cameraMatrix1)
print("Coeficientes de distorsión de la cámara izquierda:", distCoeffs1)
print("Matriz de la cámara derecha:", cameraMatrix2)
print("Coeficientes de distorsión de la cámara derecha:", distCoeffs2)
print("Matriz de rotación:", R)
print("Vector de traslación:", T)

# Start video capture
cap_left = cv2.VideoCapture(1)  # left camera index
cap_right = cv2.VideoCapture(0) # rigth camera index

factor=1
blockSize=15

while True:
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()

    if not ret_left or not ret_right:
        print("Error en la captura de video")
        break

    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

    if ret_left and ret_right:

        # Obtain parameters for image rectification
        R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
            cameraMatrix1, distCoeffs1,
            cameraMatrix2, distCoeffs2,
            gray_left.shape[::-1], R, T)
            

        # Obtain rectification maps
        left_map1, left_map2 = cv2.initUndistortRectifyMap(
            cameraMatrix1, distCoeffs1, R1, P1, gray_left.shape[::-1], cv2.CV_16SC2)
            #cameraMatrix1, distCoeffs1, R1, P1, frame_left.shape[:-1], cv2.CV_8UC1)
        right_map1, right_map2 = cv2.initUndistortRectifyMap(
            cameraMatrix2, distCoeffs2, R2, P2, gray_right.shape[::-1], cv2.CV_16SC2)
            #cameraMatrix2, distCoeffs2, R2, P2, frame_right.shape[:-1], cv2.CV_8UC1)

        # Apply rectification
        left_rectified = cv2.remap(gray_left, left_map1, left_map2, cv2.INTER_LINEAR)
        right_rectified = cv2.remap(gray_right, right_map1, right_map2, cv2.INTER_LINEAR)

        # Initialise the stereo matching block
        stereo = cv2.StereoBM_create(numDisparities=16*factor, blockSize=blockSize)

        # Calculate the disparity map
        disparity = stereo.compute(left_rectified, right_rectified)

        # Normalise the disparity map for visualisation
        disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Apply a colourmap to improve visualisation
        disparity_color = cv2.applyColorMap(disparity_normalized, cv2.COLORMAP_JET)

        # Display the normalised disparity map and the original images.
        cv2.imshow('Normalized Disparity Map', disparity_color)
        cv2.imshow('Izq', gray_left)
        cv2.imshow('Der', gray_right)
        

    k = cv2.waitKey(1)
    if k == ord('q'):
        break    
    if k == ord('+'):
        factor +=1
        print('factor: ',factor)
        time.sleep(0.5)
    if k == ord('-'):
        factor -=1
        print('factor: ',factor)
        time.sleep(0.5)

    if k == ord('*'):
        blockSize +=2
        print('blockSize: ',blockSize)
        time.sleep(0.5)
    if k == ord('/'):
        blockSize -=2
        print('blockSize: ',blockSize)
        time.sleep(0.5)

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
