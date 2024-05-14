import cv2
import numpy as np
import time
import json


# Defining the size of the chessboard
chessboard_size = (7, 9) #number of squares - 1

# Prepare 3D object points in real space (0,0,0), (1,0,0), (2,0,0) ..., as examples.
objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points of all images.
objpoints = [] # 3d points in real space
imgpoints_left = [] # 2d points in the image plane of the left camera
imgpoints_right = [] # 2d points in the image plane of the rigth camera

# Start video capture
cap_left = cv2.VideoCapture(1)  # Index of the left camera
cap_right = cv2.VideoCapture(0) # Index of the rigth camera

while True:
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()

    if not ret_left or not ret_right:
        print("Error en la captura de video")
        break

    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

    ret_left, corners_left = cv2.findChessboardCorners(gray_left, chessboard_size, None)
    ret_right, corners_right = cv2.findChessboardCorners(gray_right, chessboard_size, None)

    if ret_left and ret_right and cv2.waitKey(1) == ord('c'):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)
        objpoints.append(objp)

        corners2_left = cv2.cornerSubPix(gray_left, corners_left, (11,11), (-1,-1), criteria)
        imgpoints_left.append(corners2_left)

        corners2_right = cv2.cornerSubPix(gray_right, corners_right, (11,11), (-1,-1), criteria)
        imgpoints_right.append(corners2_right)

        # Drawing and showing corners
        cv2.drawChessboardCorners(frame_left, chessboard_size, corners2_left, ret_left)
        cv2.drawChessboardCorners(frame_right, chessboard_size, corners2_right, ret_right)
        cv2.imshow('Frame Izquierda', frame_left)
        cv2.imshow('Frame Derecha', frame_right)
        
        print('captura')

        # save numbered images
        cv2.imwrite('images/img_left_' + str(len(imgpoints_left)) + '.png', frame_left)
        cv2.imwrite('images/img_right_' + str(len(imgpoints_right)) + '.png', frame_right)
        time.sleep(3)
    else:
        cv2.imshow('Frame Izquierda', frame_left)
        cv2.imshow('Frame Derecha', frame_right)

    if cv2.waitKey(1) == ord('q'):
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()

print('inicio calibracion')

# calibrate the left camera
retval, cameraMatrix1, distCoeffs1, rvecs1, tvecs1 = cv2.calibrateCamera(
     objpoints, imgpoints_left, gray_left.shape[::-1], None, None
    )
if retval:
    print('calibracion izquierda exitosa')

# calibrate the rigth camera
retval, cameraMatrix2, distCoeffs2, rvecs2, tvecs2 = cv2.calibrateCamera(
     objpoints, imgpoints_right, gray_right.shape[::-1], None, None
    )
if retval:
    print('calibracion derecha exitosa')

# we calibrate in stereo taking into account the calibrations of the cameras.
retval, _cameraMatrix1, _distCoeffs1, _cameraMatrix2, _distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right,
    cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, gray_left.shape[::-1]
)
if retval:
    print('calibracion estereo exitosa')

print("Matriz de la cámara izquierda:", cameraMatrix1)
print("Coeficientes de distorsión de la cámara izquierda:", distCoeffs1)
print("Matriz de la cámara derecha:", cameraMatrix2)
print("Coeficientes de distorsión de la cámara derecha:", distCoeffs2)
print("Matriz de rotación:", R)
print("Vector de traslación:", T)

# Saving calibration parameters to files
np.save('cameraMatrix1.npy', cameraMatrix1)
np.save('distCoeffs1.npy', distCoeffs1)
np.save('cameraMatrix2.npy', cameraMatrix2)
np.save('distCoeffs2.npy', distCoeffs2)
np.save('R.npy', R)
np.save('T.npy', T)


