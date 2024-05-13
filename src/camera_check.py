import cv2
import numpy as np

# Initialising the cameras
# Replace 0 and 1 with the indexes of your cameras.
cam_left = cv2.VideoCapture(0)
cam_right = cv2.VideoCapture(1)

# Check that both chambers are open
if not cam_left.isOpened() or not cam_right.isOpened():
    print("No se pueden abrir las cámaras")
    exit()

while True:
    # Read images from both cameras
    ret_right, frame_right = cam_right.read()
    ret_left, frame_left = cam_left.read()
    

    if not ret_left or not ret_right:
        print("No se pueden leer las imágenes de las cámaras")
        break

    # Display captured images
    cv2.imshow('Camara Izquierda', frame_left)
    cv2.imshow('Camara Derecha', frame_right)

    # Exit with the ‘q’ key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Freeing up resources and closing windows
cam_left.release()
#cam_right.release()
cv2.destroyAllWindows()
