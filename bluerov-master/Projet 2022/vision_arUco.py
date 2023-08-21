import cv2
from object_detector import *
import numpy as np

# Load Aruco detector
parameters = cv2.aruco.DetectorParameters_create()
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100)
color_infos = (0,55,255)

# Load Object Detector
detector = HomogeneousBgDetector()

# Load Cap
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

matrix_coefficients = np.array([[503.7, 0, 313.7],[0, 503.4, 243.3],[0, 0, 1]])#camera matrix for f = 50mm with chipsize = 0.00345 mm and 2048x2448px
distortion_coefficients = np.ndarray([0]) #distortion coefficients



while True:
    _, img = cap.read()

    # Get Aruco marker
    corners, _, _ = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    if corners:

        # Draw polygon around the marker
        int_corners = np.int0(corners)
        cv2.polylines(img, int_corners, True, (0, 255, 0), 5)
        
        # Recup corners markers
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, matrix_coefficients,
                                                                           distortion_coefficients)
        (rvec - tvec).any()     # get rid of that nasty numpy value array error
        cv2.aruco.drawDetectedMarkers(img, corners)     # Draw A square around the markers
        cv2.aruco.drawAxis(img, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)      # Draw axis
        
    # Rotation matrix
    print("Rotation matrix", rvec)

    # Translation matrix
    print("Translation matrix", tvec)
                
    cv2.imshow("Image", img)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()