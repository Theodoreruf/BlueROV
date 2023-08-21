#from cv2 import HISTCMP_BHATTACHARYYA
import numpy as np
import cv2
import sys, time, math
from hiya import *

# #--- Define Tag
id_to_find  = 69
marker_size  = 25 #- [cm]

camera_matrix = np.array([[503.7, 0, 313.7],[0, 503.4, 243.3],[0, 0, 1]])#camera matrix for f = 50mm with chipsize = 0.00345 mm and 2048x2448px
#print('corners', matrix_coefficients)
camera_distortion = np.ndarray([0]) #distortion coefficients

#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
#-- Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    
    #-- Read the camera frame
    ret, frame = cap.read()
    
    #draw camera axis
    cv2.circle(frame, (int(xc),int(yc)), 3 ,(80,0,255),5) #point centrzl de l'objet
    cv2.line(frame,(int(xc),int(yc)),(int(xc)+50,int(yc)),(0,0,255),2)
    cv2.line(frame,(int(xc),int(yc)),(int(xc),int(yc)+50),(0,255,0),2)
    
    frame = aruco_detection(frame, camera_matrix, camera_distortion, id_to_find, marker_size)
    
    #--- Display the frame
    cv2.imshow('frame', frame)

    #--- use 'echap' to quit
    key = cv2.waitKey(1)
    if key == 27:
        cap.release()
        cv2.destroyAllWindows()
        break