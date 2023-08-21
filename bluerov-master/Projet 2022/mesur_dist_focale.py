import cv2 as cv
import numpy as np
import pyzbar.pyzbar as pyzbar


print("DEBUT TEST")
cap = cv.VideoCapture(1)
ret, color_frame = cap.read()
print("TEST EN COURS")
if not cap.isOpened():
    print("Cannot open camera")
    exit()
print("TEST FINI")

font = cv.FONT_HERSHEY_PLAIN
color_contour = (0,255,255)
color_infos = (0,55,255)
color_point=(0,125,0)

# variables
# distance from camera to object(face) measured
KNOWN_DISTANCE = 50  # centimeter
# width of face in the real world or Object Plane
KNOWN_WIDTH = 17  # centimeter
# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
fonts = cv.FONT_HERSHEY_COMPLEX
#cap = cv.VideoCapture(0)

# face detector object
#face_detector = cv.CascadeClassifier("haarcascade_frontalface_default.xml")


# focal length finder function
def focal_length(measured_distance, real_width, width_in_rf_image):
    """
    This Function Calculate the Focal Length(distance between lens to CMOS sensor), it is simple constant we can find by using
    MEASURED_DISTACE, REAL_WIDTH(Actual width of object) and WIDTH_OF_OBJECT_IN_IMAGE
    :param1 Measure_Distance(int): It is distance measured from object to the Camera while Capturing Reference image

    :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 14.3 centimeters)
    :param3 Width_In_Image(int): It is object width in the frame /image in our case in the reference image(found by Face detector)
    :retrun focal_length(Float):"""
    focal_length_value = (width_in_rf_image * measured_distance) / real_width
    return focal_length_value


# distance estimation function
def distance_finder(focal_length, real_face_width, face_width_in_frame):
    """
    This Function simply Estimates the distance between object and camera using arguments(focal_length, Actual_object_width, Object_width_in_the_image)
    :param1 focal_length(float): return by the focal_length_Finder function

    :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 5.7 Inches)
    :param3 object_Width_Fra : distance Estimated
    """
    if face_width_in_frame > 0: #on évite la division par zéro 
        distance = (real_face_width * focal_length) / face_width_in_frame
    else:
        distance = 0
    return distance

_, frame = cap.read()
#print('dimensions', frame.shape)

decodedObjects = pyzbar.decode(frame)
    #elements = cv2.findContours(decodedObjects,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
for obj in decodedObjects:
    #print("Data", obj.data)
    pts=np.array([obj.polygon], np.int32)
    #pts=pts.reshape((-1,1,2))
    cv.polylines(frame, [pts],True, color_contour,5)
    pts2=obj.rect

    # Show distance for a specific point at mouse position
    #cv2.circle(frame, point[3], 4, (0, 0, 255))
    #cv2.putText(frame, "position mouse", (point[0], point[1]), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)


    cv.putText(frame, str(obj.data), (pts2[0], pts2[1]), font, 2,color_infos, 2)
    cv.circle(frame, (int(pts2[0]+pts2[2]/2),int(pts2[1]+pts2[3]/2)), 5 ,color_point,10) #point centrzl de l'objet
    #cv2.circle(frame, (int(pts2[0]),int(pts2[1])), 5 ,color_point,10) #point centrzl de l'objet

    
    print("x y w h : ", pts2)
    print(pts2[2])
    #print("Center of QR code is at : ", (int(pts2[0]+pts2[2]/2),int(pts2[1]+pts2[3]/2)))  


focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, pts2[2])
print("DISTANCE FOCALE = ",focal_length_found)

