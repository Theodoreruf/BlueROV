import cv2
import numpy as np
from pyzbar.pyzbar import decode

cap = cv2.VideoCapture(0)
width = 640
height = 480
cap.set(3,width)
cap.set(4,height)
EPSILON = 6


while True:
    success, img = cap.read()
    for barcode in decode(img):
        #print(barcode.data)
        myData = barcode.data.decode('utf-8')
        #print(myData)
        pts = np.array(barcode.polygon,np.int32) #récupère les coordonnées x y sous la forme d'un vecteur colonne
        pts = pts.reshape((-1,1,2))
        pts2 = barcode.rect
        rect = barcode.rect
        cv2.polylines(img,[pts],True,(255,0,255),3) # draw polygon
        cv2.rectangle(img,(rect.left, rect.top),(rect.left + rect.width, rect.top + rect.height),(0,255,0),2)
        top_left_corner = pts[0]
        bottom_left_corner = pts[1]
        bottom_right_corner = pts[2]
        top_right_corner = pts[3]
        # coins pas absolus mais dépendants de l'orientation du QRcode, quelle qu'elle soit top_left_corner sera en haut à gauche, etc

        # -------------------------------TESTS POLYGON EXTRACT
    # https://webdevdesigner.com/q/extract-points-coordinates-from-python-shapely-polygon-110691/
        # -------------------------------end TESTS POLYGON EXTRACT

        rleftside = rect.left
        rtopside = rect.top
        rrightside = rect.left + rect.width
        rbottomside = rect.top + rect.height
        rhalfheight = int(rect.height / 2)
        rhalfwidth = int(rect.width / 2)

        # *******************************
        y_11 = top_left_corner[~(top_left_corner == rleftside)].sum() # extrait y_11, coordonnée en y de l'angle haut gauche en retirant la
                                                                      # coordonnée en x qui est égale à rleftside (le côté gauche du carré rect encadrant
                                                                      # le QRcode sans suivre son orientation (affiché en vert)) et en faisant la somme
        # print("y_11:",y_11)
        x_11 = top_left_corner[~(top_left_corner == y_11)].sum() # extrait x_11, coordonnée en x de l'angle haut gauche en retirant la
                                                                 # coordonnée en y précédemment calculée et en faisant la somme
        # print("x_11:", x_11)
        # print(top_left_corner)
        # *******************************

        x_21 = bottom_left_corner[~(bottom_left_corner == rbottomside)].sum()
        # print("x_21:",x_21)
        y_21 = bottom_left_corner[~(bottom_left_corner == x_21)].sum()
        # print("y_21:", y_21)

        y_22 = bottom_right_corner[~(bottom_right_corner == rrightside)].sum()
        # print("y_22:", y_22)
        x_22 = bottom_right_corner[~(bottom_right_corner == y_22)].sum()
        # print("x_22:", x_22)

        x_12 = top_right_corner[~(top_right_corner == rtopside)].sum()
        # print("x_12:",x_12)
        y_12 = top_right_corner[~(top_right_corner == x_12)].sum()
        # print("y_12:", y_12)

        # ----------------------------------
        cv2.rectangle(img, (x_11, y_11), (x_11 + 3 , y_11 + 3), (255, 255, 0), 3) # point bleu clair, intersection entre rect et le carré polygon suivant
                                                                                  # l'angle du QRcode
        cv2.rectangle(img, (x_21, y_21), (x_21 + 3, y_21 + 3), (0, 255, 255), 3) # point jaune
        cv2.rectangle(img, (x_22, y_22), (x_22 + 3, y_22 + 3), (255, 0, 0), 3) # point bleu foncé
        cv2.rectangle(img, (x_12, y_12), (x_12 + 3, y_12 + 3), (0, 0, 255), 3) # point rouge
        # cv2.rectangle(img, (rleftside, rtopside), (rleftside + 1, rtopside + rhalfheight + 3), (0, 0, 255), 3)  # moitié sup rect

        # =================simulation contrôle robot

        if rleftside + rhalfwidth < width/2 - 3 * EPSILON:
            print("trop à gauche")
        elif rleftside + rhalfwidth > width/2 + 3 * EPSILON:
            print("trop à droite")
        else:
            print("=> horizontale OK")
            if rtopside + rhalfheight < height / 2 - 3 * EPSILON:
                print("trop haut")
            elif rtopside + rhalfheight > height / 2 + 3 * EPSILON:
                print("trop bas")
            else:
                print("==> verticale OK")
                if y_11 > rtopside + EPSILON and y_11 < rtopside + rhalfheight:
                    print("penché trop à gauche")
                elif y_11 < rbottomside - EPSILON and y_11 >= rbottomside - rhalfheight:
                    print("penché trop à droite")
                else:
                    print("===> orientation OK")
                    if rect.width < 200 - 2 * EPSILON:
                        print("trop loin")
                    elif rect.width > 200 + 2 * EPSILON:
                        print("trop près")
                    else:
                        print("====> distance OK")
                        print(" ******ALL OK******")

    cv2.imshow('Result', img)
    cv2.waitKey(1)