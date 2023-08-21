import cv2
import numpy as np

#Decommenter si utilisation de la camera (webcam)
#capture = cv2.VideoCapture(0)
#Ouverture d'une video ou de la camera distante, ex le RaspeberryPi
capture = cv2.VideoCapture('video-fish.avi')

#Video pas encore lancee donc pas de frame precedente pour l'instant
prevFrame = None

while True:
    #Lit une image de la video et la place dans le buffer
    (grabbed,frame) = capture.read()

    #Si la video n'est pas lue correctement dans le buffer, on quitte la boucle
    if not grabbed:
        break

    #On passe l'image en niveau de gris et on lui applique un flou gaussien pour supprimmer le bruit
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(25,25), 0)


    if prevFrame is None:
        prevFrame = gray

    #On fait la difference absolue de l'image actuelle et la precedente
    frameDelta = cv2.absdiff(prevFrame,gray)
    #On fait un seuillage binaire sur cette nouvelle image
    thresh = cv2.threshold(frameDelta, 7, 255, cv2.THRESH_BINARY)[1]
    kernel = np.ones((11,11),np.uint8)
    #Puis on la dilate pour pouvoir plus facilement trouver les contours par la suite
    thresh = cv2.dilate(thresh, kernel, iterations=2)

    #Recherche des contours des objets de l'image dilate
    (contr,hrchy) = cv2.findContours(thresh.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    #Ce mask va nous servir a encadrer l'objet de la couleur de celui ci
    mask = np.zeros(frame.shape[:2],np.uint8)

    for c in contr:

        #Tous les petits objets de surface inferieure a l'intervalle sont ignores
        if cv2.contourArea(c) < 10 or cv2.contourArea(c) > 5000:
			continue

        #Recherche des coordonnees de l'objet
        (x,y,w,h) = cv2.boundingRect(c)
        #On adapte le mask en fonction de l'objet
        mask[y:y+h, x:x+w] = 255

        #Recuperation de la couleur du pixel du milieu
        #On suppose qu'on est dans le cas ou l'objet a une couleur uniforme
        (bl,gr,re) = frame[y+(h/2),x+(w/2)]
        bl = int(bl)
        gr = int(gr)
        re = int(re)
        couleur ='B:'+ str(bl) + 'G:' + str(gr) + 'R:' + str(re)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame,couleur,(x+w,y), font, 1, (255,255,255), 2, cv2.LINE_AA)

        #Le rectangle prend la couleur de l'objet et les nuances de bleu,vert,rouge sont affiche sur le cote
        cv2.rectangle(frame,(x,y),(x+w,y+h),(bl,gr,re),3)

    masked_img = cv2.bitwise_and(frame,frame,mask=mask)

    #On affiche la video avec les rectangles
    cv2.imshow('contour',frame)
    cv2.waitKey(50)

    #L'image actuelle devient la future image precedente
    prevFrame = gray

    #Quitte la capture video lorsque la touche q est appuye
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()
