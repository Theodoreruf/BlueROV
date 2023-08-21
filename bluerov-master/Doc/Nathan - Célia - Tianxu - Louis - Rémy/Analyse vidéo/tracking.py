from __future__ import print_function
import sys
import cv2
from random import randint

#Differents types de tracking possible
trackerTypes = ['BOOSTING', 'MIL', 'KCF','TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']

def createTrackerByName(trackerType):
  #Cree un tracker selon le type de tracking
  if trackerType == trackerTypes[0]:
    tracker = cv2.TrackerBoosting_create()
  elif trackerType == trackerTypes[1]:
    tracker = cv2.TrackerMIL_create()
  elif trackerType == trackerTypes[2]:
    tracker = cv2.TrackerKCF_create()
  elif trackerType == trackerTypes[3]:
    tracker = cv2.TrackerTLD_create()
  elif trackerType == trackerTypes[4]:
    tracker = cv2.TrackerMedianFlow_create()
  elif trackerType == trackerTypes[5]:
    tracker = cv2.TrackerGOTURN_create()
  elif trackerType == trackerTypes[6]:
    tracker = cv2.TrackerMOSSE_create()
  elif trackerType == trackerTypes[7]:
    tracker = cv2.TrackerCSRT_create()
  else:
    tracker = None
    print('Incorrect tracker name')
    print('Available trackers are:')
    for t in trackerTypes:
      print(t)
  return tracker
#Attribue chemin de la video a la variable videoPath
videoPath = "video-fish.mp4"

#Cree un objet capture de video pour lire la video
cap = cv2.VideoCapture(videoPath)

#Lis la premiere image
success, frame = cap.read()
#Quitte si impossible de lire l'image
if not success:
  print('Failed to read video')
  sys.exit(1)
  #Selection des rectangls / contours
bboxes = []
colors = []

#La fonction selectROI ne permet pas de selectionner plusieurs objets en Python
#On va donc rappeler cette fonction dans une boucle pour effectuer plus d'une selection
while True:
  #Dessine des rectangles autour de l'objet selectionne
  #Depart du rectangle au point en bas a gauche a droite
  bbox = cv2.selectROI('MultiTracker', frame)
  bboxes.append(bbox)
  #Couleur de rectangle aleatoire
  colors.append((randint(0, 255), randint(0, 255), randint(0, 255)))
  print("Appuyer sur q pour arreter la selection et commencer le tracking")
  print("Appuyer sur n'importe quelle autre touche pour selectionner un autre objet")
  k = cv2.waitKey(0) & 0xFF
  if (k == 113):  #On appuie sur q
  #Si on appuie sur q le programme se lance
    break

print('Selectionner objets a traquer {}'.format(bboxes))

#Specifie le type de tracking utilise
trackerType = "CSRT"

#Cree un objet pour traquer plusieurs objects selectionnes
multiTracker = cv2.MultiTracker_create()

#Initialisation MultiTracker
for bbox in bboxes:
  multiTracker.add(createTrackerByName(trackerType), frame, bbox)

#Traitement de la video et tracking des objets selectionnes
while cap.isOpened():
  success, frame = cap.read()
  if not success:
    break

  #Met a jour la localisation des objets dans l'image qui arrive
  success, boxes = multiTracker.update(frame)

  #Dessine les rectangles autour des objets choisis
  for i, newbox in enumerate(boxes):
    p1 = (int(newbox[0]), int(newbox[1]))
    p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
    cv2.rectangle(frame, p1, p2, colors[i], 2, 1)

  #Affiche l'image
  cv2.imshow('MultiTracker', frame)


  # Appuyer sur Echap pour quitter le programme
  if cv2.waitKey(1) & 0xFF == 27:  # Esc pressed
    break
