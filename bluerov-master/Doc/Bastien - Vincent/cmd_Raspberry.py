# -*- coding: utf-8 -*-

# Auteurs: Vincent Causse et Bastien Muraccioli MEA4 2020-2021
# Ce programme arme le robot et envoi des commandes RC depuis Companion vers Ardusub
# Ce programme n'a pas été testé il se peut que la connexion pymavlink soit à modifier
# Voir si le programme reste bloqué à la ligne 25 (wait heat beat...)
# Pour fonctionner la caméra du BlueRov doit être configuré en 640x480p (http://192.168.2.2:2770/camera)
# et en multiudpsink voir https://www.ardusub.com/developers/opencv.html

# Imports
from pymavlink import mavutil
import time
import cv2
import numpy as np
from pyzbar.pyzbar import decode

# Variables globales pour détection
# detection QRCode
video = Video(port=4777)
width = 640
height = 480
EPSILON = 6
Wanted_Distance = 200

# Create the connection see https://www.ardusub.com/developers/pymavlink.html
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Function to detect QRcode's position
def getpositionQRcode(barcode, EPSILON, width, height,Wanted_Distance):  # renvoie le pourcentage d'exactitude par rapport à position désirée
                                                                         # donc pourcentage d'éloignement = 1 - position
    position = float

    pts = np.array(barcode.polygon, np.int32)  # récupère les coordonnées x y sous la forme d'un vecteur colonne
    pts = pts.reshape((-1, 1, 2))

    rect = barcode.rect

    rleftside = rect.left                   # coordonée x côté gauche du rectangle
    rtopside = rect.top                     # coordonée y côté haut du rectangle
    rrightside = rect.left + rect.width     # coordonée x côté droit du rectangle
    rbottomside = rect.top + rect.height    # coordonée y côté bas du rectangle
    rhalfheight = int(rect.height / 2)      # coordonée y centre du rectangle
    rhalfwidth = int(rect.width / 2)        # coordonée x centre du rectangle

    top_left_corner = pts[0]
    # bottom_left_corner = pts[1]
    # bottom_right_corner = pts[2]
    # top_right_corner = pts[3]
    y_11 = top_left_corner[~(top_left_corner == rleftside)].sum()

    if rleftside + rhalfwidth < width / 2 - 3 * EPSILON:
        # # print("trop à gauche")
        position = 34 + (rleftside + rhalfwidth) / (width * 0.5 - 3 * EPSILON)
    elif rleftside + rhalfwidth > width / 2 + 3 * EPSILON:
        # # print("trop à droite")
        position = 33 + (width * 0.5 - 3 * EPSILON) / (rleftside + rhalfwidth)
    else:
        # # print("=> horizontale OK")
        if rtopside + rhalfheight < height / 2 - 3 * EPSILON:
            # # print("trop haut")
            position = 32 + (rtopside + rhalfheight) / (height * 0.5 - 3 * EPSILON)
        elif rtopside + rhalfheight > height / 2 + 3 * EPSILON:
            # # print("trop bas")
            position = 31 + (height * 0.5 - 3 * EPSILON) / (rtopside + rhalfheight)
        else:
            # # print("==> verticale OK")
            if y_11 > rtopside + EPSILON and y_11 < rtopside + rhalfheight:
                # # print("penché trop à gauche")
                position = 22 + (rtopside + EPSILON) / y_11
            elif y_11 < rbottomside - EPSILON and y_11 >= rbottomside - rhalfheight:
                # # print("penché trop à droite")
                position = 21 + y_11 / (rtopside + EPSILON)
            else:
                # # print("===> orientation OK")
                if rect.width < Wanted_Distance - 2 * EPSILON:
                    # # print("trop loin")
                    position = 12 + rect.width / (Wanted_Distance - 2 * EPSILON)
                elif rect.width > Wanted_Distance + 2 * EPSILON:
                    # # print("trop près")
                    position = 11 + (Wanted_Distance - 2 * EPSILON) / rect.width
                else:
                    # # print("====> distance OK")
                    # # print(" ******ALL OK******")
                    position = 0
    return position

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        # print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


# Les fonctions suivantes permettent de contrôle les mouvements du ROV
# Pitch, Yaw, Roll, Throttle, Forward & Lateral
# La vitesse speed_pct est un pourcentage positif ou négatif [-100 ; 100]

def move_pitch(speed_pct):
    # La commande PWM va de 1100us à 1900us
    # Quand speed_pct vaut 0%, aucune action ne doit être envoyée
    set_rc_channel_pwm(1, 1500+4*speed_pct)

def move_roll(speed_pct):
    set_rc_channel_pwm(2, 1500+4*speed_pct)

def move_throttle(speed_pct):
    set_rc_channel_pwm(3, 1500+4*speed_pct)

def move_yaw(speed_pct):
    set_rc_channel_pwm(4, 1500+4*speed_pct)

def move_forward(speed_pct):
    set_rc_channel_pwm(5, 1500+4*speed_pct)

def move_lateral(speed_pct):
    set_rc_channel_pwm(6, 1500+4*speed_pct)

### ENVOI DE COMMANDES

# Armage du robot (activation)
master.arducopter_arm()

# Envoi d'une commande
# La commande doit être envoyée fréquemment, d'où la boucle while

while 1:
    # Wait for the next frame
    while not video.frame_available():
        continue
    frame = video.frame()
    print("frame recorded")
    for barcode in decode(frame):

        ####### commande robot

        if (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 34):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 34)
            print("aller à droite : ", pct)
            move_lateral(10*pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 33):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 33)
            print("aller à gauche : ", pct)
            move_lateral(-10*pct)
        # elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 32):
        #     pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 32)
        #     # print("=> horizontale OK")
        #     # print("aller en bas : ", pct)
        #
        # elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 31):
        #     pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 31)
        #     # print("aller en haut : ", pct)
        # ============================================> utilisation de la commande en position ?
        
        # Send a positive x value, negative y, negative z,
        # positive rotation and no button.
        # https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        # Warning: Because of some legacy workaround, z will work between [0-1000]
        # where 0 is full reverse, 500 is no output and 1000 is full throttle.
        # x,y and r will be between [-1000 and 1000].
        
        # master.mav.manual_control_send( master.target_system, 500, -500, 250, 500, 0)
        
        # ============================================> utilisation de la commande en position ?


        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 22):
            pct = - (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 22)
            print("=> horizontale OK")
            print("==> verticale OK")
            print("tourner à gauche/trigo : ", pct)
            move_roll(-10*pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 21):
            pct = - (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 21)
            print("tourner à droite/horaire : ", pct)
            move_roll(10 * pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 12):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 12)
            print("=> horizontale OK")
            print("==> verticale OK")
            print("===> orientation OK")
            print("avancer : ", pct)
            move_forward(10*pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 11):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 11)
            print("reculer : ", pct)
            move_forward(-10 * pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) == 0):
            print("******ALL OK******")
