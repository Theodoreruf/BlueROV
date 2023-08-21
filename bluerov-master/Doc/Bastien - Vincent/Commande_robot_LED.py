# -*- coding: utf-8 -*-

# Ce programme arme le robot et permet de le commander en simulant
# des commandes joystick RC

# Imports
from pymavlink import mavutil
import time
import numpy as np
from pyzbar.pyzbar import decode
from gpiozero import LED
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

"""
MAVlink class
"""
def wait_conn():
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)

"""
BlueRov video capture class
"""

class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK

def getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance): # renvoie le pourcentage d'exactitude par rapport à position désirée
                                                                         # donc pourcentage d'éloignement = 1 - position
    position = float

    pts = np.array(barcode.polygon, np.int32)  # récupère les coordonnées x y sous la forme d'un vecteur colonne
    pts = pts.reshape((-1, 1, 2))

    rect = barcode.rect

    rleftside = rect.left
    rtopside = rect.top
    # rrightside = rect.left + rect.width
    rbottomside = rect.top + rect.height
    rhalfheight = int(rect.height / 2)
    rhalfwidth = int(rect.width / 2)
    top_left_corner = pts[0]
    # bottom_left_corner = pts[1]
    # bottom_right_corner = pts[2]
    # top_right_corner = pts[3]
    y_11 = top_left_corner[~(top_left_corner == rleftside)].sum()

    if rleftside + rhalfwidth < width / 2 - 3 * EPSILON:
        # print("trop à gauche")
        position = 34 + (rleftside + rhalfwidth)/(width*0.5 - 3 * EPSILON)
    elif rleftside + rhalfwidth > width / 2 + 3 * EPSILON:
        # print("trop à droite")
        position = 33 + (width * 0.5 - 3 * EPSILON)/(rleftside + rhalfwidth)
    else:
        # print("=> horizontale OK")
        if rtopside + rhalfheight < height / 2 - 3 * EPSILON:
            # print("trop haut")
            position = 32 + (rtopside + rhalfheight)/(height*0.5 - 3 * EPSILON)
        elif rtopside + rhalfheight > height / 2 + 3 * EPSILON:
            # print("trop bas")
            position = 31 + (height * 0.5 - 3 * EPSILON)/(rtopside + rhalfheight)
        else:
            # print("==> verticale OK")
            if y_11 > rtopside + EPSILON and y_11 < rtopside + rhalfheight:
                # print("penché trop à gauche")
                position = 22 + (rtopside + EPSILON)/y_11
            elif y_11 < rbottomside - EPSILON and y_11 >= rbottomside - rhalfheight:
                # print("penché trop à droite")
                position = 21 + y_11/(rtopside + EPSILON)
            else:
                # print("===> orientation OK")
                if rect.width < Wanted_Distance - 2 * EPSILON:
                    # print("trop loin")
                    position = 12 + rect.width/(Wanted_Distance - 2 * EPSILON)
                elif rect.width > Wanted_Distance + 2 * EPSILON:
                    # print("trop près")
                    position = 11 + (Wanted_Distance - 2 * EPSILON)/rect.width
                else:
                    # print("====> distance OK")
                    # print(" ******ALL OK******")
                    position = 0
    return position
#======================================================================================

# Variables globales pour détection
# detection QRCode
video = Video(port=4777)
width = 640
height = 480
EPSILON = 6
Wanted_Distance = 200

# activation LEDs
lLeft = LED(2)
lRight = LED(3)
forward = LED(4)
reverse = LED(5)
rLeft = LED(6)
rRight = LED(13)
ascend = LED(19)
descend = LED(26)

# Create the connection
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')
# Wait a connexion
# wait_conn()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Get some information !
while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)

# Function to detect QRcode's position
def getpositionQRcode(barcode, EPSILON, width, height,Wanted_Distance):  # renvoie le pourcentage d'exactitude par rapport à position désirée
                                                                         # donc pourcentage d'éloignement = 1 - position
    position = float

    pts = np.array(barcode.polygon, np.int32)  # récupère les coordonnées x y sous la forme d'un vecteur colonne
    pts = pts.reshape((-1, 1, 2))

    rect = barcode.rect

    rleftside = rect.left
    rtopside = rect.top
    # rrightside = rect.left + rect.width
    rbottomside = rect.top + rect.height
    rhalfheight = int(rect.height / 2)
    rhalfwidth = int(rect.width / 2)
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
    # Switch off testing leds
    lRight.off()
    lLeft.off()
    forward.off()
    reverse.off()
    rLeft.off()
    rRight.off()
    ascend.off()
    descend.off()
    # Wait for the next frame
    while not video.frame_available():
        continue
    frame = video.frame()
    print("frame recorded")
    for barcode in decode(frame):

        ####### commande robot

        if (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 34):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 34)
            lRight.on()
            print("aller à droite : ", pct)
            move_lateral(10*pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 33):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 33)
            lLeft.on()
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
        # ============================================> quelle commande utiliser ici ?

        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 22):
            pct = - (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 22)
            print("=> horizontale OK")
            print("==> verticale OK")
            rLeft.on()
            print("tourner à gauche/trigo : ", pct)
            move_roll(-10*pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 21):
            pct = - (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 21)
            rRight.on()
            print("tourner à droite/horaire : ", pct)
            move_roll(10 * pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 12):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 12)
            print("=> horizontale OK")
            print("==> verticale OK")
            print("===> orientation OK")
            forward.on()
            print("avancer : ", pct)
            move_forward(10*pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 11):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 11)
            reverse.on()
            print("reculer : ", pct)
            move_forward(-10 * pct)
        elif (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) == 0):
            print("******ALL OK******")
