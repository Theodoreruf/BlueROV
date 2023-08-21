#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from pyzbar.pyzbar import decode
from gpiozero import LED
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

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


# Create the video object
# Add port= if is necessary to use a different one

# detection QRCode
video = Video(port=4777)
width = 640
height = 480
#cap.set(3,width)
#cap.set(4,height)
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

while True:
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
    #success, img = cap.read()
    for barcode in decode(frame):

        ####### commande robot

        if (getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance)>=34):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 34)
            print("aller à droite : ", pct)
            lRight.on()
        elif(getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance)>=33):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 33)
            print("aller à gauche : ", pct)
            lLeft.on()
        elif(getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 32):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 32)
            print("=> horizontale OK")
            print("aller en bas : ", pct)
            descend.on()
        elif(getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) >= 31):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 31)
            print("aller en haut : ", pct)
            ascend.on()
        elif(getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance)>=22):
            pct = - (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 22)
            print("=> horizontale OK")
            print("==> verticale OK")
            print("tourner à gauche/trigo : ", pct)
            rLeft.on()
        elif(getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance)>=21):
            pct = - (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 21)
            print("tourner à droite/horaire : ", pct)
            rRight.on()
        elif(getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance)>=12):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 12)
            print("=> horizontale OK")
            print("==> verticale OK")
            print("===> orientation OK")
            print("avancer : ", pct)
            forward.on()
        elif(getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance)>=11):
            pct = (1 - getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance) + 11)
            print("reculer : ", pct)
            reverse.on()
        elif(getpositionQRcode(barcode, EPSILON, width, height, Wanted_Distance)==0):
            print("******ALL OK******")

    #cv2.imshow('Result', img)
    #cv2.waitKey(1)
