# -*- coding: utf-8 -*-
"""
Created on Tue Mar 23 10:10:26 2021

@author: nathan.leroy
"""
############################################ Definition ############################################################


#utilisation de PyQt5 --> bibli python pour les interfaces graphiques 
import sys
import cv2
from PyQt5.QtWidgets import (QMainWindow, QAction, QMenu, QPushButton, QGridLayout,
                             QLabel, QVBoxLayout, QHBoxLayout, QWidget,QTextEdit, QLineEdit,
                             QApplication)

from PyQt5.QtGui import QIcon, QImage, QPixmap
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot


#Appel des widgets utilisés
import Data, Mode, Image

####################################################################################################################




####################################################### Vidéo #################################################################

class Thread(QThread):
    changePixmap = pyqtSignal(QImage)

    def run(self):
        self.cap = cv2.VideoCapture(0)

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.codec = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
        self.writer = cv2.VideoWriter('output.avi', self.codec, 30.0, (self.width, self.height))

        while self.cap.isOpened():
            ret, self.frame = self.cap.read()
            if ret:
                self.frame = cv2.flip(self.frame, 1)
                rgbimage = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbimage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(rgbimage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                p = convertToQtFormat.scaled(1280, 720, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)

class VideoWidget(QWidget):
    def __init__(self):
        super(VideoWidget, self).__init__()
        self.title = 'Camera'
        self.initUI()

    def initUI(self):
        self.label = QLabel(self)
        lay = QVBoxLayout()
        lay.addWidget(self.label)
        self.setLayout(lay)

        self.th = Thread()
        self.th.changePixmap.connect(self.setImage)
        self.th.start()
        self.show()

    @pyqtSlot(QImage)
    def setImage(self, image):
        self.label.setPixmap(QPixmap.fromImage(image))
        self.th.writer.write(image)

####################################################################################################################





##############################################   Fenêtre principale   ###############################################

        
class MainWindows(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()
        

    def initUI(self):
        self.data = Data.DataWidget()
        self.data.setStyleSheet(" background: white;  border-radius: 6px")
        self.video = VideoWidget()
        self.video.setStyleSheet(" background: white; border: 5px solid black; border-radius: 6px")
        self.mode = Mode.ModeWidget()
        self.mode.setStyleSheet(" background: white")
        
    
        layout = QGridLayout()
        layout.addWidget(self.video,0,0,14,14)
        layout.addWidget(self.data,0,15,13,2)
        layout.addWidget(self.mode,15,0,3,17)
        
    
        self.setLayout(layout)
        
        self.setStyleSheet('background-color:grey')
        
        self.setGeometry(300, 300, 1280, 720)
        self.setWindowTitle('BlueRov')
        self.show()


def main():
    app = QApplication(sys.argv)
    ex = MainWindows()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
