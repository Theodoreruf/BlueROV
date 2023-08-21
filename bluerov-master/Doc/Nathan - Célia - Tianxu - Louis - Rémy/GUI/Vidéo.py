# -*- coding: utf-8 -*-
"""
Created on Tue Apr 13 14:32:17 2021

@author: nathan.leroy
"""

import sys 

#utilisation de PyQt5 --> bibli python pour les interfaces graphiques 

from PyQt5.QtWidgets import (QMainWindow, QAction, QMenu, QPushButton, QGridLayout,
                             QLabel, QVBoxLayout, QHBoxLayout, QWidget,QTextEdit, QLineEdit,
                             QApplication)

from PyQt5.QtGui import QIcon, QImage, QPixmap
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot


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



        
def main():
    app = QApplication(sys.argv)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
