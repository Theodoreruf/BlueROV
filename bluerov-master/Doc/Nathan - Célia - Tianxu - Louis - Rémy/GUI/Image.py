# -*- coding: utf-8 -*-
"""
Created on Tue Apr 13 14:32:17 2021

@author: nathan.leroy
"""

import sys 

#utilisation de PyQt5 --> bibli python pour les interfaces graphiques 

from PyQt5.QtWidgets import( QWidget, QLabel)
from PyQt5.QtGui import QPixmap                             


from PyQt5.QtCore import Qt



class ImageWidget(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
     
        self.label = QLabel(self)
        pixmap = QPixmap('underwater')
        self.label.setPixmap(pixmap)
        self.label.setStyleSheet("border: 1px solid black; background: white")
    
        

        
        
def main():
    app = QApplication(sys.argv)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
