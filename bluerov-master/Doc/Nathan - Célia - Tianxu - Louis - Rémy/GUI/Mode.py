# -*- coding: utf-8 -*-
"""
Created on Tue Apr 13 14:32:17 2021

@author: nathan.leroy
"""

import sys 

#utilisation de PyQt5 --> bibli python pour les interfaces graphiques 

from PyQt5.QtWidgets import( QWidget, QPushButton, QHBoxLayout, QApplication)
                             


from PyQt5.QtCore import Qt





class ModeWidget(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setStyleSheet("border: 5px solid black; background: white")

        self.bInit = QPushButton('Initialisation',self)
        self.bInit.setMinimumSize(20,5)
        
        self.bAutom = QPushButton('Auto Mode',self)
        self.bAutom.setMinimumSize(20,5)
        
        self.bGPS = QPushButton('GPS',self)
        self.bGPS.setMinimumSize(20,5)
        
        self.bRetour = QPushButton('Return',self)
        self.bRetour.setMinimumSize(20,5)

        layout = QHBoxLayout()
        layout.addWidget(self.bInit)
        layout.addWidget(self.bAutom)
        layout.addWidget(self.bGPS)
        layout.addWidget(self.bRetour)
     
        
        self.setLayout(layout)
        
        self.show()
        
        
def main():
    app = QApplication(sys.argv)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
