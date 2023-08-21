# -*- coding: utf-8 -*-
"""
Created on Tue Apr 13 14:32:17 2021

@author: nathan.leroy
"""

import sys 

#utilisation de PyQt5 --> bibli python pour les interfaces graphiques 

from PyQt5.QtWidgets import( QWidget, QLabel, QVBoxLayout, QApplication)
                             


from PyQt5.QtCore import Qt





class DataWidget(QWidget):
    #manière de définir une classe
    def __init__(self):
        super().__init__()

        self.initUI()
    
    #corps de la classe
    def initUI(self):

        #self.xx définie un sous widget
        self.depth = QLabel('<strong> Depth </strong> <br> 42 m', self)#étiquette pour l'affichage avec html
        self.depth.setAlignment(Qt.AlignCenter)#mettre au milieu 
        
        self.hour = QLabel('<strong> Hour </strong> <br>  09 : 15 ',self)
        self.hour.setAlignment(Qt.AlignCenter)
        
        self.date = QLabel('<strong> Date </strong> <br>  29/05/199 ',self)
        self.date.setAlignment(Qt.AlignCenter)
        
        self.temp = QLabel('<strong> Temperature </strong> <br>  37 °C',self)
        self.temp.setAlignment(Qt.AlignCenter)
        
        self.gps = QLabel('<strong> GPS </strong> <br> Lattitude <br> --.-- <br> Longitude <br> --.--',self)
        self.gps.setAlignment(Qt.AlignCenter)
        
        self.battery = QLabel('<strong> Battery </strong> <br> 100%',self)
        self.battery.setAlignment(Qt.AlignCenter)
        
        layout = QVBoxLayout()
        layout.addWidget(self.depth)
        layout.addWidget(self.hour)
        layout.addWidget(self.date)
        layout.addWidget(self.temp)
        layout.addWidget(self.gps)
        layout.addWidget(self.battery)
        
        self.setLayout(layout)
        self.show()
        
        
def main():
    app = QApplication(sys.argv)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()