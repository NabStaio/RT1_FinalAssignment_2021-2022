#! /usr/bin/env python

from std_srvs.srv import *

import math
import rospy
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import *

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

########################################################################################################################################
# APP(QWidget):												                         #
# Custom class created for a simple GUI that works as a user's manual to well interface with the terminator where the commands         #
# will be inserted.                                                                                                                    #
# METHODS---> __init__(): it's simply the initialization of the UI                                                                     #
#             initUI(): set the geometry and the text of the window                                                                    #
########################################################################################################################################

class App(QWidget):
    def __init__(self):
        super().__init__()
        self.title = 'USER INTERFACE'
        self.left = 50
        self.top = 50
        self.width = 600
        self.height = 500
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        
        self.label_title = QLabel(self)
        self.label_title.move(5, 0)
        self.label_title.setText("DRIVER SIMULATOR")
        self.label_title.setStyleSheet('color: red')
        self.label_title.setFont(QFont('Times', 20))
        
        
        self.label_subtitle = QLabel(self)
        self.label_subtitle.move(5, 30)
        self.label_subtitle.setText("Choose the modality to drive the robot by typing\nthe corresponding number on your keyboard")
        self.label_subtitle.setFont(QFont('Georgia', 10))
        
        self.label_waiting = QLabel(self)
        self.label_waiting.move(5, 100)
        self.label_waiting.setText("PRESS <0> WAITING MODE")
        self.label_waiting.setStyleSheet('color: goldenrod')
        self.label_waiting.setFont(QFont('Comic Sans MS', 11))
        
        self.label_autonomous = QLabel(self)
        self.label_autonomous.move(5, 130)
        self.label_autonomous.setText("PRESS <1> AUTONOMOUS DRIVE\nIn this modality you can delete the goal by taping <0>")
        self.label_autonomous.setStyleSheet('color: blue')
        self.label_autonomous.setFont(QFont('Comic Sans MS', 11))
        
        self.label_teleop = QLabel(self)
        self.label_teleop.move(5, 170)
        self.label_teleop.setText("PRESS <2> KEYBOARD DRIVE\nIn this modality to guide the robot with the keyboard use the other terminal")
        self.label_teleop.setStyleSheet('color: lime')
        self.label_teleop.setFont(QFont('Comic Sans MS', 11))
        
        self.label_mix = QLabel(self)
        self.label_mix.move(5, 210)
        self.label_mix.setText("PRESS <3> ASSISTANCE DRIVE\nSame of the previous modality")
        self.label_mix.setStyleSheet('color: black')
        self.label_mix.setFont(QFont('Comic Sans MS', 11))
        
        self.label_switch = QLabel(self)
        self.label_switch.move(5, 280)
        self.label_switch.setText("To switch between modalities you must return to WAITING MODE first,\nso press <0> before changing and tap another mode")
        self.label_switch.setFont(QFont('Georgia', 10))
        
        self.label_close = QLabel(self)
        self.label_close.move(5, 320)
        self.label_close.setText("To close the UI first type the cross to close the instrunctions' window\nand then CRTL+C to stop the application")
        self.label_close.setFont(QFont('Georgia', 10))
        
        self.show()

#################################################################################################
# MAIN():                                                                                       #
# Main function that will pop up the window with the instructions through App().                #
# Input commands are retrieved to choose the modality the user want to use.                     #
# <0> is the WAITING MODE, used for switching between modes                                     #
# <1> is the AUTONOMOUS MODE in which the robot drives autonomously                             #
# <2> is the KEYBOARD MODE where the use through the keyboard will guide the robot              #
# <3> is the ASSISTANCE MODE that includes obstacle avoidance with teleoperation by user        #
#################################################################################################

def main():
	app = QApplication(sys.argv)
	ex = App()
	
	flag = 0
	while not rospy.is_shutdown():
	 
		command = int(input('CHOOSE MODALITY: \n')) #Take the input from keyboard
		
		# WAITING MODALITY
		if command == 0:
			
			# First modality cancel message.
			if flag == 1:
				print("GOAL DELETED")
				flag=0
				
			rospy.set_param('active', 0)# Setting the parameter to 0.
			print("WAITING")# Printing the actual state.
				
		# AUTONOMOUS DRIVE
		elif command == 1:
		
			# First modality cancel message
			if flag == 1:
				print("GOAL DELETED")
				flag=0
				
			rospy.set_param('active', 0)		# Reset of the modality
			
			print("YOU'RE IN AUTONOMOUS DRIVE, PRESS <0> TO CANCEL THE TARGET.")
			print("SELECT DESTINATION")
			des_x_input = float(input("INSERT X COORDINATE: "))
			des_y_input = float(input("INSERT Y COORDINATE: "))
			rospy.set_param('des_pos_x', des_x_input)
			rospy.set_param('des_pos_y', des_y_input)
			rospy.set_param('active', 1)
			print("AUTONOMOUS DRIVE MODALITY IS ACTIVE.")
			flag = 1
			
		# KEYBOARD DRIVE
		elif command == 2:
		
			# First modality cancel message
			if flag == 1:
				print("GOAL DELETED")
				flag=0
						
			rospy.set_param('active', 2)
			print(msg)
			print("KEYBOARD DRIVE MODALITY IS ACTIVE.")
			
			
				
		# ASSISTANCE DRIVE
		elif command == 3:
		
			# First modality cancel message
			if flag == 1:
				print("GOAL DELETED")
				flag=0
				
			rospy.set_param('active', 3)
			print(msg)
			print("ASSISTANCE DRIVE IS ACTIVE.")
			
				
				
		else:
			print("WRONG KEY, TRY AGAIN.")


if __name__ == "__main__":
	main()


