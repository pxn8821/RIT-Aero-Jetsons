#!/usr/bin/env python

import roslib; roslib.load_manifest('rit_aero')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay
from ardrone_autonomy.msg import Navdata

# Finally the GUI libraries
from PySide import QtCore, QtGui

class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0
		
		
	def keyPressEvent(self, event):
		key = event.key()
		print key
		controller.SendTakeoff()

def ReceiveData(data):
	print data.tags_type
# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('rit_aero_auto')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()
	
	# Init navdata
	sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)
	
	display.show()

	# executes the QT application
	status = app.exec_()


	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
