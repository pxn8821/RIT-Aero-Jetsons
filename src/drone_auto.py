#!/usr/bin/env python

import roslib; roslib.load_manifest('rit_aero')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay
from ardrone_autonomy.msg import Navdata

# Finally the GUI libraries
from PySide import QtCore, QtGui

CENTER_PIXEL = 480
CENTER_DEADBAND = 50

tags_xc = 0
tags_yc = 0
tags_width = 0
tags_height = 0
tags_distance = 0
tags_count = 0;

STATE_CENTERING = False
STATE_LAST_SPINNING = 1

controller = None

class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0
		
		
	def keyPressEvent(self, event):
		global STATE_CENTERING
		key = event.key()
		
		if(key == 84):
			print "Takeoff"
			controller.SendTakeoff()
		elif(key == 76):
			print "Land"
			controller.SendLand()
			pass
		elif(key == 69):
			print "Emergency"
			controller.SendEmergency()
		elif(key == 67):
			print "Centering"
			STATE_CENTERING = not STATE_CENTERING
					
		print key	
		

def ReceiveData(data):
	global STATE_LAST_SPINNING
	tags_count = 0
	if(data.tags_count == 1):
		tags_xc = float(data.tags_xc[0])
		tags_yc = float(data.tags_yc[0])
		tags_width = float(data.tags_width[0])
		tags_height = float(data.tags_height[0])
		tags_distance = data.tags_distance[0]
		tags_count = int(data.tags_count)
	
	
	if tags_count == 1:
		tags_distance = tags_distance / 100
		tags_distance = 6.223 * float(tags_distance)
		#print "x:" + str(tags_xc) + ", y:" + str(tags_yc) + ", width:" + str(tags_width) + ", height:" + str(tags_height) + ", distance:" + str(tags_distance)

	if STATE_CENTERING:
		if tags_count == 1:
			if(tags_xc > (CENTER_PIXEL + CENTER_DEADBAND)):
				controller.SetCommand(yaw_velocity=-0.1)
				STATE_LAST_SPINNING = 1
				print "Right"
			elif(tags_xc < (CENTER_PIXEL - CENTER_DEADBAND)):
				STATE_LAST_SPINNING = 0
				print "Left"
				controller.SetCommand(yaw_velocity=0.1)
			else:
				controller.SetCommand(yaw_velocity=0)
				print "Centered"
		else:
			if STATE_LAST_SPINNING == 1:
				controller.SetCommand(yaw_velocity=-0.1)
			else:
				controller.SetCommand(yaw_velocity=0.1)
				
		
# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('rit_aero_auto')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()
	
	STATE_LAST_SPINNING = 0
	
	# Init navdata
	sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)
	
	display.show()
	
	

	# executes the QT application
	status = app.exec_()


	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
