#!/usr/bin/env python

import roslib; roslib.load_manifest('rit_aero')
import rospy
import PIDController
import serial
import time

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay
from ardrone_autonomy.msg import Navdata

# Finally the GUI libraries
from PySide import QtCore, QtGui

########################
# BEGIN USER VARIABLES #
########################

## Center X coordinate for centering the drone using yaw
CENTER_PIXEL = 480
## Deadzone for the center X coordinate
CENTER_DEADBAND = 50
## Distance in meters for how far to stay from the target
DISTANCE = 5
## Deadzone in meters for how far to stay from the target
DISTANCE_DEADBAND = 1
## Pitch amount for following to target
FOLLOW_SPEED = 0.1

## PID values for yaw
yaw_Kp = 0.4
yaw_Ki = 0.6
yaw_Kd = 0.3

## Distance correction factor for larger targets
DISTANCE_CORRECTION_FACTOR = 6.223

########################
#  END USER VARIABLES  #
########################

## Program variables

ser = None

done_total = False

# Whether it is within range of the target or not
reached_goal_target = False

# Whether it sees a target on the front camera or not
found_front_tag = False
found_bottom_tag = False

# Current type of PID adjustment
pidAdjustType = ''

tags_xc = 0
tags_yc = 0
tags_width = 0
tags_height = 0
tags_distance = 0
tags_count = 0;

STATE_CENTERING = False
STATE_FLYING = False
STATE_LAST_SPINNING = 1

controller = None
pidController = PIDController.PID(yaw_Kp, yaw_Ki, yaw_Kd)
followPIDController = PIDController.PID(3, 0.5, 4)

class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0
		
		
	def keyPressEvent(self, event):
		global STATE_CENTERING, STATE_FLYING, pidController, pidAdjustType,reached_goal_target,ser
		key = event.key()
		
		if(key == 84): # T key for takeoff
			print "Takeoff"
			reached_goal_target = False
			controller.SendFlatTrim()
			controller.SendTakeoff()
			controller.SetCommand(hover=True)
			STATE_FLYING = True
		elif(key == 76): # L key for landing
			print "Land"
			controller.SendLand()
			STATE_FLYING = False
		elif(key == 68): # D to connect to bluetooth
			ser = serial.Serial('/dev/rfcomm0', 57600, timeout=10)
			print "Connecting Bluetooth"

		elif(key == 69): # E key to shut off everything
			print "Emergency"
			controller.SendEmergency()
			STATE_FLYING = False
		elif(key == 67): # C key Toggle centering
			STATE_CENTERING = not STATE_CENTERING
			print "Centering: " + str(STATE_CENTERING)
		elif(key == 80): # P key toggle PID
			pidAdjustType = 'P'
			print "Toggle P adjustment"
			pass
		elif(key == 73): # I key toggle PID
			pidAdjustType = 'I'
			print "Toggle I adjustment"
			pass
		elif(key == 68): # D key toggle PID
			pidAdjustType = 'D'
			print "Toggle D adjustment"
			pass
		elif(key == 61): # + key increase adjustment
			if(pidAdjustType == 'P'):
				pidController.setKp( pidController.Kp + 0.1 )
			elif(pidAdjustType == 'I'):
				pidController.setKi( pidController.Ki + 0.1 )
			elif(pidAdjustType == 'D'):
				pidController.setKd( pidController.Kd + 0.1 )
			print "P:" + str(pidController.Kp) + ", I:" + str(pidController.Ki) + ", D: " + str(pidController.Kd) 
		elif(key == 45): # - key decrease adjustment
			if(pidAdjustType == 'P'):
				pidController.setKp( pidController.Kp - 0.1 )
			elif(pidAdjustType == 'I'):
				pidController.setKi( pidController.Ki - 0.1 )
			elif(pidAdjustType == 'D'):
				pidController.setKd( pidController.Kd - 0.1 )
			print "P:" + str(pidController.Kp) + ", I:" + str(pidController.Ki) + ", D: " + str(pidController.Kd) 
		else:
			print "Unknown command: " + str(key)
		
def parseData(data):
	global tags_xc, tags_yc, tags_width, tags_height, tags_distance, tags_count, found_front_tag, found_bottom_tag
	tags_count = data.tags_count
	found_front_tag = False
	found_bottom_tag = False
	
	for i in range(int(tags_count)):
		if(data.tags_type[i] == 0):
			found_front_tag = True
			tags_xc = float(data.tags_xc[i])
			tags_yc = float(data.tags_yc[i])
			tags_width = float(data.tags_width[i])
			tags_height = float(data.tags_height[i])
			tags_distance = data.tags_distance[i]
			tags_count = int(data.tags_count)

			tags_distance = tags_distance / 100
			tags_distance = DISTANCE_CORRECTION_FACTOR * float(tags_distance)
		if(data.tags_type[i] == 131072):
			found_bottom_tag = True
			

def dropObject():
	global ser, done_total
	if not done_total:
		print "Starting Drop"
		ser.write('2')
		time.sleep(10)
		ser.write('2')
		done_total = True
		print "Dropping Object"
	pass
def centerOnTagPID():
	global reached_goal_target
	if reached_goal_target:
		dropObject()
		return
	if not found_bottom_tag:
		if STATE_CENTERING:
			yaw_command = 0
			follow_command = 0
			if found_front_tag and not reached_goal_target:
				if(tags_xc > (CENTER_PIXEL + CENTER_DEADBAND) or tags_xc < (CENTER_PIXEL - CENTER_DEADBAND)):
					yaw_command = pidController.update(tags_xc) / 2500
				
				if(tags_distance > (DISTANCE + DISTANCE_DEADBAND) or tags_distance < (DISTANCE - DISTANCE_DEADBAND)):
					follow_command = -1 * (followPIDController.update(tags_distance) / 2100)
				else:
					reached_goal_target = True
					print "Reached target"
					controller.SetCommand(hover=True)	
			else:
				if not reached_goal_target:
					if STATE_LAST_SPINNING == 1:
						yaw_command = -0.1
					else:
						yaw_command = 0.1
					
			if(follow_command == 0 and yaw_command == 0):
				controller.SetCommand(hover=True)
			else:
				controller.SetCommand(pitch=follow_command, yaw_velocity=yaw_command)
		else:
			controller.SetCommand(hover=True)	
	else:
		reached_goal_target = True
		controller.SetCommand(hover=True)	
		

def ReceiveData(data):
	parseData(data)
	centerOnTagPID()
				
		
# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('rit_aero_auto')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()
	
	STATE_FLYING = False
	STATE_LAST_SPINNING = 0
	pidController.setPoint(CENTER_PIXEL)
	followPIDController.setPoint(DISTANCE)
	# Init navdata
	sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)
	
	display.show()
	
	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
