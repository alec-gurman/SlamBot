'''

MAIN ROBOT DRIVING SCRIPT


UPDATED 20 April 2:30pm

'''

import time
import os
import sys
import cv2
import numpy as np
import motor_controller as motors
import colorBlobDetector as blob
import pid_controller as pid
import camera_setup as cam
import distance_calibrate as distcal
import Landmarks as lf
from camera_setup import PiVideoStream

#initialize the robot

motors.init() #setup the motors
vs = PiVideoStream().start() #start the video stream on a seperate thread
time.sleep(2.0) #allow camera to warm up
pc = distcal.pixelCalibrate(1200,90) #calibrate the camera for distances

#GLOBALS

mainP = 0.5
mainI = 0
mainD = 0

robot_speed = 40 #max 100
disable = False #disable motors for debugging
debug = True #show contour features?
debug_images = True #enable imshow?

travel_distance = 0
robot_y = 240
robot_x = 160

#Create the PID object

pid_angle = pid.pidcontrol(mainP,mainI,mainD)

#start the state selector

print("\n")
print("Which state do you want to run?")
print("current states: 1: stop yellow, 2: avoid red")
print("\n")
state = int(input())

if state == 3:
	print("\n")
	print("Which landmark are you tracking?")
	print("\n")
	landmark_goal = int(input())
	print("Tracking goal: {}".format(landmark_goal))

print("Running state: {}".format(state))

state_machine = 1

def containAngle360(angle):
	if(angle < 1):
		angle = angle + 270
	elif(angle >= 361):
		angle = angle - 270
	return angle

while True:

	if state == 1:
		if state_machine == 1:
			img = vs.read()
			#img = cv2.imread('saved_images/opencv_image_3.png')

			detectYellow = blob.get_blob('yellow', img)
			y_cent_x, y_cent_y, y_heading_angle, y_marker, y_area = detectYellow.getFeatures(160,240)

			detectRed = blob.get_blob('red', img)
			r_cent_x, r_cent_y, r_heading_angle, r_marker, r_area = detectRed.getFeatures(160,240)

			if (y_area > 0) and (r_area > 500):
				state = 2
			elif (y_area > 0) and (r_area < 500):
				state_machine = 2
			elif (y_area < 0):
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(40,-40)

			if debug == True:
				detectYellow.drawFeatures()
				detectRed.drawFeatures()


			if debug_images == True:
				cv2.imshow('image', img)

		if state_machine == 2:

			img = vs.read()
			#img = cv2.imread('saved_images/opencv_image_3.png')

			detectYellow = blob.get_blob('yellow', img)
			y_cent_x, y_cent_y, y_heading_angle, y_marker, y_area = detectYellow.getFeatures(160,240)

			detectRed = blob.get_blob('red', img)
			r_cent_x, r_cent_y, r_heading_angle, r_marker, r_area = detectRed.getFeatures(160,240)


			if (r_area > 500): #if in tracking yellow mode and a red obstacle appears, deal with it
				state_machine = 1

			pid_return = (pid_angle.update(y_heading_angle))
			pid_wheel = int(robot_speed - abs(pid_return))

			if debug == True:
				detectYellow.drawFeatures()
				detectRed.drawFeatures()
				cv2.putText(img, 'PID OUT: {}'.format(pid_return), (50,460), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
				cv2.putText(img, 'PID WHEELS: {}'.format(pid_wheel), (50,430), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)

			if y_heading_angle <= 180:
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(pid_wheel, robot_speed)
			if y_heading_angle > 180:
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(robot_speed,pid_wheel)
			if y_heading_angle == 0:
				state_machine = 1

			if (robot_y - y_cent_y) < 70: #if we get close enough to the yellow goal
				in_mm = pc.distance_to_camera(y_marker[1][0])
				print(in_mm)
				if debug == True:
					cv2.putText(img, 'DISTANCE: {}'.format(in_mm), (50,400), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
				motors.driveMotors(0,0)
				time.sleep(0.5)
				#check if the remaining distance to object is more than 20cm
				if in_mm > 200:
					travel_distance = (in_mm - 200)
				else:
					travel_distance = 1
				state_machine = 3
				vs.stop()

			if debug_images == True:
				cv2.imshow('image', img)

		if state_machine == 3:
			initial_ticksA = motors.get_ticksA()
			initial_ticksB = motors.get_ticksB()
			initial_position = 0.0
			state_machine = 4

		if state_machine == 4:
			ticksA = (motors.get_ticksA()) - initial_ticksA #attempt to reset stored ticks
			ticksB = (motors.get_ticksB()) - initial_ticksB #attempt to reset stored ticks
			distances = motors.get_distance(ticksA,ticksB)
			positionA = distances.return_distanceA()
			#print("current pos: {}".format(positionA))
			motors.driveMotors(30,robot_speed)
			if (positionA - initial_position) <= -travel_distance:
				motors.driveMotors(0,0)
				time.sleep(0.1)
				sys.exit()

		k = cv2.waitKey(1)

		if k%256 == 27:
			#ESC PRESSED
			motors.driveMotors(0,0)
			vs.stop()
			print("------------------------------")
			print("Escape hit, closing .....")
			print("------------------------------")
			print("\n")
			sys.exit()

	if state == 2:

		img = vs.read()

		detectYellow = blob.get_blob('yellow', img)
		y_cent_x, y_cent_y, y_heading_angle, y_marker, y_area = detectYellow.getFeatures(160,240)

		detectRed = blob.get_blob('red', img)
		r_cent_x, r_cent_y, r_heading_angle, r_marker, r_area = detectRed.getFeatures(160,240)

		if state_machine == 1:

			pid_track_red = (pid_angle.update(r_heading_angle))
			track_red_wheels = int(robot_speed - abs(pid_track_red))

			if debug == True:

				detectRed.drawFeatures()
				#detectYellow.drawFeatures()
				cv2.putText(img, 'PID OUT: {}'.format(pid_track_red), (50,460), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
				cv2.putText(img, 'PID WHEELS: {}'.format(track_red_wheels), (50,430), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)

			if r_heading_angle <= 180:
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(track_red_wheels, robot_speed)
			if r_heading_angle > 180:
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(robot_speed,track_red_wheels)
			if r_heading_angle == 0:
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(40,-40)

			if (robot_y - r_cent_y) < 50:
				motors.driveMotors(0,0)
				state_machine = 2

		if state_machine == 2: #turn until cant see red
			motors.driveMotors(robot_speed,-robot_speed) #turn
			if r_area < 500:
				motors.driveMotors(0,0)
				state_machine = 3

		if state_machine == 3:
			motors.driveMotors((robot_speed - 10),robot_speed)
			time.sleep(2) #drive straight for 2 seconds
			state_machine = 4

		if state_machine == 4:
			if (r_area < 500) and (y_area < 200):
				motors.driveMotors(-robot_speed, robot_speed)
			elif (r_area > 500) and (y_area > 200):
				state_machine = 2
			elif (r_area < 500) and (y_area > 200):
				state = 1
				state_machine = 1 #restore the state machine variable
			elif (r_area > 500) and (y_area < 200):
				state_machine = 2

		if debug_images == True:
			cv2.imshow('image', img)

		k = cv2.waitKey(1)

		if k%256 == 27:
			#ESC PRESSED
			motors.driveMotors(0,0)
			vs.stop()
			print("\n")
			print("------------------------------")
			print("Escape hit, closing .....")
			print("------------------------------")
			print("\n")
			sys.exit()

	if state == 3:

		#marker tracking code goes here

		img = vs.read()
		#img = cv2.imread('../tools/saved_images/opencv_image_10.png')

		detectRed = blob.get_blob('red', img)
		red_blobs = []
		red_blobs = detectRed.getMultipleFeatures(160,240)

		detectGreen = blob.get_blob('green', img)
		green_blobs = []
		green_blobs = detectGreen.getMultipleFeatures(160,240)

		detectBlue = blob.get_blob('blue', img)
		blue_blobs = []
		blue_blobs = detectBlue.getMultipleFeatures(160,240)

		get_landmark = lf.find_landmark(red_blobs,green_blobs,blue_blobs) #initialize the landmarker finder class with our three blobs

		landmark_cx, landmark_cy, landmark_area, landmark_marker = get_landmark.position(landmark_goal)

		if debug == True:

			detectGreen.drawMultipleFeatures(green_blobs)
			detectRed.drawMultipleFeatures(red_blobs)
			detectBlue.drawMultipleFeatures(blue_blobs)
			cv2.putText(img, 'Landmark: {}, {}'.format(landmark_cx, landmark_cy), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2,cv2.LINE_AA)
			#cv2.putText(img, 'Landmark 2: {}, {}'.format(landmark_cx, landmark_cy), (50,80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2,cv2.LINE_AA)
			#cv2.putText(img, 'Landmark 3: {}, {}'.format(landmark_cx, landmark_cy), (50,110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2,cv2.LINE_AA)

		previous_found = False

		if state_machine == 1:

			if landmark_cx > 0:
				state_machine = 2
				previous_found = True
				#landmark_heading = landmark1_heading
			else:
				if previous_found == True:
					motors.driveMotors(-30,30)
					previous_found = False
				else:
					if disable == True:
						motors.driveMotors(0,0)
					else:
						motors.driveMotors(30,-30)

		if state_machine == 2:

			goal_rad = np.arctan2(landmark_cy - robot_y, landmark_cx - robot_x)
			landmark_heading = containAngle360(np.degrees(goal_rad))

			pid_return = (pid_angle.update(landmark_heading))
			print("heading: {}".format(landmark_heading))
			pid_wheel = int(robot_speed - abs(pid_return))
			print("pid_wheels: {}".format(pid_wheel))

			if landmark_heading <= 180:
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(pid_wheel, robot_speed)
			if landmark_heading > 180:
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(robot_speed,pid_wheel)

			if landmark_area > 1500: #if we get close enough to the yellow goal
				print("close enough to goal")
				in_mm = pc.distance_to_camera(landmark_marker[1][0])
				print(in_mm)
				if debug == True:
					cv2.putText(img, 'DISTANCE: {}'.format(in_mm), (50,400), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
				motors.driveMotors(0,0)
				time.sleep(0.5)
				if (in_mm / 2) > 200:
					travel_distance = ((in_mm / 2) - 200)
				else:
					travel_distance = 1
				state_machine = 3
				vs.stop()
			else:
				state_machine = 1

			#cv2.imshow('image', img)

		if state_machine == 3:
			print("in state machine 3")
			initial_ticksA = motors.get_ticksA()
			initial_ticksB = motors.get_ticksB()
			initial_position = 0.0
			state_machine = 4

		if state_machine == 4:
			ticksA = (motors.get_ticksA()) - initial_ticksA #attempt to reset stored ticks
			ticksB = (motors.get_ticksB()) - initial_ticksB #attempt to reset stored ticks
			distances = motors.get_distance(ticksA,ticksB)
			positionA = distances.return_distanceA()
			#print("current pos: {}".format(positionA))
			motors.driveMotors(30,robot_speed)
			if (positionA - initial_position) <= -travel_distance:
				motors.driveMotors(0,0)
				time.sleep(0.1)
				sys.exit()

		if debug_images == True:
			cv2.imshow('image', img)

		k = cv2.waitKey(1)

		if k%256 == 27:
			#ESC PRESSED
			motors.driveMotors(0,0)
			vs.stop()
			print("\n")
			print("------------------------------")
			print("Escape hit, closing .....")
			print("------------------------------")
			print("\n")
			sys.exit()

cv2.destroyAllWindows()
vs.stop()
