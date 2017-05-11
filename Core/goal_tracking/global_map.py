#!/usr/bin/env python

'''

GLOBAL MAPPING SCRIPT WEEK 8

BASIC YELLOW FINDING

'''

import time
import os
import sys
import cv2
import math
import numpy as np
import motor_controller as motors
import colorBlobDetector as blob
import pid_controller as pid
import camera_setup as cam
import distance_calibrate as distcal
import landmarkFinder
from camera_setup import PiVideoStream
import matplotlib.pyplot as plt

#initialize the robot

motors.init() #setup the motors
vs = PiVideoStream().start() #start the video stream on a seperate thread
time.sleep(2.0) #allow camera to warm up
pc = distcal.pixelCalibrate(1200,90) #calibrate the camera for distances

#GLOBALS

mainP = 0.5
mainI = 0
mainD = 0

FREQUENCY = 0.1 #10 Hz
robot_speed = 40 #max 100
ROBOT_V = 40 * 0.025 #convert our arbitary units to a simulation of m/s lel...
distance_travelled = ROBOT_V * FREQUENCY
delta_theta = 0.0 #initialize the change in theta
ROBOT_W = 0.0 #initialize angular velocity which will be updated by the pid
disable = False #disable motors for debugging
debug = True #show contour features?
debug_images = True #enable imshow?
travel_distance = 0
robot_y = 240
robot_x = 160
ROBOT_GRAPH_X = 0.0
ROBOT_GRAPH_Y = 0.0
ROBOT_GRAPH_HEADING = 0.0
state_machine = 1

#Create the PID object

pid_angle = pid.pidcontrol(mainP,mainI,mainD)

x_axis = []
y_axis = []
heading_axis = []

while True:

	if state_machine == 1:
		img = vs.read()
		#img = cv2.imread('saved_images/opencv_image_3.png')

		detectYellow = blob.get_blob('yellow', img)
		y_cent_x, y_cent_y, y_heading_angle, y_marker, y_area = detectYellow.getFeatures(160,240)

		if (y_area > 0):
			state_machine = 2
		elif (y_area < 0):
			if disable == True:
				motors.driveMotors(0,0)
			else:
				motors.driveMotors(40,-40)

		if debug == True:
			detectYellow.drawFeatures()
			#detectRed.drawFeatures()

		if debug_images == True:
			cv2.imshow('image', img)

	if state_machine == 2:

		img = vs.read()
		#img = cv2.imread('saved_images/opencv_image_3.png')

		detectYellow = blob.get_blob('yellow', img)
		y_cent_x, y_cent_y, y_heading_angle, y_marker, y_area = detectYellow.getFeatures(160,240)

		pid_return = (pid_angle.update(y_heading_angle))
		pid_wheel = int(robot_speed - abs(pid_return))

		ROBOT_W = pid_wheel*0.1
		delta_theta = ROBOT_W * FREQUENCY

		if debug == True:
			detectYellow.drawFeatures()
			#detectRed.drawFeatures()
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

		# robot_error = y_heading_angle - ROBOT__GRAPH_HEADING

		# if robot_error > 0.0:
		ROBOT_GRAPH_HEADING = ROBOT_GRAPH_HEADING + delta_theta
		# if robot_error <= 0.0:
		#     ROBOT_HEADING = ROBOT_HEADING

		ROBOT_GRAPH_X = ROBOT_GRAPH_X + distance_travelled * math.cos(math.radians(ROBOT_GRAPH_HEADING))
		ROBOT_GRAPH_Y = ROBOT_GRAPH_Y + distance_travelled * math.sin(math.radians(ROBOT_GRAPH_HEADING))

		x_axis.append(ROBOT_GRAPH_X)
		y_axis.append(ROBOT_GRAPH_Y)
		heading_axis.append(ROBOT_GRAPH_HEADING)

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
			for s,i in enumerate(x_axis):
				plt.plot(i, y_axis[s], marker=(3, 0, heading_axis[s]), markersize=10, linestyle='None')


			#plt.plot([x_axis],[y_axis],'r>')
			plt.axis([0, 3, 0, 4])
			plt.show()


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

	time.sleep(FREQUENCY)
