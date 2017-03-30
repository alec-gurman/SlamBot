'''

MAIN ROBOT DRIVING SCRIPT

'''

import time
import os
import sys
import cv2
import motor_controller as motors
import colorBlobDetector as blob
import pid_controller as pid
import camera_setup as cam
import distance_calibrate as distcal
from camera_setup import PiVideoStream

motors.init() #setup the motors
vs = PiVideoStream().start()
time.sleep(2.0) #allow camera to warm up
pc = distcal.pixelCalibrate(1200,90)

#GLOBALS

mainP = 0.5
mainI = 0
mainD = 0

pid_angle = pid.pidcontrol(mainP,mainI,mainD)

robot_speed = 40 #max 100
disable = False #disable motors for debugging 	
debug = True
show_fps = True
travel_distance = 0
frames = 0
initial_t = time.time()

print("\n")
print("Which state do you want to run?")
print("current states: 1: stop yellow, 2: avoid red")
print("\n")
state = int(input())
print("Running state: {}".format(state))

state_machine = 1

while True:
	
	if show_fps == True:	
		frames += 1
		current_t = time.time()
		if (current_t - initial_t) > 1:
			fps = frames
			print("fps: {}".format(fps))
			initial_t = time.time()
			frames = 0
				
	if state == 1:
		if state_machine == 1:
			img = vs.read()
			#img = cv2.imread('saved_images/opencv_image_3.png')
			
			detectYellow = blob.get_blob('yellow', img)
			cent_x, cent_y, heading_angle, marker, area = detectYellow.getFeatures(160,240)
			
			pid_return = (pid_angle.update(heading_angle))
			pid_wheel = int(robot_speed - abs(pid_return))
			
			if debug == True:
				detectYellow.drawFeatures()
				cv2.putText(img, 'PID OUT: {}'.format(pid_return), (50,460), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
				cv2.putText(img, 'PID WHEELS: {}'.format(pid_wheel), (50,430), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
			
			if heading_angle <= 180:
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(pid_wheel, robot_speed)
			if heading_angle > 180:	
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(robot_speed,pid_wheel)
			if heading_angle == 0:
				if disable == True:
					motors.driveMotors(0,0)
				else:
					motors.driveMotors(40,-40)
				
			cv2.line(img,(320,480),(320,380),(0,0,255),2)
			cv2.imshow('image', img)
			
			is_close = 240 - cent_y
			if is_close < 50:
				in_mm = pc.distance_to_camera(marker[1][0])
				if debug == True:
					cv2.putText(img, 'DISTANCE: {}'.format(in_mm), (50,400), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
				motors.driveMotors(0,0)
				time.sleep(0.5)
				travel_distance = in_mm
				state_machine = 2
				vs.stop()
				
		if state_machine == 2:
			initial_ticksA = motors.get_ticksA()
			initial_ticksB = motors.get_ticksB()
			initial_position = 0.0
			state_machine = 3
		
		if state_machine == 3:
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
	
		print(r_area)
		
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
					
			is_close = 240 - r_cent_y
			if is_close < 50:
				motors.driveMotors(0,0)
				#time.sleep(1)
				state_machine = 2

		if state_machine == 2: #turn until cant see red
			motors.driveMotors(robot_speed,-robot_speed) #turn
			if r_area < 100:
				motors.driveMotors(0,0)
				state_machine = 3
					
		if state_machine == 3:
			motors.driveMotors(30,robot_speed)
			time.sleep(2)
			state_machine = 4

		if state_machine == 4:
			if (r_area < 500) and (y_area < 500):
				motors.driveMotors(-robot_speed, robot_speed)
			elif (r_area > 500) and (y_area > 500):
				state_machine = 2
			elif (r_area < 500) and (y_area > 500):
				state = 1
				state_machine = 1
			elif (r_area > 500) and (y_area < 500):
				state_machine = 2
			
		cv2.imshow('image', img)
			
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
	
cv2.destroyAllWindows()
vs.stop()
