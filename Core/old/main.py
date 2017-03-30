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

motors.init() #setup the motors
camera, rawCapture = cam.init() #setup the camera
time.sleep(0.1) #allow camera to warm up
pc = distcal.pixelCalibrate(1150,90)

#GLOBALS

mainP = 0.5
mainI = 0
mainD = 0

pid_angle = pid.pidcontrol(mainP,mainI,mainD)

robot_speed = 40 #max 100
disable = True #disable motors for debugging 	
debug = True
travel_distance = 0
state_machine = 1

frames = 0

#choose a state

print("\n")
print("Which state do you want to run?")
print("current states: 1: stop yellow, 2: avoid red")
print("\n")
state = int(input())
print("Running state: {}".format(state))
stage = 1

while stage == 1:
	
	'''
	frames = frames + 1
	current_t = time.time()
	if (current_t - initial_t) > 1:
		initial_t = time.time()
		fps = frames
		print("fps: {}".format(fps))
		frames = 0
	'''
	
	'''
	
	if state == 1:
		for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
			
			img = frame.array
			#img = cv2.imread('saved_images/opencv_image_3.png')
			
			detectYellow = blob.get_blob('yellow', img)
			cent_x, cent_y, heading_angle, marker = detectYellow.getFeatures(320,480)
			
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
			
			is_close = 480 - cent_y
			if is_close < 50:
				in_mm = pc.distance_to_camera(marker[1][0])
				if debug == True:
					cv2.putText(img, 'DISTANCE: {}'.format(in_mm), (50,400), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
				motors.driveMotors(0,0)
				time.sleep(0.5)
				travel_distance = in_mm
				print("stopped and about to move {}".format(travel_distance))
				state = 10
				rawCapture.truncate(0)
				break
			
			k = cv2.waitKey(1)
			
			if k%256 == 27:
				#ESC PRESSED
				motors.driveMotors(0,0)
				rawCapture.truncate(0)
				print("------------------------------")
				print("Escape hit, closing .....")
				print("------------------------------")
				print("\n")
				sys.exit()
				
				
			cv2.imshow('image', img)
				
			rawCapture.truncate(0) #clear steam for next frame
			
	if state == 10:
			
		print("moving to stage 2")
		initial_ticksA = motors.get_ticksA()
		initial_ticksB = motors.get_ticksB()
		initial_position = 0.0
		print("initial pos: {}".format(initial_position))
		state = 11
	 
	if state == 11:
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
			
	'''
			
	if state == 2:
		for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
			img = frame.array
			#img = cv2.imread('saved_images/opencv_image_0.png')
			
			'''
			detectYellow = blob.get_blob('yellow', img)
			y_cent_x, y_cent_y, y_heading_angle, y_marker, y_area = detectYellow.getFeatures(320,480)
			
			detectRed = blob.get_blob('red', img)
			r_cent_x, r_cent_y, r_heading_angle, r_marker, r_area = detectRed.getFeatures(320,480)
			
			
			
			pid_track_red = (pid_angle.update(r_heading_angle))
			track_red_wheels = int(robot_speed - abs(pid_track_red))
		
			if debug == True:
				
				detectRed.drawFeatures()
				#detectYellow.drawFeatures()
				cv2.putText(img, 'PID OUT: {}'.format(pid_track_red), (50,460), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
				cv2.putText(img, 'PID WHEELS: {}'.format(track_red_wheels), (50,430), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
			'''		
			
			'''
			if state_machine == 1:
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
						
				is_close = 480 - r_cent_y
				if is_close < 200:
					motors.driveMotors(0,0)
					time.sleep(1)
					state_machine = 2
					initial_ticksA = motors.get_ticksA()
					initial_ticksB = motors.get_ticksB()
					initial_position = 0.0
					
			#print("area: {}".format(r_area))
					
			if state_machine == 2: #turn until cant see red
				motors.driveMotors(robot_speed,-robot_speed) #turn
				if r_cent_x < 10 and r_cent_y < 10:
					motors.driveMotors(0,0)
					state_machine = 3
					time.sleep(1)
					
			if state_machine == 3:
				motors.driveMotors(robot_speed,robot_speed)
				time.sleep(2)
				state_machine = 4

			if state_machine == 4:
				if (r_cent_x < 10 and r_cent_y < 10) or (y_cent_x < 10 and y_cent_y < 10):
					motors.driveMotors(-robot_speed, robot_speed)
				else:
					motors.driveMotors(0,0)
					sys.exit()
				
			
			
			'''
			
			'''
			
			if state_machine == 2: #turn 90 degrees
				
				print("entering 90 degree turn state")	
				motors.driveMotors(robot_speed,-robot_speed)
				ticksA = (motors.get_ticksA()) - initial_ticksA #attempt to reset stored ticks
				ticksB = (motors.get_ticksB()) - initial_ticksB #attempt to reset stored ticks
				distances = motors.get_distance(ticksA,ticksB)
				positionA = distances.return_distanceA()
				motors.driveMotors(robot_speed,-robot_speed)
				if (positionA - initial_position) <= -112:
					motors.driveMotors(0,0)
					time.sleep(0.1)
					sys.exit()
					
			'''
				
			'''	
				
			k = cv2.waitKey(1)
			
			if k%256 == 27:
				#ESC PRESSED
				motors.driveMotors(0,0)
				rawCapture.truncate(0)
				print("------------------------------")
				print("Escape hit, closing .....")
				print("------------------------------")
				print("\n")
				sys.exit()
				
			'''
				
				
			cv2.imshow('image', img)
				
			rawCapture.truncate(0) #clear steam for next frame
		
	
#cv2.destroyAllWindows()
