#!/usr/bin/env python

'''

Main SLAM Implementation

'''

#from Draw import Draw as Draw
#from EKF import RobotEKF as ekf
from Robot import robot
from Landmarks import find_landmark as Landmarks
from numpy import dot
import Vision as Vision
import Motors as Motors
import numpy as np
import math
import time
import sys
import cv2

def find_landmark(robot, ID):

	#scan each color pattern in the current frame to try and find a landmark
	img = robot.stream.read()
	#img_path = '../tools/calibration/opencv_image_0.png'
	#img = cv2.imread(img_path)

	detectRed = Vision.get_blob('red', img)
	red_blobs = []
	red_blobs = detectRed.getMultipleFeatures(160,240)

	detectGreen = Vision.get_blob('green', img)
	green_blobs = []
	green_blobs = detectGreen.getMultipleFeatures(160,240)

	detectBlue = Vision.get_blob('blue', img)
	blue_blobs = []
	blue_blobs = detectBlue.getMultipleFeatures(160,240)

	get_landmark = Landmarks(red_blobs,green_blobs,blue_blobs) #initialize the landmarker finder class with our three blobs
	landmark_bearing, landmark_cx, landmark_cy, landmark_area, landmark_marker = get_landmark.position(ID)

	if(robot.debug):

		detectGreen.drawMultipleFeatures(green_blobs)
		detectRed.drawMultipleFeatures(red_blobs)
		detectBlue.drawMultipleFeatures(blue_blobs)
		cv2.putText(img, 'Landmark: {}, {}'.format(landmark_cx, landmark_cy), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2,cv2.LINE_AA)

		cv2.imshow('image', img)

	if not (landmark_bearing == 0):
		landmark_range = robot.measure.distance_to_camera(landmark_marker[1][0]) / 100
		print('[SLAMBOT][DEBUG] LANDMARK %s: %s, %s' % (ID, landmark_range, landmark_bearing))
		return np.array([[landmark_range, landmark_bearing, ID]]).T

	return np.array([[0.0, 0.0, -1]]).T

def update_motion_jacobians(current_pose, delta_d):

	XJacobianR = np.matrix([[1, 0, (-delta_d*math.sin(current_pose[2]))],
						  [0, 1, (delta_d*math.cos(current_pose[2]))],
						  [0, 0, 1]])

	UJacobianR = np.matrix([[(math.cos(current_pose[2])), 0],
						  [(math.sin(current_pose[2])), 0],
						  [0, 1]])

	n = len(robot.landmarks)

	top_xjac  = np.concatenate((XJacobianR, np.zeros((3,(2 * n)))), axis=1)
	bottom_xjac = np.concatenate((np.zeros(((2 * n), 3)), np.identity(2 * n)), axis=1)

	#Update the motion model jacobians
	robot.xjac = np.concatenate((top_xjac,bottom_xjac))
	robot.ujac = np.concatenate((UJacobianR,np.zeros(((2 * n), 2))))

def update_motion_model():

	robot.send() #send the robot data before the update

	current_pose, delta_d = robot.odom.update(robot.x[2]) #update the odometry data
	robot.update_pose(current_pose) #update the robot's pose
	update_motion_jacobians(current_pose, delta_d) #update the motion jacobians

def drive_relative(x, y, robot):
	'''
	Drive relative to the robot's coordinate frame
	inputs: x, y
	'''
	finished = False
	#have we reached our goal?
	if not (((x - 0.05) <= robot.x[0] <= (x + 0.05)) and ((y - 0.05) <= robot.x[1] <= (y + 0.05))):
		#We shouldn't need to contain this angle as it is simply for a driving system and PID
		heading = (np.arctan2(y - robot.x[1], x - robot.x[0])) - robot.x[2] #in radians
		# the output of our pid represents a signed number depending on the direction we are turning
		pid_return = abs(robot.PID.update(heading)) #abs so that our motors dont travel in reverse direction
		#limit to robots maximum angular velocity
		if pid_return > robot.std_steer:
			pid_return = robot.std_steer
		pid_wheel = int(robot.std_vel - pid_return) #integer as our motor function only accepts an integer
		#basic drive loop
		if heading > 0.0:
			#pass
			Motors.driveMotors(pid_wheel, robot.std_vel)
		if heading <= 0.0:
			#pass
			Motors.driveMotors(robot.std_vel,pid_wheel)
	else:
		Motors.driveMotors(0,0)
		return True

	return False

def landmark_init(robot, sensor):

	if ((not(sensor[0] == 0.0 and sensor[1] == 0.0)) and (sensor[2] not in robot.landmarks)):
		#expanded the state vector
		robot.landmarks.append(sensor[2]) #add the landmark to known landmark matrix
		robot.u[int(3 + (sensor[2] * 2))] = robot.u[0] + (sensor[0] * np.cos(robot.x[2] + sensor[1]))
		robot.u[int(4 + (sensor[2] * 2))] = robot.u[1] + (sensor[0] * np.sin(robot.x[2] + sensor[1]))
		robot.zjac = np.array([[float(np.cos(robot.x[2] + sensor[1])), float(-sensor[0] * (np.sin(robot.x[2] + sensor[1])))],
							   [float(np.sin(robot.x[2] + sensor[1])), float(sensor[0] * (np.cos(robot.x[2] + sensor[1])))]])
		landmark_sigma = dot(robot.zjac,robot.Q).dot(robot.zjac.T)
		zsigma_zeros = np.zeros((2,len(robot.sigma)))
		#expanded the covariance
		robot.sigma = np.concatenate(((np.concatenate((robot.sigma,zsigma_zeros),axis=0)),(np.concatenate((zsigma_zeros.T,landmark_sigma),axis=0))),axis=1)

def run_localization(robot):

	robot.odom.set_initial() #set initial odom

	#robot.state = 20
	if robot.state == 0:
		#DRIVE TO CENTER OF MAP
		path = drive_relative(0.5,0.5, robot)
		if path:
			robot.state = 1
			Motors.driveMotors(0,0)
		#	robot.stored_theta = float(robot.u[2])

		time.sleep(robot.dt)
		update_motion_model()
		robot.ekf_predict() #run the prediction step


	if robot.state == 1:

		for i in range(5):
			sensor = find_landmark(robot, i)
			if (len(robot.landmarks) > 0) and sensor[0] > 0:
				robot.ekf_update(sensor) #call the ekf_update for each landmark
			landmark_init(robot, sensor) #check for any new landmarks
			if sensor[2] == 0:
				robot.state = 2

		Motors.driveMotors(40,0)
		time.sleep(robot.dt)
		Motors.driveMotors(0,0)

		update_motion_model()
		robot.ekf_predict() #run the prediction step

	if robot.state == 2:

		for i in range(5):
			sensor = find_landmark(robot, i)
			landmark_init(robot, sensor) #check for any new landmarks
			if (len(robot.landmarks) > 0) and sensor[0] > 0:
				robot.ekf_update(sensor) #call the ekf_update for each landmark
			if sensor[2] == 4:
				robot.state = 3

		Motors.driveMotors(0,40)
		time.sleep(robot.dt)
		Motors.driveMotors(0,0)

		update_motion_model()
		robot.ekf_predict() #run the prediction step

	if robot.state == 3:

		for i in range(5):
			sensor = find_landmark(robot, i)
			landmark_init(robot, sensor) #check for any new landmarks
			if (len(robot.landmarks) > 0) and sensor[0] > 0:
				robot.ekf_update(sensor) #call the ekf_update for each landmark
			if sensor[2] == 4:
				robot.state = 3

		if robot.u[2] > 0.8:
			Motors.driveMotors(40,0)
		elif robot.u[2] < 0.6:
			Motors.driveMotors(0,40)
		else:
			Motors.driveMotors(0,0)
			robot.state = 4

		update_motion_model()
		robot.ekf_predict() #run the prediction step

	if robot.state == 4:

		path = drive_relative(0.8,0.8, robot)
		if path:
			robot.state = 5
			Motors.driveMotors(0,0)
		#	robot.stored_theta = float(robot.u[2])

		for i in range(5):
			sensor = find_landmark(robot, i)
			landmark_init(robot, sensor) #check for any new landmarks
			if (len(robot.landmarks) > 0) and sensor[0] > 0:
				robot.ekf_update(sensor) #call the ekf_update for each landmark
			if sensor[2] == 4:
				robot.state = 5

		time.sleep(robot.dt)
		update_motion_model()
		robot.ekf_predict() #run the prediction step

	if robot.state == 5:

		for i in range(5):
			sensor = find_landmark(robot, i)
			if (len(robot.landmarks) > 0) and sensor[0] > 0:
				robot.ekf_update(sensor) #call the ekf_update for each landmark
			landmark_init(robot, sensor) #check for any new landmarks
			if sensor[2] == 0:
				robot.state = 6

		Motors.driveMotors(40,0)
		time.sleep(robot.dt)
		Motors.driveMotors(0,0)

		update_motion_model()
		robot.ekf_predict() #run the prediction step

	if robot.state == 6:

		for i in range(5):
			sensor = find_landmark(robot, i)
			landmark_init(robot, sensor) #check for any new landmarks
			if (len(robot.landmarks) > 0) and sensor[0] > 0:
				robot.ekf_update(sensor) #call the ekf_update for each landmark
			if sensor[2] == 4:
				robot.state = 5

		Motors.driveMotors(0,40)
		time.sleep(robot.dt)
		Motors.driveMotors(0,0)

		update_motion_model()
		robot.ekf_predict() #run the prediction step




	if robot.state == 100:
		Motors.driveMotors(0,0);
		time.sleep(robot.dt)
		robot.send()

	if robot.state == 20:
		Motors.driveMotors(-30,30)
		if robot.u[2] > 1.57:
			Motors.driveMotors(0,0)
			shutdown(robot)

	#ANOTHER METHOD, SCAN ONE LANDMARK PER LOOP!!!!????

def shutdown(robot):
	robot.stream.stop()
	print('[SLAMBOT] Shutting down...')
	robot.client.sock.close()
	Motors.driveMotors(0,0)
	sys.exit()


if __name__ == "__main__":

	print('[SLAMBOT] Starting main program')
	print('[SLAMBOT] Warming up the camera')

	robot = robot(std_vel=40, std_steer=30, dt=0.25) #speed units are in a scaled from 0 to 100
	robot.x = np.zeros((3,1)) #robot_x, robot_y, robot_theta ROBOT INITALS
	robot.u = np.zeros((13,1)) #robot_x, robot_y, robot_theta ROBOT INITALS
	robot.xjac = np.zeros((3,3))
	robot.ujac = np.zeros((3,2))
	robot.sigma = np.diag((0.0005,0.0005,0.0005))
	robot.client.connect() #Start the python socket
	robot.stream.start() #Start the camera
	time.sleep(2.0) #allow camera to warm up

	print('[SLAMBOT] initializing the motors')
	Motors.init() #Init the motors

	print('[SLAMBOT] Initialization complete, starting...')

	while True:

		try:
			run_localization(robot)

			#END OF PROGRAM HANDLERS
			k = cv2.waitKey(1)
			if k%256 == 27:
				shutdown(robot)

		except KeyboardInterrupt:
			shutdown(robot)
