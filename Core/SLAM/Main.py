#!/usr/bin/env python

'''

Main SLAM Implementation

'''

#from Draw import Draw as Draw
#from EKF import RobotEKF as ekf
from PID import pidcontrol as PID
from Robot import robot
from Motors import odometry as odom
from Landmarks import find_landmark as Landmarks
from Camera import PiVideoStream
from SocketClient import SocketClient
from Measure import pixelCalibrate
import Vision as Vision
import Motors as Motors
import numpy as np
import time
import sys
import cv2

def contain_pi(theta):
	'''
	Little function to contain an angle between -pi and pi
	'''

	#WRAP BETWEEN -pi AND pi
	theta = theta % (2 * np.pi)    # force in range [0, 2 pi)
	if theta > np.pi:             # move to [-pi, pi)
		theta -= 2 * np.pi
	return theta

def find_landmark(robot):

	#scan each color pattern in the current frame to try and find a landmark
	for i in range(5):
		img = robot.stream.read()

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
		landmark_bearing, landmark_cx, landmark_cy, landmark_area, landmark_marker = get_landmark.position(i)

		detectGreen.drawMultipleFeatures(green_blobs)
		detectRed.drawMultipleFeatures(red_blobs)
		detectBlue.drawMultipleFeatures(blue_blobs)
		cv2.putText(img, 'Landmark: {}, {}'.format(landmark_cx, landmark_cy), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2,cv2.LINE_AA)

		cv2.imshow('image', img)

		if not (landmark_bearing == 0):
			landmark_range = robot.measure.distance_to_camera(landmark_marker[1][0])
			return np.array([[landmark_range, landmark_bearing, i]]).T

		return False

def drive_relative(x, y, robot):
	'''
	Drive relative to the robot's coordinate frame
	inputs: x, y
	'''

	#have we reached our goal?
	if not (((x - 0.05) <= robot.x[0] <= (x + 0.05)) and ((y - 0.05) <= robot.x[1] <= (y + 0.05))):
		#print('[SLAMBOT] Goal not reached, moving robot')
		robot.odom.set_initial()
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
		return True
		Motors.driveMotors(0,0)

	robot.send()
	time.sleep(robot.dt)
	current_pose, delta_d = robot.odom.update(robot.x[2])
	robot.update_pose(current_pose)

    XJacobianR = np.matrix([[1, 0, (-delta_d*math.sin(current_pose[2]))],
                          [0, 1, (delta_d*math.cos(current_pose[2]))],
                          [0, 0, 1]])

    UJacobianR = np.matrix([[(math.cos(current_pose[2])), 0],
                          [(math.sin(current_pose[2])), 0],
                          [0, 1]])

	n = len(robot.landmarks)

	top_xjac  = np.concatenate((XJacobianR, np.zeros((3,(2 * n)))), axis=1)
	bottom_xjac = np.concatenate((np.zeros(((2 * n), 3)), np.identity(2 * n)), axis=1)

	robot.xjac = np.concatenate((top_xjac,bottom_xjac))
	robot.ujac = np.concatenate((UJacobianR,np.zeros(((2 * n), 2))))

	return False

def landmark_init(robot):

	sensor = find_landmark(robot)

	if (not(sensor == False)) and (sensor[2] not in robot.landmarks):
		#expanded the state vector
		robot.u[(3 + robot.landmarks[sensor[2]] * 2)] = robot.x[0] + sensor[0] * np.cos(robot.x[2] + sensor[1])
		robot.u[(4 + robot.landmarks[sensor[2]] * 2)] = robot.x[1] + sensor[0] * np.sin(robot.x[2] + sensor[1])
		robot.landmarks.append(sensor[2]) #add the landmark to known landmark matrix
		robot.zjac = np.array([[np.cos(robot.x[2] + sensor[1]), -sensor[0] * (np.sin(robot.x[2] + sensor[1]))],
							   [np.sin(robot.x[2] + sensor[1]), sensor[0] * (np.cos(robot.x[2] + sensor[1]))]])
		landmark_sigma = dot(robot.zjac,robot.Q).dot(robot.zjac.T)
		zsigma_zeros = np.zeros((2,len(robot.sigma)))
		#expanded the covariance
		robot.sigma = np.concatenate(((np.concatenate((robot.sigma,zsigma_zeros),axis=0)),(np.concatenate((zsigma_zeros.T,landmark_sigma),axis=0))),axis=1)

def run_localization(robot):

	path = drive_relative(0.9,0.9, robot) #make a move
	if path: shutdown(robot) #check when path is finished
	robot.ekf_predict() #run the prediction step
	landmark_init(robot) #check for any new landmarks
	robot.ekf_update()


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
	robot.sigma = np.identity(3)
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
