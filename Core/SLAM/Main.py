#!/usr/bin/env python

'''

Main SLAM Implementation

'''

#from Draw import Draw as Draw
#from EKF import RobotEKF as ekf
from PID import pidcontrol as PID
from Motors import odometry as odom
from Landmarks import find_landmark as Landmarks
from Camera import PiVideoStream
from SocketClient import SocketClient
from Measure import pixelCalibrate
from filterpy.kalman import ExtendedKalmanFilter as EKF
import Vision as Vision
import Motors as Motors
import numpy as np
import time
import sys
import cv2

class robot(object):

	def __init__(self, std_vel, std_steer, dt):
		EKF.__init__(self, 3, 2, 2)
		self.std_vel = std_vel
		self.std_steer = std_steer
		self.max_angular = std_vel - std_steer
		self.wheelbase = 0.15
		self.odom = odom(self.wheelbase)
		self.PID = PID(40,0.0,0.0) #P, I, D
		self.client = SocketClient()
		self.dt = dt
		self.landmarks = []
		self.R = np.diag([0.1, 0.1])
		self.Q = np.diag([0.1, 0.1])
		self.current_path = 0
		self.stream = PiVideoStream() #start the video stream on a seperate thread
		self.measure = pixelCalibrate(1200,90) #calibrate the camera for distances

	def update_pose(self, current_pose):
		self.x = self.x + current_pose

	def ekf_predict(self, landmarks):

		n = len(self.landmarks)
		#PREDICT ROBOT POSE
		self.u[0] = self.x[0]
        self.u[1] = self.x[1]
        self.u[2] = self.x[2]

        #PREDICT LANDMARK POSITION
		if n > 0:
			for i in range(len(self.landmarks)):
				self.u[(3 + self.landmarks[i] * 2)] = landmarks[self.landmarks[i]][0]
				self.u[(4 + self.landmarks[i] * 2)] = landmarks[self.landmarks[i]][1]

		#COVARIANCE
		#self.sigma = dot(self.xjac, self.sigma).dot(self.xjac.T) + dot(self.ujac, self.R).dot(self.ujac.T)
		#self.sigma = np.matrix([[self.sigma],[np.zeros()]])

	def send(self):

		message = np.array([[self.u]])
		self.client.send(message)



def contain_pi(theta):
	'''
	Little function to contain an angle between -pi and pi
	'''

	#WRAP BETWEEN -pi AND pi
	#theta = theta - np.pi
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
	# 
    # XJacobianR = np.matrix([[1, 0, (-delta_d*math.sin(current_pose[2]))],
    #                       [0, 1, (delta_d*math.cos(current_pose[2]))],
    #                       [0, 0, 1]])
	#
    # UJacobianR = np.matrix([[(math.cos(current_pose[2])), 0],
    #                       [(math.sin(current_pose[2])), 0],
    #                       [0, 1]])
	#
	# n = len(robot.landmarks)
	#
	# robot.xjac = np.matrix([[XJacobianR, np.zeros((3,(2 * n)))],
	# 					  [np.zeros(((2 * n), 3)), np.identity(2 * n)]])
	#
	# robot.ujac = np.matrix([[UJacobianR],[np.zeros(((2 * n), 2))]])

	return False

def drive_path(path, robot):
	current_path = drive_relative(path[robot.current_path][0],path[robot.current_path][1],robot)
	print(robot.current_path)
	if current_path:
		if robot.current_path < (len(path) - 1):
			robot.current_path = robot.current_path + 1
		else:
			shutdown(robot)

def run_localization(robot):

	path = drive_relative(0.9,0.9, robot) #make a move
	landmarks = np.zeros((5,2)) #initialize the landmarks array
	robot.ekf_predict(landmarks)
	if path: shutdown(robot) #check when path is finished
	sensor = find_landmark(robot) #find any landmarks
	if not(sensor == False):
		if sensor[2] not in robot.landmarks:
			l_x = robot.x[0] + sensor[0] * np.cos(robot.x[2] + sensor[1])
			l_y = robot.x[1] + sensor[0] * np.sin(robot.x[2] + sensor[1])
			landmarks[sensor[2]][0] = l_x
			landmarks[sensor[2]][1] = l_y
			robot.landmarks.append(sensor[2])
			robot.ekf_predict(landmarks)





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

	print('[SLAMBOT] Initialization complete, starting')

	while True:

		try:
			run_localization(robot)

			#END OF PROGRAM HANDLERS
			k = cv2.waitKey(1)
			if k%256 == 27:
				shutdown(robot)

		except KeyboardInterrupt:
			shutdown(robot)
