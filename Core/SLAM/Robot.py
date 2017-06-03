#!/usr/bin/env python

'''

MAIN ROBOT CLASS

'''

from Motors import odometry as odom
from SocketClient import SocketClient
from PID import pidcontrol as PID
from Camera import PiVideoStream
from Measure import pixelCalibrate
from numpy import dot
import numpy as np
import math

class robot(object):

	def __init__(self, std_vel, std_steer, dt):
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
		self.I = np.identity(3) #inital robot_x , robot_y, robot_theta state vector?
		self.current_path = 0
		self.debug = False
		self.update = 0
		self.state = 0
		self.stream = PiVideoStream() #start the video stream on a seperate thread
		self.measure = pixelCalibrate(1200,90) #calibrate the camera for distances

	def update_pose(self, current_pose):
		self.x = self.x + current_pose
		self.x[2] = self.contain_pi(self.x[2]) #contain robot theta between pi and - pi


	def ekf_predict(self):

		#PREDICT ROBOT POSE
		self.u[0] = self.x[0]
		self.u[1] = self.x[1]
		self.u[2] = self.x[2]

		#COVARIANCE
		self.sigma = dot(self.xjac, self.sigma).dot(self.xjac.T) + dot(self.ujac, self.R).dot(self.ujac.T)

	def ekf_update(self, landmark_id, sensor):

		#UPDATE STEP
		#We probably want to ditch the kalman filter library
		#and do the update step here

		H = self.H_of(landmark_id, sensor)

		S = (dot(H, self.sigma).dot(H.T)) + self.Q
		K = dot(self.sigma, H.T).dot(np.linalg.inv(S))
		#Here we can see the H.T needs to be expanded with the landmarks as self.sigma
		#is expanded with the landmarks also so to retain symmetry for dot product we must match the
		#dimensions

		hx =  self.Hx(landmark_id)
		y = self.residual(sensor, hx)
		self.u = self.u + dot(K, y)

		#I believe the I Matrix should be a 2x2 identity as the H Jacobian is only 2 rows
		#This could be wrong. It may need to be the size of number of state variables which is
		#dependent on number of landmarks currently detected
		I_KH = self.I - dot(K, H)
		self.sigma = dot(I_KH, self.sigma)


	def send(self):

		self.client.send(self.u) #send the state vector array
		#self.client.send(self.sigma) #send the covariance array

	def H_of(self, landmark_id, sensor):

		px = self.u[(3 + (landmark_id * 2))]
		py = self.u[(4 + (landmark_id * 2))]
		hyp = sensor[0]**2
		dist = sensor[0]

		#Expand with landmarks?
		H = np.array([[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
					  [(py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])

		self.hjac = False
		return H

	def Hx(self, landmark_id):

		px = self.u[(3 + (landmark_id * 2))]
		py = self.u[(4 + (landmark_id * 2))]
		robot_x = self.u[0]
		robot_y = self.u[1]
		robot_theta = self.u[2]
		Hx = np.array([[(np.sqrt((robot_x - px)**2 + (robot_y - py)**2))],
					[(math.atan2(py - robot_y, px - robot_x)) - robot_theta]])

		#WRAP BETWEEN -pi AND pi
		Hx[1] = Hx[1] % (2 * np.pi)    # force in range [0, 2 pi)
		if Hx[1] > np.pi:             # move to [-pi, pi)
			Hx[1] -= 2 * np.pi

		return Hx

	def residual(self, a, b):
		""" compute residual (a-b) between measurements containing
		[range, bearing]. Bearing is normalized to [-pi, pi)"""
		y = a - b
		#WRAP BETWEEN -pi AND pi
		y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
		if y[1] > np.pi:             # move to [-pi, pi)
			y[1] -= 2 * np.pi
		return y

	def contain_pi(self, theta):
		'''
		Little function to contain an angle between -pi and pi
		'''

		#WRAP BETWEEN -pi AND pi
		theta = theta % (2 * np.pi)    # force in range [0, 2 pi)
		if theta > np.pi:             # move to [-pi, pi)
			theta -= 2 * np.pi
		return theta
