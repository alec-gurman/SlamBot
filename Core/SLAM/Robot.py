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
		self.current_path = 0
		self.debug = False
		self.update = 10
		self.stream = PiVideoStream() #start the video stream on a seperate thread
		self.measure = pixelCalibrate(1200,90) #calibrate the camera for distances

	def update_pose(self, current_pose):
		self.x = self.x + current_pose

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

		hx =  self.Hx(landmark_id)
		y = self.residual(sensor, hx)
		self.u = self.u + dot(K, y)

		I_KH = self._I - dot(K, H)
		self._P = dot(I_KH, P)


	def send(self):

		message = self.u
		self.client.send(message)

	def H_of(self, landmark_id):

		px = self.u[(3 + (landmark_id * 2))]
		py = self.u[(4 + (landmark_id * 2))]
		hyp = sensor[1]**2
		dist = sensor[1]

		H = np.array([[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
					  [(py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
		return H

	def Hx(self, landmark_id):

		px = self.u[(3 + (landmark_id * 2))]
		py = self.u[(4 + (landmark_id * 2))]
		Hx = np.array([[(np.sqrt((x[0, 0] - px)**2 + (x[1, 0] - py)**2))],
					[(math.atan2(py - x[1, 0], px - x[0, 0])) - x[2, 0]]])

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
