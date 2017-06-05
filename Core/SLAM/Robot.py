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
import time

class robot(object):

	def __init__(self, std_vel, std_steer, dt):
		self.std_vel = std_vel
		self.std_steer = std_steer
		self.max_angular = std_vel - std_steer
		self.wheelbase = 0.135
		self.odom = odom(self.wheelbase)
		self.PID = PID(45,0.0,0.0) #P, I, D
		self.client = SocketClient()
		self.dt = dt
		self.landmarks = []
		self.R = np.diag([0.00025, 0.00025])
		self.Q = np.diag([0.01, 0.01])
		self.I = np.identity(3) #inital robot_x , robot_y, robot_theta state vector?
		self.current_path = 0
		self.debug = False
		self.update = 0
		self.state = 0
		self.stored_theta = 0
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

	def ekf_update(self, sensor):

		#UPDATE STEP

		landmark_id = sensor[2]

		H = self.H_of(landmark_id, sensor)

		S = (dot(H, self.sigma).dot(H.T)) + self.Q
		K = dot(self.sigma, H.T).dot(np.linalg.inv(S))

		hx =  self.Hx(landmark_id)
		sensor_only = sensor[0:2,:]
		y = self.residual(sensor_only, hx)
		ky_dot = np.zeros((13,1))
		ky_dot[0:((2*len(self.landmarks))+3),:] = dot(K, y)
		self.u = self.u + ky_dot

		self.I = np.identity((3 + (2*len(self.landmarks)))) #the identity expands with N amount of landmarks init
		I_KH = self.I - dot(K, H)
		self.sigma = dot(I_KH, self.sigma)


	def send(self):
		try:
			msg = np.empty(2,dtype=object)
			if len(self.sigma) >= 3:
				robot_sigma = self.sigma[0:2,0:2]
				reduced_sigma = robot_sigma
			if len(self.sigma) >= 5:
				L0_sigma = self.sigma[3:5,3:5]
				reduced_sigma = np.concatenate((robot_sigma,L0_sigma),axis=0)
			if len(self.sigma) >= 7:
				L1_sigma = self.sigma[5:7,5:7]
				reduced_sigma = np.concatenate((robot_sigma,L0_sigma,L1_sigma),axis=0)
			if len(self.sigma) >= 9:
				L2_sigma = self.sigma[7:9,7:9]
				reduced_sigma = np.concatenate((robot_sigma,L0_sigma,L1_sigma,L2_sigma),axis=0)
			if len(self.sigma) >= 11:
				L3_sigma = self.sigma[9:11,9:11]
				reduced_sigma = np.concatenate((robot_sigma,L0_sigma,L1_sigma,L2_sigma,L3_sigma),axis=0)
			if len(self.sigma) >= 13:
				L4_sigma = self.sigma[11:13,11:13]
				reduced_sigma = np.concatenate((robot_sigma,L0_sigma,L1_sigma,L2_sigma,L3_sigma,L4_sigma),axis=0)
			msg[:] = [self.u, reduced_sigma]
			self.client.send(msg)
			#another method is number vtsack
		except Exception as e:
			print(e)


	def H_of(self, landmark_id, sensor):
		px = float(self.u[int(3 + (landmark_id * 2))])
		py = float(self.u[int(4 + (landmark_id * 2))])
		hyp = float(sensor[0]**2)
		dist = float(sensor[0])

		#Expand with landmarks
		n = len(self.landmarks)
		robot_H = np.array([[-(px - float(self.u[0])) / dist, -(py - float(self.u[1])) / dist, 0],
					  [(py - float(self.u[1])) / hyp,  -(px - float(self.u[0])) / hyp, -1]])
		landmark_H = np.array([[-(px - float(self.u[0])) / dist, -(py - float(self.u[1])) / dist],
					  [(py - float(self.u[1])) / hyp,  -(px - float(self.u[0])) / hyp]])
		zeros_fill = np.zeros((2,int(2 * n)))
		H = np.concatenate((robot_H,zeros_fill), axis=1)
		landmark_index = self.landmarks.index(landmark_id)
		s = (2*landmark_index + 3)
		H[0:2, s:(s+2)] = landmark_H

		return H

	def Hx(self, landmark_id):

		px = float(self.u[int(3 + (landmark_id * 2))])
		py = float(self.u[int(4 + (landmark_id * 2))])
		robot_x = float(self.u[0])
		robot_y = float(self.u[1])
		robot_theta = float(self.u[2])
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
