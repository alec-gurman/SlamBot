#!usr/bin/python3

'''

MOTOR CONTROLLER CLASS

'''

import math
import penguinPi as ppi
import time
import numpy as np

mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)

def init():
	ppi.init()
	driveMotors(0,0)

def driveMotors(speedA, speedB):

	mA.set_power(-speedA)
	mB.set_power(speedB)

def get_ticksA():
	return mA.get_ticks()

def get_ticksB():
	return mB.get_ticks()

class odometry(object):

	def __init__(self, wheelbase):
		self.ticksA = 0.0
		self.ticksB = 0.0
		self.delta_x = 0.0
		self.delta_y = 0.0
		self.delta_theta = 0.0
		self.wheelbase = wheelbase

	def set_initial(self):
		self.ticksA = get_ticksA() #attempt to reset stored ticks
		self.ticksB = get_ticksB() #attempt to reset stored ticks


	def update(self,theta):

		deltaA = -(((get_ticksA() - self.ticksA) / 360) * (math.pi * 0.065))
		deltaB = -(((get_ticksB() - self.ticksB) / 90) * (math.pi * 0.065))
		delta_dist = ((deltaA + deltaB) / 2)
		self.delta_x = float(delta_dist * np.cos(theta))
		self.delta_y = float(delta_dist * np.sin(theta))
		self.delta_theta = (deltaB - deltaA) / self.wheelbase
		#print(self.delta_theta)
		#contain delta theta between pi and -pi
		# I RECENTLY UNCOMMENTED THIS AS WE MAY NEED DELTA THETA IN THIS RANGE
		self.delta_theta = self.delta_theta % (2 * np.pi)
		if self.delta_theta > np.pi:
		   self.delta_theta -= 2 * np.pi
		delta_pose = np.array([[self.delta_x, self.delta_y, self.delta_theta]]).T
		return delta_pose, delta_dist
