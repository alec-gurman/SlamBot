#!usr/bin/python3

'''

Assessment Sript, Drive foward x distance

PID MODIFIED AS OF 16/03/2017

THIS FILE IS DESIGNED FOR RUNNING WHEELS FOWARD *******

'''

import penguinPi as ppi
import time
from wheel_encoders import get_distance

#PID VARIABLES

mainP = 3200.0
mainI = 0.0
mainD = 0.0

#GLOBALS

speed = 65 #try turn this down for more accuracy in straights?
max_speed = 100

debug = False

#PID CLASS

class PID:

	def __init__(self, P,I,D):
		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.set_point=0.0
		self.error=0.0
		self.prevError= 0.0
		self.intAccum = 0.0

	def update(self, current_value):
		self.error = self.set_point - current_value
		self.intAccum += self.error
		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * (self.error - self.prevError)
		self.I_value = self.Ki * self.intAccum
		self.prevError = self.error

		Output = self.P_value + self.D_value + self.I_value
		return Output

	def setPoint(self, set_point):
		self.set_point = set_point

#Create our device objects
ppi.init()
pid_out = PID(mainP,mainI,mainD)
mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)
display = ppi.Display(ppi.AD_DISPLAY_A)
display.set_mode('u')

#drive motors, input speeds motor A and motor B
def driveMotors(speedA, speedB):
	
	mA.set_power(-speedA) #fowards with wheels facing foward
	mB.set_power(speedB) #fowards with wheels facing foward
		
		
initial_ticksA = mA.get_ticks()
initial_ticksB = mB.get_ticks() 
ticksA = (mA.get_ticks()) - initial_ticksA#attempt to reset stored ticks
ticksB = (mB.get_ticks()) - initial_ticksB #attempt to reset stored ticks
distances = get_distance(ticksA,ticksB)
initial_position = distances.return_distanceA()
print("\n")
print("Enter the Distance you would like to travel: ")
distance = float(input())
print("\n")
print("----------------------------")
print("Travelling %s meters!" % distance)
print("----------------------------")
print("\n")


try:
	while True:
		ticksA = (mA.get_ticks()) - initial_ticksA
		ticksB = (mB.get_ticks()) - initial_ticksB
		distances = get_distance(ticksA,ticksB)
		positionA = abs(distances.return_distanceA()) #disregard direction
		positionB = abs(distances.return_distanceB()) #disregard direction	
		wheel_error = abs((ticksA / 360) - (ticksB / 90))
		pid_return = abs(int(pid_out.update(wheel_error)))
		foward_speed = speed - pid_return
		if positionA >= positionB:
			driveMotors(foward_speed, speed)
			if debug == True:
				print('Moving Right, A: %s, B: %s' % (foward_speed, speed))
				print('Wheel Error: %s' % wheel_error)
				print('PID: %s' % pid_return)
		if positionA < positionB:
			driveMotors(speed, foward_speed)
			if debug == True:
				print('Moving Left, A: %s, B: %s' % (speed, foward_speed))
				print('Wheel Error: %s' % wheel_error)
				print('PID: %s' % pid_return)
		if (positionA - initial_position) >= distance:
			driveMotors(0,0)
			time.sleep(0.1)
			print('-----------FINISHED!--------------')
			print('\n')
			break
			
except KeyboardInterrupt:
	driveMotors(0,0)
	print('----------STOPPED------------')
	print('\n')
	
	

display.set_value(0) #reset display
time.sleep(0.1)
