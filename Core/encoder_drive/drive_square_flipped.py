#!usr/bin/python3

'''

Assessment Script, Drive in a square of side length 1 m

PID CONTROLLED

'''

import penguinPi as ppi
import time
from wheel_encoders import get_distance



#PID VARIABLES

mainP = 3200.0
mainI = 0.0
mainD = 0.0

mainP_turn = 5500.0
mainI_turn = 0.0
mainD_turn = 50.0

speed = 65 #try turn this down for more accuracy in straights?
max_speed = 100
speed_turn = 80


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

#create our device objects

ppi.init()
pid_out = PID(mainP,mainI,mainD)
pid_turn = PID(mainP_turn, mainI_turn, mainD_turn)
mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)
display = ppi.Display(ppi.AD_DISPLAY_A)
display.set_mode('u')

def driveMotors(speedA, speedB):

	mA.set_power(-speedA) #wheels facing fowards
	mB.set_power(speedB) #wheels facing fowards
	
try:
	while True:
		initial_ticksA = mA.get_ticks() 
		initial_ticksB = mB.get_ticks() 
		ticksA = (mA.get_ticks()) - initial_ticksA
		ticksB = (mB.get_ticks()) - initial_ticksB
		distances = get_distance(ticksA,ticksB)
		initial_position = distances.return_distanceA()
		side = 1
		while side == 1:
			ticksA = (mA.get_ticks()) - initial_ticksA
			ticksB = (mB.get_ticks()) - initial_ticksB
			distances = get_distance(ticksA,ticksB)
			positionA = abs(distances.return_distanceA())
			positionB = abs(distances.return_distanceB())
			wheel_error = abs((ticksA / 360) - (ticksB / 90))
			pid_return = abs(int(pid_out.update(wheel_error)))
			foward_speed = speed - pid_return
			#create some thresholds for max speed
			if foward_speed > max_speed:
				foward_speed = max_speed
			if foward_speed < -max_speed:
				foward_speed = -max_speed
			if positionA >= positionB:
				driveMotors(foward_speed, speed)
				print('Moving Right, A: %s, B: %s' % (foward_speed, speed))
				print('Wheel Error: %s' % wheel_error)
				print('PID: %s' % pid_return)
			if positionA < positionB:
				driveMotors(speed, foward_speed)
				print('Moving Left, A: %s, B: %s' % (speed, foward_speed))
				print('Wheel Error: %s' % wheel_error)
				print('PID: %s' % pid_return)
			if (positionA - initial_position) >= 1:
				driveMotors(0,0)
				side = 2
		time.sleep(2)
		turn_init_position = distances.return_distanceA()
		inc = 0
		initial_ticksA = mA.get_ticks()
		initial_ticksB = mB.get_ticks()
		while side == 2:
			ticksA = (mA.get_ticks()) - initial_ticksA
			ticksB = (mB.get_ticks()) - initial_ticksB
			pid_turn.setPoint(-0.303)
			distances = get_distance(ticksA,ticksB)
			positionA = abs(distances.return_distanceA())
			positionB = abs(distances.return_distanceB())
			wheel_error = abs(positionA - positionB)
			pid_return_turn = pid_turn.update(wheel_error)
			turn_speed = int(pid_return_turn)
			if turn_speed > speed_turn:
				turn_speed = speed_turn
			if turn_speed < -speed_turn:
				turn_speed = -speed_turn
			driveMotors(-turn_speed,turn_speed)
			print("turn: %s" % int(pid_return_turn))
			if turn_speed < 20:
				inc = inc + 1
				if inc > 100:
					side = 1
					driveMotors(0,0)
					
		time.sleep(2)
			
except KeyboardInterrupt:
	print('Stopped!')
	driveMotors(0,0)
			
	

#decompose into left and right wheel velocities
