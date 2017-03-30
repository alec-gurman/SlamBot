'''

PID CONTROLLER CLASS

'''

class pidcontrol(object):

	def __init__(self, P,I,D):
		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.set_point=180.0
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
