'''

COLOR BLOB DETECTOR CLASS


'''

import cv2
import numpy as np

class get_blob(object):

	def __init__(self, color, image):
		self.image = image
		self.lower_yellow = np.array([23,150,200]) #color threshold lower value
		self.upper_yellow = np.array([40,255,255]) #color threshold upper value
		self.lower_red_first = np.array([0,100,90]) #color threshold lower value
		self.upper_red_first = np.array([15,255,255]) #color threshold upper value
		self.lower_red_second = np.array([170,100,90])
		self.upper_red_second = np.array([180,255,255])
		self.lower_green = np.array([30,120,60])
		self.upper_green = np.array([50,255,255])
		self.lower_blue = np.array([100,80,60])
		self.upper_blue = np.array([140,255,255])
		self.hsv_img = cv2.cvtColor(self.image,cv2.COLOR_BGR2HSV) #convert to HSV

		if color == 'yellow':
			self.lower = self.lower_yellow
			self.upper = self.upper_yellow
			self.mask = cv2.inRange(self.hsv_img,self.lower,self.upper)
		if color == 'red':
			self.lower = self.lower_red_first
			self.upper = self.upper_red_first
			self.lower_additional = self.lower_red_second
			self.upper_additional = self.upper_red_second
			self.mask_first = cv2.inRange(self.hsv_img, self.lower, self.upper)
			self.mask_second = cv2.inRange(self.hsv_img, self.lower_additional, self.upper_additional)
			self.mask = cv2.bitwise_or(self.mask_first, self.mask_second)
		if color == 'green':
			self.lower = self.lower_green
			self.upper = self.upper_green
			self.mask = cv2.inRange(self.hsv_img,self.lower,self.upper)
		if color == 'blue':
			self.lower = self.lower_blue
			self.upper = self.upper_blue
			self.mask = cv2.inRange(self.hsv_img,self.lower,self.upper)

		self.largest_area = 0
		self.cx = 0
		self.cy = 0
		self.x = 0
		self.y = 0
		self.area = 0
		self.w = 0
		self.h = 0
		self.area_rect = ((1,1), (1,1) , 1)

	def getFeatures(self, robot_x, robot_y):
		_, contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for cnts in contours:
			self.area = cv2.contourArea(cnts)
			if self.area > self.largest_area:
				self.largest_area = self.area
				self.x,self.y,self.w,self.h = cv2.boundingRect(cnts)
				M = cv2.moments(cnts)
				self.cx = int(M['m10']/M['m00'])
				self.cy = int(M['m01']/M['m00'])
				self.area_rect = cv2.minAreaRect(cnts)

		if self.cx > 0 and self.cy > 0:
			goal_rad = np.arctan2(self.cy - robot_y, self.cx - robot_x)
			self.goal_angle = containAngle360(np.degrees(goal_rad))
		else:
			self.goal_angle = 0 #spin until object is found

		return (self.cx, self.cy, self.goal_angle, self.area_rect, self.largest_area)

	def getMultipleFeatures(self, robot_x, robot_y):
		_, contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		self.multiple_contours = []
		for cnts in contours:
			self.area = cv2.contourArea(cnts)
			if self.area > 50:
				current_contour = []
				self.x,self.y,self.w,self.h = cv2.boundingRect(cnts)
				M = cv2.moments(cnts)
				self.cx = int(M['m10']/M['m00'])
				self.cy = int(M['m01']/M['m00'])
				self.area_rect = cv2.minAreaRect(cnts)
				if self.cx > 0 and self.cy > 0:
					goal_rad = np.arctan2(self.cy - robot_y, self.cx - robot_x)
					self.goal_angle = contain_pi(goal_rad) #contain angle between pi and -pi
				else:
					self.goal_angle = 0
				current_contour.append(self.area)
				current_contour.append(self.cx)
				current_contour.append(self.cy)
				current_contour.append(self.x)
				current_contour.append(self.y)
				current_contour.append(self.w)
				current_contour.append(self.h)
				current_contour.append(self.area_rect)
				current_contour.append(self.goal_angle)
				self.multiple_contours.append(current_contour)

		return self.multiple_contours

	def drawFeatures(self):

		if self.cx > 0 and self.cy > 0:
			cv2.putText(self.image, 'X: {}'.format(self.cx), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
			cv2.putText(self.image, 'Y: {}'.format(self.cy), (50,80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
			cv2.putText(self.image, 'Heading: {}'.format(self.goal_angle), (50,110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2,cv2.LINE_AA)
			cv2.rectangle(self.image,(self.x,self.y), (self.x+self.w,self.y+self.h), (0,255,0),2)
			cv2.circle(self.image,(self.cx,self.cy),2,(255,0,0),5)
			cv2.line(self.image,(320,480),(self.cx,self.cy),(255,0,0),2)

	def drawMultipleFeatures(self, array):

			for blobs in array:
				cv2.rectangle(self.image,(blobs[3],blobs[4]), (blobs[3]+blobs[5],blobs[4]+blobs[6]), (0,255,0),2)
				cv2.circle(self.image,(blobs[1],blobs[2]),2,(255,0,0),5)
				cv2.line(self.image,(320,480),(blobs[1],blobs[2]),(255,0,0),2)

	def find_marker(self):

		_, contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for cnts in contours:
			self.area = cv2.contourArea(cnts)
			if self.area > self.largest_area:
				self.largest_area = self.area
				self.area_rect = cv2.minAreaRect(cnts)
		return self.area_rect

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
