'''

DISTANCE PIXEL CALIBRATOR CLASS 

'''

import cv2 
import colorBlobDetector as blob

class pixelCalibrate(object):
	
	def __init__(self, knownDistance, knownWidth):

		self.KNOWN_DISTANCE = knownDistance
		self.KNOWN_WIDTH = knownWidth
		self.image_path = 'tools/calibration/opencv_image_0.png'	
		image_calibrate = cv2.imread(self.image_path)
		detectYellow = blob.get_blob('yellow', image_calibrate)
		c_marker = detectYellow.find_marker()	
		self.focalLength = (c_marker[1][0] * knownDistance) / knownWidth
		
	def distance_to_camera(self, perWidth):
		return (self.KNOWN_WIDTH * self.focalLength) / perWidth
	
