'''

Find yellow objects and outline them
find critical information such as area and centroid

updated 22/03 by alec

HOW TO USE:

change the filename of the image below to your directory
run python find_yellow.py

the current HSV values are tuned for the YELLOW DOT
change the lower and upper values below to your own HSV values to detect
different colors. Use hsv_picker.py to assist

'''

import numpy as np
import cv2

img = cv2.imread('saved_images/opencv_image_0.png')

largest_area = 0
lower = np.array([25,150,200]) #color threshold lower value
upper = np.array([40,230,255]) #color threshold upper value
_, contours, _ = cv2.findContours(cv2.inRange(cv2.cvtColor(img,cv2.COLOR_BGR2HSV), lower, upper), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

for cnts in contours:
	area = cv2.contourArea(cnts)
	if area > largest_area:
		largest_area = area
		x,y,w,h = cv2.boundingRect(cnts)
		M = cv2.moments(cnts)
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		cv2.rectangle(img,(x,y), (x+w,y+h), (0,0,255),2)
		cv2.circle(img,(cx,cy),2,(255,0,0),5)
		cv2.line(img,(320,480),(cx,cy),(255,0,0),2)

cv2.line(img,(320,480),(320,380),(0,0,255),2)
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
