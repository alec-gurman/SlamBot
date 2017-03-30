'''

HSV TRHESHOLD VALUE FINDER SCRIPT
used to find the upper and lower limits of a color
you are trying to threshold in HSV colorspace

updated 22/03 by alec

HOW TO USE:

change "file_name" below to your image file
run python hsv_picker.py

Step 1.
	- Move all UPPER H, S, V sliders to there MAX setting
Step 2.
	- Slide the LOWER H value until your wanted color is on the verge of going
	all black, then go back a bit so its a constant color
	- Slide the LOWER S, V sliders to try and get rid of other specs of color we dont want
	- Slide the UPPER H, S ,V sliders to improve mask of wanted color even more
Step 3.
	- record values for use in a script

'''

import numpy as np
import cv2 

def nothing(x):
	pass

file_name = 'saved_images/opencv_image_0.png'

#define the image
cv2.namedWindow('image')

#create trackbars
cv2.createTrackbar('H_lower', 'image', 0, 180, nothing)
cv2.createTrackbar('S_lower', 'image', 0, 255, nothing)
cv2.createTrackbar('V_lower', 'image', 0, 255, nothing)

cv2.createTrackbar('H_upper', 'image', 0, 180, nothing)
cv2.createTrackbar('S_upper', 'image', 0, 255, nothing)
cv2.createTrackbar('V_upper', 'image', 0, 255, nothing)

while(1):

	img = cv2.imread(file_name)
	#convert the image to hsv
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	#get trackbar positions
	h_lower = cv2.getTrackbarPos('H_lower', 'image')
	s_lower = cv2.getTrackbarPos('S_lower', 'image')
	v_lower = cv2.getTrackbarPos('V_lower', 'image')

	h_upper = cv2.getTrackbarPos('H_upper', 'image')
	s_upper = cv2.getTrackbarPos('S_upper', 'image')
	v_upper = cv2.getTrackbarPos('V_upper', 'image')

	#define range of yellow color in HSV
	lower_yellow = np.array([h_lower,s_lower,v_lower])
	upper_yellow = np.array([h_upper,s_upper,v_upper])

	#threshold the HSV image to get only the required colour
	mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

	#bitwise AND mask and original image
	res = cv2.bitwise_and(img,img,mask = mask)

	cv2.imshow('image', res)
	
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()

#RECORDED VALUES

'''

yellow_lower 25,150,200
yellow_upper 40,230,255

'''
