'''

CAMERA SETUP CLASS

'''


from picamera.array import PiRGBArray
from picamera import PiCamera

def init():
	camera = PiCamera()
	camera.resolution = (320,240)
	camera.framerate = 30
	camera.sensor_mode = 3
	camera.rotation = 90
	#camera.exposure_compensation = 0
	camera.shutter_speed = camera.exposure_speed
	camera.exposure_mode = 'off'
	#camera.awb_mode = 'off'
	#camera.awb_gains = g
	#camera.contrast = 10
	camera.brightness = 50
	camera.saturation = 50 #brighter colours
	camera.ISO = 100
	rawCapture = PiRGBArray(camera, size=(320,240))
	return (camera, rawCapture)
