#!/usr/bin/env python

'''

Main SLAM Implementation

'''

#from Draw import Draw as Draw
#from EKF import RobotEKF as ekf
from PID import pidcontrol as PID
from Motors import odometry as odom
from Landmarks import find_landmark as findL
from Camera import PiVideoStream
import Measure as distcal
import Vision as Vision
import Motors as Motors
import numpy as np
import time

class robot(object):

    def __init__(self, std_vel, std_steer):
        self.std_vel = std_vel
        self.std_steer = std_steer
        self.max_angular = std_vel - std_steer
        self.wheelbase = 0.15

    def update_pose(self, current_pose):

        self.x = self.x + current_pose

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

def time_step(step):

	time.sleep(step)

def find_landmark(ID,Stream,Measure):

    img = Stream.read()

    detectRed = Vision.get_blob('red', img)
    red_blobs = []
    red_blobs = detectRed.getMultipleFeatures(160,240)

    detectGreen = Vision.get_blob('green', img)
    green_blobs = []
    green_blobs = detectGreen.getMultipleFeatures(160,240)

    detectBlue = Vision.get_blob('blue', img)
    blue_blobs = []
    blue_blobs = de tectBlue.getMultipleFeatures(160,240)

    get_landmark = Landmarks.findL(red_blobs,green_blobs,blue_blobs) #initialize the landmarker finder class with our three blobs

    landmark_bearing, landmark_cx, landmark_cy, landmark_area, landmark_marker = get_landmark.position(ID)

    if not (landmark_bearing == 0):
        landmark_range = Measure.distance_to_camera(landmark_marker[1][0])
        return np.array([[landmark_range, landmark_bearing]]).T
    else:
        # print('[SLAMBOT][ERROR] Landmark cannot be seen')
        return np.array([[0.0, 0.0]]).T


def drive_to_global(x, y, robot, PID, robot_odom, dt):
    '''
    Drive to a given global position
    inputs: x, y
    '''

    #have we reached our goal?
    if robot.x[0] < x or robot.x[1] < y:
        #print('[SLAMBOT] Goal not reached, moving robot')
        robot_odom.set_initial()
        heading = (np.arctan2(y - robot.x[1], x - robot.x[0])) - robot.x[2] #in radians
        # the output of our pid represents a signed number depending on the direction we are turning
        pid_return = abs(PID.update(heading)) #abs so that our motors dont travel in reverse direction
        #limit to robots maximum angular velocity
        if pid_return > robot.std_steer:
            pid_return = robot.std_steer
        pid_wheel = int(robot.std_vel - pid_return) #integer as our motor function only accepts an integer
        #basic drive loop
        if heading > 0.0:
            Motors.driveMotors(pid_wheel, robot.std_vel)
        if heading <= 0.0:
            Motors.driveMotors(robot.std_vel,pid_wheel)
    else:
        Motors.driveMotors(0,0)

    time_step(dt)

    current_pose = robot_odom.update(robot.x[2])
    print('Odometry: \n%s\n' % current_pose)
    print('Robot Pose: \n%s\n' % robot.x)
    robot.update_pose(current_pose)


if __name__ == "__main__":

    print('[SLAMBOT] Starting main program')
    print('[SLAMBOT] Warming up the camera')
    #Stream = PiVideoStream().start() #start the video stream on a seperate thread
    #time.sleep(2.0) #allow camera to warm up
    print('[SLAMBOT] Calibrating distances')
    Measure = distcal.pixelCalibrate(1200,90) #calibrate the camera for distances

    print('[SLAMBOT] initializing the motors')
    Motors.init() #setup the motors
    PID = PID(40,0.0,0.0) #P, I, D

    robot = robot(std_vel=40, std_steer=30) #speed units are in a scaled from 0 to 100
    robot.x = np.array([[0.0, 0.0, 0.0]]).T #robot_x, robot_y, robot_theta
    robot_odom = odom(robot.wheelbase)

    dt = 0.25 # TIME STEP IN HZ

    print('[SLAMBOT] Initialization complete, starting')

    while True:

        drive_to_global(0.5, 0.5, robot, PID, robot_odom, dt) #units are in meters
        # for i in range(2):
        #     range_bearing = find_landmark(i,Stream,Measure) #find landmark 1 using the VS video stream
        #     if (range_bearing[0] > 0) and (range_bearing[1] > 0): #landmark found
        #         print('[SLAMBOT] Found landmark: %s\n' % i)
        #         print(range_bearing)

    Motors.driveMotors(0,0)
