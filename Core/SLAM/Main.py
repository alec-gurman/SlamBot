#!/usr/bin/env python

'''

Main SLAM Implementation

'''

from Draw import Draw as Draw
from EKF import RobotEKF as ekf
from PID import pidcontrol as PID
import Motors
from Motors import odometry as odom
import numpy as np

class robot(object):

    def _init_(self, std_vel, std_steer):
        self.std_vel = std_vel
        self.std_steer = std_steer
        self.max_angular = std_vel - std_steer

    def update_pose(self, current_pose):

        self.x = self.x + current_pose

def contain_pi(theta):
    '''
    Little function to contain an angle between -pi and pi
    '''

    #WRAP BETWEEN -pi AND pi
    theta = theta % (2 * np.pi)    # force in range [0, 2 pi)
    if theta > np.pi:             # move to [-pi, pi)
        theta -= 2 * np.pi
    return theta


def drive_to_global(x, y, robot, PID):
    '''
    Drive to a given global position
    inputs: x, y
    '''

    #have we reached our goal?
    if robot.x[0] < x or robot.x[1] < y:
        odom.set_initial()
        heading = contain_pi(np.arctan2(robot.x[1] - y, robot.x[0] - x)) #in radians
        # the output of our pid represents a signed number depending on the direction we are turning
        pid_return = abs(PID.update(heading)) #abs so that our motors dont travel in reverse direction
        #limit to robots maximum angular velocity
        if pid_return < robot.max_angular:
            pid_return = robot.max_angular
        pid_wheel = int(robot.std_vel - pid_return) #integer as our motor function only accepts an integer
        #basic drive loop
		if heading <= 0.0:
			motors.driveMotors(pid_wheel, robot.std_vel)
		if heading > 0.0:
			motors.driveMotors(robot.std_vel,pid_wheel)

    current_pose = odom.update()
    robot.update_pose(current_pose)


if __name__ == "__main__":

    motors.init() #setup the motors
    PID(0.5,0.0,0.0) #P, I, D

    robot = robot(std_vel=40, std_steer=30) #speed units are in a scaled from 0 to 100
    robot.x = np.array([[0.0, 0.0, 0.0]]).T #robot_x, robot_y, robot_theta

    while True:
        drive_to_global(0.5, 0, robot, PID) #units are in meters
