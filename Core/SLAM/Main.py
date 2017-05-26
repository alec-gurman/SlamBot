#!/usr/bin/env python

'''

Main SLAM Implementation

'''

from Draw import Draw
from EKF import RobotEKF as ekf

def run_localization(std_vel, std_steer, std_range, std_bearing, ylim=None):

    ekf = RobotEKF(dt, wheelbase=0.5, std_vel=std_vel, std_steer=std_steer)
    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0

    ekf.x = np.array([[robot_x, robot_y, robot_theta]]).T # x, y, theta
    ekf.P = np.diag([.1, .1, .1])

    #NOISE
    ekf.R = np.diag([0.1, 0.1]) #Q range and bearing
    M = np.diag([0.1, 0.1]) #R velocity and angle

    plt.figure()
    Draw.draw_base_map()

    #LOCALISATION Implementation GOES HERE

    plt.show()
    return ekf

if __name__ == "__main__":
    dt = 0.25
    ekf = run_localization(std_vel=0.1, std_steer=np.radians(1),
        std_range=0.3, std_bearing=0.1)
    print('Final P:', ekf.P.diagonal())
