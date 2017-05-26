#!/usr/bin/env python

'''

MAPPING SCRIPT WEEK 10

'''

import time
import math
import matplotlib.pyplot as plt
import matplotlib.path as mpath
from matplotlib.patches import Ellipse
import numpy as np
from numpy import dot
import base_map as bm
from filterpy.kalman import ExtendedKalmanFilter as EKF
import sympy
from sympy.abc import alpha, x, y, v, w, R, theta
from sympy import symbols, Matrix
from filterpy.stats import plot_covariance_ellipse
from numpy.random import randn

class RobotEKF(EKF):
    def __init__(self, dt, wheelbase, std_vel, std_steer):
        EKF.__init__(self, 3, 2, 2)
        self.dt = dt
        self.wheelbase = wheelbase
        self.std_vel = std_vel
        self.std_steer = std_steer

        a, x, y, v, w, theta, time = symbols(
            'a, x, y, v, w, theta, t')
        d = v*time
        beta = (d/w)*sympy.tan(a)
        r = w/sympy.tan(a)

        self.fxu = Matrix(
            [[x-r*sympy.sin(theta)+r*sympy.sin(theta+beta)],
             [y+r*sympy.cos(theta)-r*sympy.cos(theta+beta)],
             [theta+beta]])

        self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        self.V_j = self.fxu.jacobian(Matrix([v, a]))

        # save dictionary and it's variables for later use
        self.subs = {x: 0, y: 0, v:0, a:0,
                     time:dt, w:wheelbase, theta:0}
        self.x_x, self.x_y, = x, y
        self.v, self.a, self.theta = v, a, theta

    def predict(self, xjac, ujac, m, u=0):
        self.x = self.move(self.x, u, self.dt)

        # self.subs[self.theta] = self.x[2, 0]
        # self.subs[self.v] = u[0]
        # self.subs[self.a] = u[1]
        #
        # F = np.array(self.F_j.evalf(subs=self.subs)).astype(float)
        # V = np.array(self.V_j.evalf(subs=self.subs)).astype(float)

        # covariance of motion noise in control space
        M = m
        self.P = dot(xjac, self.P).dot(xjac.T) + dot(ujac, M).dot(ujac.T)
        #self.P = dot(F, self.P).dot(F.T) + dot(V, M).dot(V.T)


    def move(self, x, u, dt):
        robot_heading = x[2, 0]
        vel = u[0]
        delta_theta = u[1]
        delta_d = vel * dt

        # if abs(delta_theta) > 0.001: # is robot turning?
        #     beta = (dist / self.wheelbase) * np.tan(delta_theta)
        #     r = self.wheelbase / np.tan(delta_theta) # radius
        #
        #     dx = np.array([[-r*np.sin(hdg) + r*np.sin(r + beta)],
        #                    [r*np.cos(hdg) - r*np.cos(hdg + beta)],
        #                    [beta]])
        # else: # moving in straight line
        #     dx = np.array([[delta_d*np.cos(robot_theta)],
        #                    [delta_d*np.sin(robot_theta)],
        #                    [0]])

        #SIMULATION MOTION CONTROL MODEL
        dx = np.array([[delta_d * np.cos(robot_heading)],
                       [delta_d * np.sin(robot_heading)],
                       [delta_theta]])
        return x + dx

def sense(steps, sensor):
    num_z = 5 #length of map
    idx = (1 + (steps-1)) * num_z
    z = []
    for i in range(idx,idx+num_z):
        z.append(sensor[i])

    return z

def ask_the_oracle(steps, xr):
    x_true = []
    x_true = xr[steps]
    return x_true

def draw_axis_marker(angle):

    outerangle = math.radians((angle + 90))
    normalangle = math.radians(angle)

    topleft_x = np.cos(outerangle) # 1.0
    topleft_y = np.sin(outerangle) # 0.0impo

    bottomleft_x = np.cos(normalangle)
    bottomleft_y = np.sin(normalangle)

    verts = [
        (0.0, 0.0), # left, bottom
        (topleft_x, topleft_y), # left, top
        (topleft_x, topleft_y),
        (0.0, 0.0),
        (bottomleft_x, bottomleft_y), # right, bottom
        (bottomleft_x, bottomleft_y),
        (0.0, 0.0),
        ]

    codes = [mpath.Path.MOVETO,
             mpath.Path.LINETO,
             mpath.Path.LINETO,
             mpath.Path.LINETO,
             mpath.Path.LINETO,
             mpath.Path.LINETO,
             mpath.Path.CLOSEPOLY,
             ]

    path = mpath.Path(verts, codes)

    return path

def H_of(x, landmark_pos, rng):

    px = landmark_pos[0]
    py = landmark_pos[1]
    hyp = rng**2
    dist = rng

    H = np.array([[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
                  [(py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
    return H

def Hx(x, landmark_pos):

    px = landmark_pos[0]
    py = landmark_pos[1]
    Hx = np.array([[(np.sqrt((x[0, 0] - px)**2 + (x[1, 0] - py)**2))],
                [(math.atan2(py - x[1, 0], px - x[0, 0])) - x[2, 0]]])

    #WRAP BETWEEN -pi AND pi
    Hx[1] = Hx[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if Hx[1] > np.pi:             # move to [-pi, pi)
        Hx[1] -= 2 * np.pi

    return Hx

def residual(a, b):
    """ compute residual (a-b) between measurements containing
    [range, bearing]. Bearing is normalized to [-pi, pi)"""
    y = a - b
    #WRAP BETWEEN -pi AND pi
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y

def z_landmark(lmark, sim_pos):
    px = lmark[0]
    py = lmark[1]
    z = np.array([[np.sqrt((sim_pos[0, 0] - px)**2 + (sim_pos[1, 0] - py)**2)],
                [np.arctan2(py - sim_pos[1, 0], px - sim_pos[0, 0]) - sim_pos[2, 0]]])
    return z

def ekf_update(ekf, z, landmark, rng):
    ekf.update(z, HJacobian=H_of, Hx=Hx,
               residual=residual,
               args=(landmark, rng), hx_args=(landmark))


def run_localization(std_vel, std_steer,
                     std_range, std_bearing, ylim=None):
    ekf = RobotEKF(dt, wheelbase=0.5, std_vel=std_vel,
                   std_steer=std_steer)
    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0
    ekf.x = np.array([[robot_x, robot_y, robot_theta]]).T # x, y, theta
    ekf.P = np.diag([.1, .1, .1])

    #NOISE
    ekf.R = np.diag([0.1, 0.1]) #Q range and bearing
    M = np.diag([0.1, 0.1]) #R velocity and angle

    data_file = open("data.txt",'r')
    sensor_file = open("sensor.txt",'r')
    xr_file = open("xr.txt", 'r')
    with data_file as f:
         data = [tuple(map(float, i.split(','))) for i in f]
    with sensor_file as f:
         sensor = [tuple(map(float, i.split(','))) for i in f]
    with xr_file as f:
         xr = [tuple(map(float, i.split(','))) for i in f]

    plt.figure()
    initialize_map = bm.base_map()
    initialize_map.draw_base_map()

    steps = 0
    for i in range(49):
        delta_d, delta_theta = data[steps]
        robot_vel = delta_d / dt
        sensor_read = sense(steps, sensor)

        XJacobian = np.matrix([[1, 0, (-delta_d*math.sin(delta_theta))],
                              [0, 1, (delta_d*math.cos(delta_theta))],
                              [0, 0, 1]])

        UJacobian = np.matrix([[(math.cos(delta_theta)), 0],
                              [(math.sin(delta_theta)), 0],
                              [0, 1]])

        # # steering command (velocity, heading)
        u = np.array([robot_vel, delta_theta])
        ekf.predict(XJacobian, UJacobian, M, u=u)

        for j in range(5):
            rng = sensor_read[j][0]
            bearing = sensor_read[j][1]
            landmark_pos = initialize_map.get_landmark_pos(j)
            lmark = [landmark_pos[1], landmark_pos[2]]
            z_new = np.array([[rng],
                          [bearing]])

            #WRAP BETWEEN -pi AND pi
            z_new[1] = z_new[1] % (2 * np.pi)    # force in range [0, 2 pi)
            if z_new[1] > np.pi:             # move to [-pi, pi)
                z_new[1] -= 2 * np.pi

            # sim_pos = ekf.x.copy() # simulated position
            # z = z_landmark(lmark, sim_pos)
            #print z_new
            ekf_update(ekf, z_new, lmark, rng)

        plot_covariance_ellipse(
            (ekf.x[0,0], ekf.x[1,0]), ekf.P[0:2, 0:2],
            std=6, facecolor='none', alpha=0.8)

        steps += 1

        #x_true is our real robot data
        x_true = ask_the_oracle(steps,xr)
        axis_marker = draw_axis_marker(math.degrees(x_true[2]))
        axis_marker_predict = draw_axis_marker(math.degrees(ekf.x[2,0]))

        #plot the data
        plt.scatter(x_true[0],x_true[1],marker=(3, 0, math.degrees(x_true[2])+26), color='b', s=200)
        plt.scatter(x_true[0],x_true[1],marker=axis_marker, color='r',s=300)
        plt.scatter(ekf.x[0,0],ekf.x[1,0],marker=(3, 0, (math.degrees(ekf.x[2,0]+26))), color='y',s=200)
        plt.scatter(ekf.x[0,0],ekf.x[1,0],marker=axis_marker_predict, color='r',s=300)
        plt.pause(dt)

    plt.show()
    return ekf

if __name__ == '__main__':
    dt = 0.25
    ekf = run_localization(std_vel=0.1, std_steer=np.radians(1),
        std_range=0.3, std_bearing=0.1)
    print('Final P:', ekf.P.diagonal())
