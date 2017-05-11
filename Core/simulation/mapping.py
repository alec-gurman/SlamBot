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
import base_map as bm

fig, ax = plt.subplots()

robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0
steps = 0
rate = 0.25 #loop at 5Hz
initialize_map = bm.base_map()

muX = np.array([[robot_x],
                [robot_y],
                [robot_theta]])

sigmaX = np.eye(3)

R = np.array([[2, 0, 0],
              [0, 2, 0],
              [0, 0, 2]])

Q = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])

def sense(steps, sensor):
    num_z = 5 #length of map
    idx = 1 + (steps-1) * num_z
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

def plot_cov(muX,sigmaX):
    sigmaX = sigmaX[0:2, 0:2]
    muX = muX[0:2]
    elliX = muX[0,0]
    elliY = muX[1,0]
    if not (np.any(np.diagonal(sigmaX) == 0)):
        V, D = np.linalg.eig(sigmaX)
        ellipse = Ellipse(xy=(elliX, elliY), width=V[1], height=V[0],
                                edgecolor='b', fc='none', lw=1)
        ax.add_patch(ellipse)

if __name__ == '__main__':

    initialize_map.draw_base_map()
    data_file = open("data.txt",'r')
    sensor_file = open("sensor.txt",'r')
    xr_file = open("xr.txt", 'r')
    with data_file as f:
         data = [tuple(map(float, i.split(','))) for i in f]
    with sensor_file as f:
         sensor = [tuple(map(float, i.split(','))) for i in f]
    with xr_file as f:
         xr = [tuple(map(float, i.split(','))) for i in f]

    while steps < 50:
        delta_d, delta_theta = data[steps]
        robot_theta = robot_theta + math.degrees(delta_theta)
        robot_x = robot_x + delta_d * math.cos(math.radians(robot_theta))
        robot_y = robot_y + delta_d * math.sin(math.radians(robot_theta))
        sensor_read = sense(steps, sensor)
        XJacobian = np.array([[1, 0, (-delta_d*math.sin(math.radians(robot_theta)))],
                              [0, 1, (delta_d*math.cos(math.radians(robot_theta)))],
                              [0, 0, 1]])
        UJacobian = np.array([[(np.cos(math.radians(robot_theta))), 0],
                              [(np.sin(math.radians(robot_theta))), 0],
                              [0, 1]])

        #PREDICTION
        muX = np.array([[robot_x],
                        [robot_y],
                        [robot_theta]])


        sigmaX = (XJacobian * sigmaX * np.transpose(XJacobian)) + (UJacobian * R * np.transpose(UJacobian))
        #UPDATE FOR EACH LANDMARK
        for i in range(5):
            landmark_pos = initialize_map.get_landmark_pos(i)
            landmark_x = landmark_pos[1]
            landmark_y = landmark_pos[2]
            rnge = sensor_read[i][0]
            print rnge
            GJacobian = np.array([[(-(landmark_x - robot_x)/rnge), (-(landmark_y - robot_y)/rnge), 0],
                                  [((landmark_y - robot_y)/np.power(rnge, 2)), (-(landmark_x - robot_x)/np.power(rnge,2)), -1],
                                  [0, 0, 1]])
            K1 = sigmaX * np.transpose(GJacobian)
            # K2 = (GJacobian * sigmaX * np.transpose(GJacobian)) + Q
            # K = K1 * np.linalg.inv(K2) #is this correct for inverse?
            # z = np.array([[sensor_read[i][0]],
            #              [sensor_read[i][1]]])
            # H = np.array([[np.sqrt(np.power((robot_x - landmark_x),2) + np.power((robot_y - landmark_y), 2))],
            #               [(np.arctan2((landmark_y - robot_y),(landmark_x - robot_x)) - robot_theta)]])
            # r = z - H
            # sized_r = np.array([[r[0][0], 0, 0],
            #                     [0, r[1][0], 0],
            #                     [0, 0, 0]])
            # muX = muX + (K * sized_r)
            #sigmaX = (eye(3) - K * GJacobian) * sigmaX

        #plot_cov(muX,sigmaX)

        #x_true is our real robot data
        x_true = ask_the_oracle(steps,xr)
        axis_marker = draw_axis_marker(math.degrees(x_true[2]))
        axis_marker_predict = draw_axis_marker(muX[2,0])

        #plot the data
        plt.scatter(x_true[0],x_true[1],marker=(3, 0, math.degrees(x_true[2])+26), color='b', s=200)
        plt.scatter(x_true[0],x_true[1],marker=axis_marker, color='r',s=300)
        plt.scatter(muX[0,0],muX[1,0],marker=(3, 0, (muX[2,0]+26)), color='y',s=200)
        plt.scatter(muX[0,0],muX[1,0],marker=axis_marker_predict, color='r',s=300)
        plt.pause(rate)
        steps += 1

    plt.show()
