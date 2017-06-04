#!/usr/bin/env python

'''

Plotting script which plots data recieved over socket
via the slambot

'''

import math
import matplotlib.pyplot as plt
import time
import sys
from Draw import Draw
from SocketServer import SocketServer
from filterpy.stats import plot_covariance_ellipse

if __name__ == '__main__':

    print('[SLAMBOT] Connected! Waiting for some points...')

    plt.figure()
    Draw = Draw()
    Draw.draw_base_map()
    server = SocketServer()
    server.connect()

    while True:
        data = server.recieve(4096)
        if data is not None:
            if len(data) > 0:
                u = data
                print(data)
                #sigma = data[1]
                #if(len(data) == 13):
                #Recived our state vector, Plot
                axis_marker = Draw.draw_axis_marker(math.degrees(float(u[2])))
                plt.scatter(float(u[0]),float(u[1]),marker=(3, 0, math.degrees(float(u[2]))+26), color='b', s=200)
                plt.scatter(float(u[0]),float(u[1]),marker=axis_marker, color='r',s=300)
                for i in range(5):
                    if not((float(u[(3 + (i * 2))]) == 0.0) and (float(u[(4 + (i * 2))]) == 0.0)):
                        #Plot Landmark Position
                        plt.scatter(float(u[3 + (i * 2)]),float(u[4 + (i * 2)]),marker=(8,2,0),color='k',s=250)
                        #Plot Landmark Covariance (if exists)
                        sigma_location = 4 + (i * 2)
                        # plot_covariance_ellipse(
                        #     (u[3 + (i * 2)], u[4 + (i * 2)]),
                        #     sigma[sigma_location:(sigma_location + 2), sigma_location:(sigma_location + 2)],
                        #     std=6, facecolor='none', ec='#004080', alpha=0.3)

                #Plot the robot covariance
                #plot_covariance_ellipse(
                #    (float(u[0]), float(u[1])), (sigma[0:2, 0:2]),
                #    std=6, facecolor='none', ec='#004080', alpha=0.3)

                print(u)
                #print(sigma)
                plt.axis([-1.0, 2.0, -1.0, 2.0])
                plt.pause(0.01)
