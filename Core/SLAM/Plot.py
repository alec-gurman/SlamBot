#!/usr/bin/env python

'''

Plotting script which plots data recieved over socket
via the slambot

'''

import math
import matplotlib.pyplot as plt
from Draw import Draw
from SocketServer import SocketServer

if __name__ == '__main__':

    print('[SLAMBOT] Starting Plotting Script and listening for a robot pose')

    plt.figure()
    plt.axis([0, 2.0, 0, 2.0])
    Draw = Draw()
    Draw.draw_base_map()
    server = SocketServer()
    server.connect()

    while True:
        try:
            data = server.recieve()
            if len(data) > 0:
                if(len(data) == 13):
                    #Recived our state vector, Plot
                    axis_marker = Draw.draw_axis_marker(math.degrees(data[2]))
                    plt.scatter(data[0],data[1],marker=(3, 0, math.degrees(data[2])+26), color='b', s=200)
                    plt.scatter(data[0],data[1],marker=axis_marker, color='r',s=300)
                    for i in range(5):
                        if (data[(3 + (i * 2))] > 0) and (data[(4 + (i * 2))] > 0):
                            plt.scatter(data[3 + (i * 2)],data[4 + (i * 2)],marker=(8,2,0),color='k',s=250)
                else:
                    #recieved our covariance matrix, Plot covariances
                    print('[SLAMBOT][DEBUG] Got Covariance Matrix')
                plt.pause(0.5)
        except KeyboardInterrupt:
            server.sock.close()
            sys.exit()
