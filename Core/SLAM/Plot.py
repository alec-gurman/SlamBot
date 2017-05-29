#!/usr/bin/env python

'''

Plotting script which plots data recieved over socket
via to slambot

'''

import math
import matplotlib.pyplot as plt
from Draw import Draw
from SocketServer import SocketServer

if __name__ == '__main__':

    plt.figure()
    plt.axis([0, 2.0, 0, 2.0])
    Draw = Draw()
    Draw.draw_base_map()
    server = SocketServer()
    server.connect()

    print('[SLAMBOT] Starting Plotting Script and listening for a robot pose')

    while True:
        try:
            data = server.recieve()
            if len(data) > 0:
                #plot the data
                axis_marker = Draw.draw_axis_marker(math.degrees(data[2]))
                plt.scatter(data[0],data[1],marker=(3, 0, math.degrees(data[2])+26), color='b', s=200)
                plt.scatter(data[0],data[1],marker=axis_marker, color='r',s=300)
                plt.pause(0.5)
        except KeyboardInterrupt:
            server.sock.close()
            sys.exit()
