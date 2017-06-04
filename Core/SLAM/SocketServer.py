#!/usr/bin/env python

'''

A Socket server for sending data over a local network
We use this to send the robots pose and localisation data
over to my main computer to use matplotlib for plotting

The main reason for this is that matplotlib is a piece of shit
and wont install correctly on the raspberry pi

'''

import socket
import sys
import struct
import numpy as np
from io import BytesIO, StringIO
import pickle

class SocketServer(object):

    def __init__(self):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server = ('192.168.43.30', 10000)
        self.address = server
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(server)
        self.sock.listen(1)

    def connect(self):
        #find connections
        self.connection, self.client_address = self.sock.accept()

    def recieve(self, n):
        packet = self.connection.recv(n)
        packet = pickle.loads(packet)

        return packet

if __name__ == '__main__':

    server = SocketServer()
    print('[SLAMBOT] Starting Socket Server on address: %s\n' % server.address[0])
    server.connect()
    while True:
        try:
            server.recieve()
        except KeyboardInterrupt:
            print('\n[SLAMBOT] Stopping Socket Server')
            server.sock.close()
            sys.exit()
