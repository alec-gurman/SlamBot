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

class SocketServer(object):

    def __init__(self):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server = ('192.168.43.30', 10000)
        self.address = server
        self.sock.bind(server)
        self.sock.listen(1)

    def recieve(self):

        #find connections
        self.connection, self.client_address = self.sock.accept()
        try:
            data = connection.recv(999)
            print('[SLAMBOT] Connected to: %s\n' % self.client_address)
            print('[SLAMBOT] Recieved: \n%s\n' % data)
        except:
            print('[SLAMBOT][ERROR] Could not connect')
            self.connection.close()

if __name__ == '__main__':

    server = SocketServer()
    print('[SLAMBOT] Starting Socket Server on address: %s\n' % server.address[0])
    while True:
        server.recieve()

    server.connection.close()
    server.sock.close()
