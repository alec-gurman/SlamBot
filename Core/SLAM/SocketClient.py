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

class SocketClient(object):

    def __init__(self):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.address = ('192.168.43.30', 10000)

    def send(self, message):

        try:
            self.sock.connect(self.address)
            print('[SLAMBOT] Connect to: %s\n' % self.address[0])
            self.sock.sendall(message)
        except:
            print('[SLAMBOT][ERROR] Could not connect to: %s\n' % self.address[0])
            break

        self.sock.close()

if __name__ == '__main__':

    client = SocketClient()
    print('[SLAMBOT] Starting Socket Client on address: %s\n' % client.address[0])
    while True:
        client.send('Message: HELLO')

    client.sock.close()
