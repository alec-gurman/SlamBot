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
import time
import struct
import numpy as np
import pickle

class SocketClient(object):

	def __init__(self):

		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.address = ('192.168.43.30', 10000)

	def connect(self):
		self.sock.connect(self.address)

	def send(self, message):
		msg = pickle.dumps(message, protocol=2)
		self.sock.sendall(msg)


if __name__ == '__main__':

	client = SocketClient()
	print('[SLAMBOT] Starting Socket Client on address: %s' % client.address[0])
	client.connect()
	while True:
		try:
			message = np.array([[1.2334, 5.23423432, -2.212425245]])
			client.send(message)
			time.sleep(0.5)
		except KeyboardInterrupt:
			client.sock.close()
			sys.exit()
