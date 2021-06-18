# a module that will transmit data received from the rocket over TCP using sockets
# the module will read data from a file and stream that data to a client, which will be the data analysis app also developed

import socket

class server:
	
	def __init__(self, host, port):
		self.HOST = host
		self.PORT = int(port)
		SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		SOCKET.bind((self.HOST, self.PORT))
		SOCKET.listen()
		self.CONN, ADDR = SOCKET.accept()

	

	def send_line(self, data):
		self.CONN.send(data.encode())
