# a test client, to see if data is being sent correctly
# thanks https://realpython.com/python-sockets/

import socket

HOST = '192.168.0.36'
PORT = 3000




SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
SOCKET.connect((HOST, PORT))
    
while True:
    data = SOCKET.recv(1024)
    data = data.decode()
    print(data)

