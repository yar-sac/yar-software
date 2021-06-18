# main code for the ground station, mounted to the launchpad. All data must be received from the rocket over radio and transmitted to the analysis tool over a network socket

#from C:/Users/CalSu/Documents/.ACTUAL_DOCS/YAR/yar-software/workspaces/ground-station import server
from server import server

HOST = '192.168.0.36'
PORT = 3000

data_file = 'data_file.txt'
file = open(data_file, 'w')

new_server = server(HOST, PORT)

data = "abcd"

while True:
	file.write(data)
	file.write("\n")
	new_server.send_line(data)
