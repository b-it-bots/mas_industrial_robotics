#!/usr/bin/python
#
#   Python Client
#   Needs the tcp//... address of the server as argument 
#	(e.g. $ pyton2.7 hwclient.py tcp://localhost.5555
#   Sends the robot name ( in our case b-it-bots) 
#   and waits for reply from the server
#	
import zmq
import sys

def obtainTaskSpecFromServer(ServerIP, ServerPort, TeamName):
	context = zmq.Context()
	connection_address = "tcp://" + ServerIP + ":" + ServerPort
	print "Start connection to " + connection_address
	#  Socket to talk to server
	print "Connecting to server..."
	socket = context.socket(zmq.REQ)
	socket.connect (connection_address)

	print "Sending request ..."
	socket.send (TeamName)
		
	#  Get the reply.
	message = socket.recv()
	socket.send ("ACK")
	socket.close()
	print "Received message: ", message
	return message




