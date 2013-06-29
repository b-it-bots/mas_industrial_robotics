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
from zmq import ssh

def obtainTaskSpecFromServer(ServerIP, ServerPort, TeamName, 
                             use_ssh=False, ssh_server="youbot-hbrs2-pc2"):
    context = zmq.Context()
    connection_address = "tcp://" + ServerIP + ":" + ServerPort
    print "Start connection to " + connection_address
    #  Socket to talk to server
    print "Connecting to server..."
    socket = context.socket(zmq.REQ)
    
    if not use_ssh:	
        socket.connect(connection_address)
    else:
        ssh.tunnel_connection(socket, connection_address,  ssh_server)
    print "Sending request ..."
    socket.send(TeamName)
	
    #  Get the reply.
    message = socket.recv()
    socket.send ("ACK")
    socket.close()
    print "Received message: ", message
    return message




