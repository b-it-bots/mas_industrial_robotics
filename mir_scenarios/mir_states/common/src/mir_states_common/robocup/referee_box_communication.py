#!/usr/bin/python
#
#   Python Client
#   Needs the tcp//... address of the server as argument 
#   (e.g. $ pyton2.7 hwclient.py tcp://localhost.5555
#   Sends the robot name ( in our case b-it-bots) 
#   and waits for reply from the server
#	
import zmq
import sys
from zmq import ssh

def obtainTaskSpecFromServer(ServerIP, ServerPort, TeamName, 
                             use_ssh=False, ssh_server="youbot-hbrs2-pc2"):
    context = zmq.Context()
    connection_address = "tcp://" + str(ServerIP) + ":" + str(ServerPort)
    print "Start connection to " + connection_address

    #  Socket to talk to server
    print "Connecting to server..."
    socket = context.socket(zmq.REQ)
    
    if not use_ssh:
        print "connecting directly, not through an ssh tunnel"
        socket.connect(connection_address)
    else:
        print "connecting via ssh tunnel through: " + ssh_server
        ssh.tunnel_connection(socket, connection_address,  ssh_server)

    print "Sending request for task specification"
    socket.send(TeamName)

    #  Get the reply.
    message = socket.recv()
    socket.send ("ACK")
    socket.close()

    print "Received task specification: " + message

    return message



