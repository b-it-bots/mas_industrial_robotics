"""
This module contains a component that communicates with
the particular arduino board and sends the message framework via
serial port with specified baudrate, timeout and board pid.
"""
import serial
import os
from time import sleep, time

class SerialInterface:
    """
    Initialize class with baudrate, timeout, and pid of arduino board
    which is fixed in our case pid = 239A.
    """
    def __init__(self, baud, timeout, port):
        self.baud = baud
        self.timeout = timeout
        self.port = port
    """
    Detect the arduino board and initialize the serial module from pyserial

    """
    def open_port(self):
        #command=os.popen("ls -l /dev/ttyACM*").read()
        #port="/dev/"+command[command.find('>')+2:].replace("\n","")

        #Opening of the serial port
        try:
            self.arduino = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except:
            print(self.port)
            print('Please check the port')

    """
    Send the string message framework to arduino

    """
    def send_command(self,message_to_send):
        #print("sending cmd")
        self.arduino.flushInput()
        self.arduino.write(message_to_send.encode())

#uno=SerialInterface(9600,1,"/dev/ttyACM1")
#uno=SerialInterface(9600,1,"/dev/ttyACM2")
#uno=SerialInterface(9600,1,"239A")
#uno.open_port()
#uno.send_command("1")
#sleep(1)
#uno.send_command("2")
#sleep(1)
#uno.send_command("3")
#sleep(1)
#uno.send_command("4")
#sleep(1)
