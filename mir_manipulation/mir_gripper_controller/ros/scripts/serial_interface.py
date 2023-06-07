"""
This module contains a component that communicates with
the particular arduino board and sends the message framework via
serial port with specified baudrate, timeout and board pid.
"""
import serial
import json
import rospy

class SerialInterface:

    def __init__(self, baud, timeout, pid):
        """This module contains a component that communicates with the particular Teensy board and sends the message via
            serial port with specified baudrate, timeout and pid of the microcontroller board.

            Keyword arguments:
            @param baud -- baudrate of the microcontroller
            @param timeout -- timeout after which communication with the microcontroller is stopped
            @param pid -- pid of the microcontroller, which is fixed to 239A.
        """
        self.baud = baud
        self.timeout = timeout
        self.pid = pid
        self.port = '/dev/youbot/gripper'
    """


    """
    def open_port(self):
        """Function for detecting the microcontroller board and initializing the serial module from pyserial
        """
        try:
            self.board = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except Exception as a:
            rospy.logerr(a)
            rospy.logerr('Please check the port {}'.format(self.port))

    def send(self, message):
        """
        Function for sending the string message framework to the microcontroller.

            Keyword arguments:
            @param message -- message to send to the microcontroller
        """
        self.board.flushInput()
        message_str = json.dumps(message)
        message_str = message_str
        self.board.write(bytes(message_str.encode()))

    def receive(self):
        """
        Function for receiving the string message from the microcontroller.

        Return:
            Message from the microcontroller in the json format.
        """
        msg_str = self.board.readline()

        msg_str = msg_str.decode().strip()


        if len(msg_str) < 5 or not '{' in msg_str:
            return None

        if '}{' not in msg_str:
            return [json.loads(msg_str)]

        msgs = msg_str.split("}{")

        for i in range(len(msgs)):
            if '{' not in msgs[i]:
                msgs[i] = '{' + msgs[i]
            if '}' not in msgs[i]:
                msgs[i] = msgs[i] + '}'

        msgs_json = []

        for msg in msgs:
            msgs_json.append(json.loads(msg))

        return msgs_json

if __name__ == '__main__':
    serial_interface = SerialInterface(9600, 0.1, '239A')
    serial_interface.open_port()
    serial_interface.send({'command': 1})
    rospy.sleep(1)
    serial_interface.send({'command': 0})