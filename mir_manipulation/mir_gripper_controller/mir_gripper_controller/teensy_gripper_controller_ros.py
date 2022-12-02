#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from mir_interfaces.msg import GripperCommand
import mir_gripper_controller.serial_interface as serial_interface
import math
import json
import time


class GripperController(Node):
    """ROS wrapper for receiving commands for the gripper and forwarding them to the Teensy board.
    """

    component_name = 'gripper_controller'

    def __init__(self, node_name):
        super().__init__(node_name)
  
        try:
            self.serial_msg = serial_interface.SerialInterface(9600, 1, "239A")
            self.serial_msg.open_port()
            self.get_logger().info("Dynamixel gripper has been intialized successfully")
            self.gripper_command_topic = self.declare_parameter('~gripper_command_topic', '/arm_1/gripper_command')
            self.feedback_topic = self.declare_parameter('~feedback_topic', '/arm_1/gripper_feedback')

            self.cmd_listener = self.create_subscription(GripperCommand, self.gripper_command_topic.get_parameter_value().string_value, self.cmd_callback, 10)
            self.feedback_publisher = self.create_publisher(String, self.feedback_topic.get_parameter_value().string_value, 10)
            
            # Joint states publisher
            self.join_state_pub = self.create_publisher(JointState, 'joint_state', 10)

            # Perform gripper close open at initialization
            self.get_logger().info("Closing and opening the gripper at initialization.")
            self.serial_msg.send({'command': 1})
            time.sleep(1)
            self.serial_msg.send({'command': 0})

        except Exception as ex:
            self.get_logger().error("Error in initializing the gripper controller: {}".format(ex))


    def cmd_callback(self, data):
        """Callback for receiving gripper command

            Keyword arguments:
            @param data -- command from the gripper
        """
        command = int(data.command)
        json_command = {
                "command": 0,
        }

        if command == 1:
            self.get_logger().debug('Closing the gripper.')
            json_command['command'] = 1

        else:
            self.get_logger().debug('Opening the gripper.')

        self.serial_msg.send(json_command)
    
    def handle_msg(self):
        """Function for receiving feedback from serial and publishing to ros
        
        Assuming feedback is in the below json format:
        {
            "state": "open" or "closed",
            "right_gripper_pos": 0 to 360 degrees,
            "left_gripper_pos": 0 to 360 degrees,
            "parsing_error": 0,
            "last_command": 1 or 0,
            # other fields can be added here
        }
        """

        rate = self.create_rate(0.1)

        msgs = self.serial_msg.receive()
        if msgs is not None:
            for msg in msgs:
                # convert to ros message
                ros_msg = String()
                ros_msg.data = json.dumps(msg)
                self.feedback_publisher.publish(ros_msg)

                # convert gripper positions from degrees to radians
                gripper_right_motor_position = math.radians(msg['right_gripper_pos'])
                gripper_left_motor_position = math.radians(msg['left_gripper_pos'])
                
                # Publish joint states
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = ['gripper_motor_right_joint', 'gripper_motor_left_joint']
                joint_state.position = [gripper_right_motor_position, gripper_left_motor_position]
                joint_state.velocity = [0.0, 0.0]
                joint_state.effort = [0.0, 0.0]
                self.join_state_pub.publish(joint_state)

                rate.sleep()

def main(args=None):
    try:
        rclpy.init(args=args)
        gripper_controller = GripperController('gripper_controller')
        rclpy.spin(gripper_controller)
        rate = gripper_controller.create_rate(10)

        while rclpy.ok():
            gripper_controller.handle_msg()
            rate.sleep()
    
    except Exception as ex:
        print('[Gripper Controller] Exception: {}'.format(ex))


if __name__ == '__main__':
    main()