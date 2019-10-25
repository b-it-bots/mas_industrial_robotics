#!/usr/bin/env python
"""
Test integration for the world model node.

"""

import unittest
import rostest
import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import tf
import time

PKG = 'mir_refbox_parser'

class RefboxParserTest(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        print("inside test")
        rospy.loginfo("inside test")
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.refbox = rospy.Publisher(
            '~refbox',
           std_msgs.msg.String
        )

        self.event_in = rospy.Publisher(
            '~event_in',
           std_msgs.msg.String
        )
        # subscribers
        rospy.Subscriber("~event_out",std_msgs.msg.String, self.result_callback)

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.refbox.unregister()

    def test_refbox_communication(self):
        """
        Verifies that the node communicates properly with refbox
        and stores the data properly in its database while retrieved

        """
        print("inside test refbox communication")

        expected_result = std_msgs.msg.String()
        expected_result.data = 'e_done'

        inventory_message = "BNT<(C1,W,3),(S1,E,3),(T3,N,3),(S3,S,3),(T1,S,3),(D1,E,3),(S4,N,3),(S5,N,3),(T4,W,3),(T2,S,3),(S2,E,3)>"

        self.event_in.publish("e_trigger")
        print("send e_trigger")
        self.refbox.publish(inventory_message)

        while not self.wait_for_result:
            print "waiting"
            time.sleep(1)

        self.assertEqual(self.result.data, expected_result.data)


    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True



if __name__ == '__main__':
    rospy.init_node('refbox_parser_test')
    rostest.rosrun(PKG, 'refbox_parser_test', RefboxParserTest)

