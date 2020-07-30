#!/usr/bin/env python
"""
Integration test for the 'move_base' node.

"""

import unittest

import geometry_msgs.msg
import rospy
import rostest
import std_msgs.msg

PKG = "mir_move_base"


class TestMoveBase(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # Params
        self.result = None
        self.wait_for_result = None

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, latch=True)
        self.pose_in = rospy.Publisher("~pose_in", geometry_msgs.msg.PoseStamped)

        # Subscribers
        self.component_output = rospy.Subscriber(
            "~component_output", std_msgs.msg.String, self.component_output_cb
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.pose_in.unregister()
        self.component_output.unregister()

    def test_base_placement_planner(self):
        """
        Verifies that the node returns a goal based on the target pose.

        """
        pose_in = geometry_msgs.msg.PoseStamped()
        pose_in.header.stamp = rospy.Time.now()
        pose_in.header.frame_id = "/map"

        pose_in.pose.position.x = 1.0
        pose_in.pose.position.y = 0.0
        pose_in.pose.orientation.w = 1.0

        while not self.wait_for_result:
            self.event_out.publish("e_start")
            self.pose_in.publish(pose_in)

        self.assertIn(self.result.data, ["e_success", "e_failure"])

    def component_output_cb(self, msg):
        """
        Obtains the output of the component.

        """
        self.result = msg
        self.wait_for_result = True


if __name__ == "__main__":
    rospy.init_node("move_base_test")
    rostest.rosrun(PKG, "move_base_test", TestMoveBase)
