#!/usr/bin/env python
"""
Integration test for the 'pregrasp_planner_pipeline' node.

"""

import math
import unittest
import rospy
import rostest
import std_msgs.msg
import geometry_msgs.msg
import brics_actuator.msg

PKG = 'mir_pregrasp_planning'


class TestPregraspPlannerPipeline(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result_configuration = None
        self.wait_for_result_configuration = None
        self.result_pose = None
        self.wait_for_result_pose = None

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.pose_in = rospy.Publisher(
            '~pose_in', geometry_msgs.msg.PoseStamped, latch=True, queue_size=1
        )

        # subscribers
        self.configuration_out = rospy.Subscriber(
            '~configuration_out', brics_actuator.msg.JointPositions,
            self.configuration_cb
        )
        self.selected_pose = rospy.Subscriber(
            '~selected_pose', geometry_msgs.msg.PoseStamped, self.selected_pose_cb
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.pose_in.unregister()
        self.configuration_out.unregister()
        self.selected_pose.unregister()

    def test_pregrasp_planner(self):
        """
        Verifies that the node returns a configuration and a pose.
        Note: This is not a functionality test.

        """
        pose_in = geometry_msgs.msg.PoseStamped()
        pose_in.header.frame_id = 'base_link'

        pose_in.pose.position.x = 0.415
        pose_in.pose.position.y = 0.0
        pose_in.pose.position.z = 0.09
        pose_in.pose.orientation.x = 0.0
        pose_in.pose.orientation.y = 0.0
        pose_in.pose.orientation.z = 0.0
        pose_in.pose.orientation.w = 1.0

        while not (self.wait_for_result_configuration and self.wait_for_result_pose):
            self.event_out.publish('e_start')
            self.pose_in.publish(pose_in)

        assert type(self.result_configuration) is brics_actuator.msg.JointPositions
        assert type(self.result_pose) is geometry_msgs.msg.PoseStamped

    def configuration_cb(self, msg):
        """
        Obtains the output configuration of the 'pregrasp_planner_pipeline'.

        """
        self.result_configuration = msg
        self.wait_for_result_configuration = True

    def selected_pose_cb(self, msg):
        """
        Obtains the output pose of the 'pregrasp_planner_pipeline'.

        """
        self.result_pose = msg
        self.wait_for_result_pose = True


if __name__ == '__main__':
    rospy.init_node('pregrasp_planner_pipeline_test')
    rostest.rosrun(PKG, 'pregrasp_planner_pipeline_test', TestPregraspPlannerPipeline)
