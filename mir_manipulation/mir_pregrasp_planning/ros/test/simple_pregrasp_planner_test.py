#!/usr/bin/env python
"""
Integration test for the 'simple_pregrasp_planner' node.

"""

import math
import unittest
import rospy
import rostest
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg

PKG = 'mir_pregrasp_planning'


class TestSimplePregraspPlanner(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result_pose = None
        self.wait_for_result_pose = None
        self.result_params = None
        self.wait_for_result_params = None
        self.result_grasp_type = None
        self.wait_for_result_grasp_type = None

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.pose_in = rospy.Publisher(
            '~pose_in', geometry_msgs.msg.PoseStamped, queue_size=1
        )

        # subscribers
        self.pose_out = rospy.Subscriber(
            '~pose_out', geometry_msgs.msg.PoseStamped, self.pose_cb
        )
        self.sampling_parameters = rospy.Subscriber(
            '~sampling_parameters', mcr_manipulation_msgs.msg.SphericalSamplerParameters,
            self.sampling_parameters_cb
        )
        self.grasp_type = rospy.Subscriber(
            '~grasp_type', std_msgs.msg.String, self.grasp_type_cb
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.pose_in.unregister()
        self.pose_out.unregister()
        self.sampling_parameters.unregister()
        self.grasp_type.unregister()

    def test_pregrasp_planner(self):
        """
        Verifies that the node returns:
            1) a modified pose,
            2) a set of sampling parameters, and
            3) grasp type;
        given a set of constraints (e.g. orientation and height).

        """
        pose_in = geometry_msgs.msg.PoseStamped()
        # the height (position in Z) will be compared against the height threshold
        pose_in.pose.position.x = 0.2
        pose_in.pose.position.y = 0.0
        pose_in.pose.position.z = 0.12
        # standing orientation (270 degree rotation around the Y axis) plus
        # 90 degree rotation around the X axis
        pose_in.pose.orientation.x = -0.5
        pose_in.pose.orientation.y = 0.5
        pose_in.pose.orientation.z = -0.5
        pose_in.pose.orientation.w = -0.5

        # the pose should be rotated since the height does not exceed the
        # height threshold. The orientation will be change from standing to
        # laying horizontally.
        expected_pose = geometry_msgs.msg.PoseStamped()
        expected_pose.pose.position.x = 0.2
        expected_pose.pose.position.y = 0.0
        expected_pose.pose.position.z = 0.12
        # laying horizontally + 90 degree rotation around the X axis
        # Note: The orientation is set to -180 degree rotation around the Y axis.
        expected_pose.pose.orientation.x = 0.0
        expected_pose.pose.orientation.y = -math.sqrt(2) / 2
        expected_pose.pose.orientation.z = math.sqrt(2) / 2
        expected_pose.pose.orientation.w = 0.0

        while not (self.wait_for_result_pose and
                   self.wait_for_result_params and
                   self.wait_for_result_grasp_type):
            self.event_out.publish('e_start')
            self.pose_in.publish(pose_in)

        self.assertEqual(self.result_grasp_type.data, 'top_grasp')
        self.assertAlmostEqual(self.result_params.radial_distance.minimum, 0.02)
        self.assertAlmostEqual(self.result_params.radial_distance.maximum, 0.06)

        self.assertAlmostEqual(
            self.result_pose.pose.orientation.x, expected_pose.pose.orientation.x,
            places=4
        )
        self.assertAlmostEqual(
            self.result_pose.pose.orientation.y, expected_pose.pose.orientation.y,
            places=4
        )
        self.assertAlmostEqual(
            self.result_pose.pose.orientation.z, expected_pose.pose.orientation.z,
            places=4
        )
        self.assertAlmostEqual(
            self.result_pose.pose.orientation.w, expected_pose.pose.orientation.w,
            places=4
        )

    def pose_cb(self, msg):
        """
        Obtains the output 'pose_out' of the component.

        """
        self.result_pose = msg
        self.wait_for_result_pose = True

    def sampling_parameters_cb(self, msg):
        """
        Obtains the output 'sampling_parameters' of the component.

        """
        self.result_params = msg
        self.wait_for_result_params = True

    def grasp_type_cb(self, msg):
        """
        Obtains the output 'grasp_type' of the component.

        """
        self.result_grasp_type = msg
        self.wait_for_result_grasp_type = True


if __name__ == '__main__':
    rospy.init_node('simple_pregrasp_planner_test')
    rostest.rosrun(PKG, 'simple_pregrasp_planner_test', TestSimplePregraspPlanner)
