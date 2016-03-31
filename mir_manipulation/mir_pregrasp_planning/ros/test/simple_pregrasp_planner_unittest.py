#!/usr/bin/env python
"""
Test unit for the functions in the simple_pregrasp_planner_utils.py module.

"""

import math
import numpy
import numpy.testing
import copy
import unittest
import rosunit
import geometry_msgs.msg
import mir_pregrasp_planning_ros.simple_pregrasp_planner_utils as pregrasp_planner_utils

PKG = 'mir_pregrasp_planning'


class TestSimplePregraspPlanner(unittest.TestCase):
    """
    Tests functions used in the simple_pregrasp_planner_utils.py module.

    """
    def test_modify_pose_no_changes(self):
        """
        Tests that the 'modify_pose' function returns the same pose, since
        the object is not standing.

        """
        height_threshold = 1.0

        pose_in = geometry_msgs.msg.PoseStamped()
        # the height (position in Z) will be compared against the height threshold
        pose_in.pose.position.x = 0.4
        pose_in.pose.position.y = 0.0
        pose_in.pose.position.z = 0.2

        # laying orientation
        pose_in.pose.orientation.x = 0.0
        pose_in.pose.orientation.y = 0.0
        pose_in.pose.orientation.z = 0.0
        pose_in.pose.orientation.w = 1.0

        expected_pose = copy.deepcopy(pose_in)

        result, standing = pregrasp_planner_utils.modify_pose(pose_in, height_threshold)

        self.assertEqual(result, expected_pose)
        self.assertEqual(standing, False)

    def test_modify_pose_short_object(self):
        """
        Tests that the 'modify_pose' function returns a modified version of the pose,
        since the object's height does not exceed the height_threshold. Namely,
        the modified version should have a 90 degree rotation around the Y axis
        with respect to the pose.

        """
        height_threshold = 0.15

        pose_in = geometry_msgs.msg.PoseStamped()
        # the height (position in Z) will be compared against the height threshold
        pose_in.pose.position.x = 0.2
        pose_in.pose.position.y = 0.0
        pose_in.pose.position.z = 0.125
        # standing orientation
        pose_in.pose.orientation.x = 0.0
        pose_in.pose.orientation.y = -math.sqrt(2) / 2
        pose_in.pose.orientation.z = 0.0
        pose_in.pose.orientation.w = math.sqrt(2) / 2

        result, standing = pregrasp_planner_utils.modify_pose(pose_in, height_threshold)

        expected_pose = copy.deepcopy(pose_in)
        expected_pose.pose.orientation.x = 0.0
        expected_pose.pose.orientation.y = -1.0
        expected_pose.pose.orientation.z = 0.0
        expected_pose.pose.orientation.w = 0.0

        self.assertEqual(standing, False)
        self.assertAlmostEqual(
            result.pose.orientation.x, expected_pose.pose.orientation.x, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.y, expected_pose.pose.orientation.y, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.z, expected_pose.pose.orientation.z, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.w, expected_pose.pose.orientation.w, places=4
        )

        # standing orientation + 45 degree rotation around the X axis
        pose_in.pose.orientation.x = 0.2706
        pose_in.pose.orientation.y = -0.65328
        pose_in.pose.orientation.z = 0.2706
        pose_in.pose.orientation.w = 0.65328

        result, standing = pregrasp_planner_utils.modify_pose(pose_in, height_threshold)

        # laying horizontally + 45 degree rotation around the X axis
        expected_pose.pose.orientation.x = 0.0
        expected_pose.pose.orientation.y = -0.9238795
        expected_pose.pose.orientation.z = 0.3826834
        expected_pose.pose.orientation.w = 0.0

        self.assertEqual(standing, False)
        self.assertAlmostEqual(
            result.pose.orientation.x, expected_pose.pose.orientation.x, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.y, expected_pose.pose.orientation.y, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.z, expected_pose.pose.orientation.z, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.w, expected_pose.pose.orientation.w, places=4
        )

        # standing orientation + 90 degree rotation around the X axis
        pose_in.pose.orientation.x = 0.5
        pose_in.pose.orientation.y = -0.5
        pose_in.pose.orientation.z = 0.5
        pose_in.pose.orientation.w = 0.5

        result, standing = pregrasp_planner_utils.modify_pose(pose_in, height_threshold)

        # laying horizontally + 90 degree rotation around the X axis
        expected_pose.pose.orientation.x = 0.0
        expected_pose.pose.orientation.y = -math.sqrt(2) / 2
        expected_pose.pose.orientation.z = math.sqrt(2) / 2
        expected_pose.pose.orientation.w = 0.0

        self.assertEqual(standing, False)
        self.assertAlmostEqual(
            result.pose.orientation.x, expected_pose.pose.orientation.x, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.y, expected_pose.pose.orientation.y, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.z, expected_pose.pose.orientation.z, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.w, expected_pose.pose.orientation.w, places=4
        )

        # standing orientation + 180 degree rotation around the X axis
        pose_in.pose.orientation.x = math.sqrt(2) / 2
        pose_in.pose.orientation.y = 0.0
        pose_in.pose.orientation.z = math.sqrt(2) / 2
        pose_in.pose.orientation.w = 0.0

        result, standing = pregrasp_planner_utils.modify_pose(pose_in, height_threshold)

        # laying horizontally + 180 degree rotation around the X axis
        expected_pose.pose.orientation.x = 0.0
        expected_pose.pose.orientation.y = 0.0
        expected_pose.pose.orientation.z = -1.0
        expected_pose.pose.orientation.w = 0.0

        self.assertEqual(standing, False)
        self.assertAlmostEqual(
            result.pose.orientation.x, expected_pose.pose.orientation.x, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.y, expected_pose.pose.orientation.y, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.z, expected_pose.pose.orientation.z, places=4
        )
        self.assertAlmostEqual(
            result.pose.orientation.w, expected_pose.pose.orientation.w, places=4
        )

    def test_modify_pose_tall_object(self):
        """
        Tests that the 'modify_pose' function returns a modified version of the pose,
        given an object that is standing and its height not exceed the
        height_threshold. Namely, the modified version should retain the orientation
        (i.e. standing), but with no rotation around the axis pointing upwards
        (X axis is assumed in this case).

        """
        height_threshold = 0.15

        pose_in = geometry_msgs.msg.PoseStamped()
        # the height (position in Z) will be compared against the height threshold
        pose_in.pose.position.x = 0.2
        pose_in.pose.position.y = 0.0
        pose_in.pose.position.z = 0.16
        # standing orientation
        pose_in.pose.orientation.x = 0.0
        pose_in.pose.orientation.y = math.sqrt(2) / 2
        pose_in.pose.orientation.z = 0.0
        pose_in.pose.orientation.w = -math.sqrt(2) / 2

        result, standing = pregrasp_planner_utils.modify_pose(pose_in, height_threshold)

        expected_pose = copy.deepcopy(pose_in)
        # standing orientation + 0 degree rotation around the X axis
        expected_pose.pose.orientation.x = 0.0
        expected_pose.pose.orientation.y = math.sqrt(2) / 2
        expected_pose.pose.orientation.z = 0.0
        expected_pose.pose.orientation.w = -math.sqrt(2) / 2

        self.assertEqual(result, expected_pose)
        self.assertEqual(standing, True)

        # standing orientation + 45 degree rotation around the X axis
        pose_in.pose.orientation.x = -0.2706
        pose_in.pose.orientation.y = 0.65328
        pose_in.pose.orientation.z = -0.2706
        pose_in.pose.orientation.w = -0.65328

        result, standing = pregrasp_planner_utils.modify_pose(pose_in, height_threshold)

        # expected_pose: standing orientation + 0 degree rotation around the X axis
        self.assertEqual(result, expected_pose)
        self.assertEqual(standing, True)

        # standing orientation + 90 degree rotation around the X axis
        pose_in.pose.orientation.x = -0.5
        pose_in.pose.orientation.y = 0.5
        pose_in.pose.orientation.z = -0.5
        pose_in.pose.orientation.w = -0.5

        result, standing = pregrasp_planner_utils.modify_pose(pose_in, height_threshold)

        # expected_pose: standing orientation + 0 degree rotation around the X axis
        self.assertEqual(result, expected_pose)
        self.assertEqual(standing, True)

        # standing orientation + 180 degree rotation around the X axis
        pose_in.pose.orientation.x = -math.sqrt(2) / 2
        pose_in.pose.orientation.y = 0.0
        pose_in.pose.orientation.z = -math.sqrt(2) / 2
        pose_in.pose.orientation.w = -0.0

        result, standing = pregrasp_planner_utils.modify_pose(pose_in, height_threshold)

        # expected_pose: standing orientation + 0 degree rotation around the X axis
        self.assertEqual(result, expected_pose)
        self.assertEqual(standing, True)

    def test_modify_pose_rotation_no_changes(self):
        """
        Tests that the 'modify_pose_rotation' function returns the same pose.

        """
        # test angles: different rotations around the Z axis
        zero_degrees = geometry_msgs.msg.PoseStamped()
        zero_degrees.pose.orientation.x = 0.0
        zero_degrees.pose.orientation.y = 0.0
        zero_degrees.pose.orientation.z = 0.0
        zero_degrees.pose.orientation.w = 1.0

        ninety_degrees = geometry_msgs.msg.PoseStamped()
        ninety_degrees.pose.orientation.x = 0.0
        ninety_degrees.pose.orientation.y = 0.0
        ninety_degrees.pose.orientation.z = math.cos(math.pi / 4)
        ninety_degrees.pose.orientation.w = math.cos(math.pi / 4)

        one_eighty_degrees = geometry_msgs.msg.PoseStamped()
        one_eighty_degrees.pose.orientation.x = 0.0
        one_eighty_degrees.pose.orientation.y = 0.0
        one_eighty_degrees.pose.orientation.z = 1.0
        one_eighty_degrees.pose.orientation.w = 0.0

        two_seventy_degrees = geometry_msgs.msg.PoseStamped()
        two_seventy_degrees.pose.orientation.x = 0.0
        two_seventy_degrees.pose.orientation.y = 0.0
        two_seventy_degrees.pose.orientation.z = math.cos(math.pi / 4)
        two_seventy_degrees.pose.orientation.w = -math.cos(math.pi / 4)

        actual_zero = pregrasp_planner_utils.modify_pose_rotation(zero_degrees)
        actual_ninety = pregrasp_planner_utils.modify_pose_rotation(ninety_degrees)
        actual_one_eighty = pregrasp_planner_utils.modify_pose_rotation(
            one_eighty_degrees
        )
        actual_two_seventy = pregrasp_planner_utils.modify_pose_rotation(
            two_seventy_degrees
        )

        self.assertAlmostEqual(
            actual_zero.pose.orientation.x, zero_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.y, zero_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.z, zero_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.w, zero_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_ninety.pose.orientation.x, ninety_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.y, ninety_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.z, ninety_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.w, ninety_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.x, one_eighty_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.y, one_eighty_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.z, one_eighty_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.w, one_eighty_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.x, two_seventy_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.y, two_seventy_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.z, two_seventy_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.w, two_seventy_degrees.pose.orientation.w,
            places=5
        )

    def test_modify_pose_rotation_add_offset(self):
        """
        Tests that the 'modify_pose_rotation' function returns the same pose,
        but with a rotation offset around the Z axis.

        """
        # test angles: different rotations around the Z axis
        zero_degrees = geometry_msgs.msg.PoseStamped()
        zero_degrees.pose.orientation.x = 0.0
        zero_degrees.pose.orientation.y = 0.0
        zero_degrees.pose.orientation.z = 0.0
        zero_degrees.pose.orientation.w = 1.0

        ninety_degrees = geometry_msgs.msg.PoseStamped()
        ninety_degrees.pose.orientation.x = 0.0
        ninety_degrees.pose.orientation.y = 0.0
        ninety_degrees.pose.orientation.z = math.cos(math.pi / 4)
        ninety_degrees.pose.orientation.w = math.cos(math.pi / 4)

        one_eighty_degrees = geometry_msgs.msg.PoseStamped()
        one_eighty_degrees.pose.orientation.x = 0.0
        one_eighty_degrees.pose.orientation.y = 0.0
        one_eighty_degrees.pose.orientation.z = 1.0
        one_eighty_degrees.pose.orientation.w = 0.0

        two_seventy_degrees = geometry_msgs.msg.PoseStamped()
        two_seventy_degrees.pose.orientation.x = 0.0
        two_seventy_degrees.pose.orientation.y = 0.0
        two_seventy_degrees.pose.orientation.z = math.cos(math.pi / 4)
        two_seventy_degrees.pose.orientation.w = -math.cos(math.pi / 4)

        actual_zero = pregrasp_planner_utils.modify_pose_rotation(
            zero_degrees, offset=0.0
        )
        actual_ninety = pregrasp_planner_utils.modify_pose_rotation(
            zero_degrees, offset=90
        )
        actual_one_eighty = pregrasp_planner_utils.modify_pose_rotation(
            zero_degrees, offset=180
        )
        actual_two_seventy = pregrasp_planner_utils.modify_pose_rotation(
            zero_degrees, offset=270
        )

        self.assertAlmostEqual(
            actual_zero.pose.orientation.x, zero_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.y, zero_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.z, zero_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.w, zero_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_ninety.pose.orientation.x, ninety_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.y, ninety_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.z, ninety_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.w, ninety_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.x, one_eighty_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.y, one_eighty_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.z, one_eighty_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.w, one_eighty_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.x, two_seventy_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.y, two_seventy_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.z, two_seventy_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.w, two_seventy_degrees.pose.orientation.w,
            places=5
        )

    def test_modify_pose_rotation_limited_range(self):
        """
        Tests that the 'modify_pose_rotation' function returns the same pose,
        but with a rotation limited to only 180 degrees around the Z axis.

        """
        # test angles: different rotations around the Z axis
        zero_degrees = geometry_msgs.msg.PoseStamped()
        zero_degrees.pose.orientation.x = 0.0
        zero_degrees.pose.orientation.y = 0.0
        zero_degrees.pose.orientation.z = 0.0
        zero_degrees.pose.orientation.w = 1.0

        ninety_degrees = geometry_msgs.msg.PoseStamped()
        ninety_degrees.pose.orientation.x = 0.0
        ninety_degrees.pose.orientation.y = 0.0
        ninety_degrees.pose.orientation.z = math.cos(math.pi / 4)
        ninety_degrees.pose.orientation.w = math.cos(math.pi / 4)

        one_eighty_degrees = geometry_msgs.msg.PoseStamped()
        one_eighty_degrees.pose.orientation.x = 0.0
        one_eighty_degrees.pose.orientation.y = 0.0
        one_eighty_degrees.pose.orientation.z = 1.0
        one_eighty_degrees.pose.orientation.w = 0.0

        two_seventy_degrees = geometry_msgs.msg.PoseStamped()
        two_seventy_degrees.pose.orientation.x = 0.0
        two_seventy_degrees.pose.orientation.y = 0.0
        two_seventy_degrees.pose.orientation.z = math.cos(math.pi / 4)
        two_seventy_degrees.pose.orientation.w = -math.cos(math.pi / 4)

        actual_zero = pregrasp_planner_utils.modify_pose_rotation(
            zero_degrees, rotation_range=[0, 180]
        )
        actual_ninety = pregrasp_planner_utils.modify_pose_rotation(
            ninety_degrees, rotation_range=[0, 180]
        )
        actual_one_eighty = pregrasp_planner_utils.modify_pose_rotation(
            one_eighty_degrees, rotation_range=[0, 180]
        )
        actual_two_seventy = pregrasp_planner_utils.modify_pose_rotation(
            two_seventy_degrees, rotation_range=[0, 180]
        )

        self.assertAlmostEqual(
            actual_zero.pose.orientation.x, zero_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.y, zero_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.z, zero_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.w, zero_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_ninety.pose.orientation.x, ninety_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.y, ninety_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.z, ninety_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.w, ninety_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.x, one_eighty_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.y, one_eighty_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.z, one_eighty_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.w, one_eighty_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.x, ninety_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.y, ninety_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.z, ninety_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.w, ninety_degrees.pose.orientation.w,
            places=5
        )

    def test_modify_pose_rotation_complete(self):
        """
        Tests that the 'modify_pose_rotation' function returns the same pose,
        but with a rotation offset and a limited rotation.

        """
        # test angles: different rotations around the Z axis
        zero_degrees = geometry_msgs.msg.PoseStamped()
        zero_degrees.pose.orientation.x = 0.0
        zero_degrees.pose.orientation.y = 0.0
        zero_degrees.pose.orientation.z = 0.0
        zero_degrees.pose.orientation.w = 1.0

        ninety_degrees = geometry_msgs.msg.PoseStamped()
        ninety_degrees.pose.orientation.x = 0.0
        ninety_degrees.pose.orientation.y = 0.0
        ninety_degrees.pose.orientation.z = math.cos(math.pi / 4)
        ninety_degrees.pose.orientation.w = math.cos(math.pi / 4)

        one_eighty_degrees = geometry_msgs.msg.PoseStamped()
        one_eighty_degrees.pose.orientation.x = 0.0
        one_eighty_degrees.pose.orientation.y = 0.0
        one_eighty_degrees.pose.orientation.z = 1.0
        one_eighty_degrees.pose.orientation.w = 0.0

        two_seventy_degrees = geometry_msgs.msg.PoseStamped()
        two_seventy_degrees.pose.orientation.x = 0.0
        two_seventy_degrees.pose.orientation.y = 0.0
        two_seventy_degrees.pose.orientation.z = math.cos(math.pi / 4)
        two_seventy_degrees.pose.orientation.w = -math.cos(math.pi / 4)

        actual_zero = pregrasp_planner_utils.modify_pose_rotation(
            zero_degrees, offset=90, rotation_range=[0, 180]
        )
        actual_ninety = pregrasp_planner_utils.modify_pose_rotation(
            ninety_degrees, offset=90, rotation_range=[0, 180]
        )
        actual_one_eighty = pregrasp_planner_utils.modify_pose_rotation(
            one_eighty_degrees, offset=90, rotation_range=[0, 180]
        )
        actual_two_seventy = pregrasp_planner_utils.modify_pose_rotation(
            two_seventy_degrees, offset=90, rotation_range=[0, 180]
        )

        self.assertAlmostEqual(
            actual_zero.pose.orientation.x, ninety_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.y, ninety_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.z, ninety_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_zero.pose.orientation.w, ninety_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_ninety.pose.orientation.x, one_eighty_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.y, one_eighty_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.z, one_eighty_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_ninety.pose.orientation.w, one_eighty_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.x, two_seventy_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.y, two_seventy_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.z, two_seventy_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_one_eighty.pose.orientation.w, two_seventy_degrees.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.x, one_eighty_degrees.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.y, one_eighty_degrees.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.z, one_eighty_degrees.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_two_seventy.pose.orientation.w, one_eighty_degrees.pose.orientation.w,
            places=5
        )

    def test_modify_pose_rotation_random_angles(self):
        """
        Tests that the 'modify_pose_rotation' function returns the same pose,
        but with a rotation offset and a limited rotation for a series of random
        angles.

        """
        # test angles: different rotations around the Z axis
        # 35 degrees
        angle_1 = geometry_msgs.msg.PoseStamped()
        angle_1.pose.orientation.x = 0.0
        angle_1.pose.orientation.y = 0.0
        angle_1.pose.orientation.z = 0.30071
        angle_1.pose.orientation.w = 0.95372

        # 120 degrees
        angle_2 = geometry_msgs.msg.PoseStamped()
        angle_2.pose.orientation.x = 0.0
        angle_2.pose.orientation.y = 0.0
        angle_2.pose.orientation.z = math.cos(math.pi / 6)
        angle_2.pose.orientation.w = 0.5

        # 250 degrees
        angle_3 = geometry_msgs.msg.PoseStamped()
        angle_3.pose.orientation.x = 0.0
        angle_3.pose.orientation.y = 0.0
        angle_3.pose.orientation.z = 0.81915
        angle_3.pose.orientation.w = -0.57358

        # 320 degrees
        angle_4 = geometry_msgs.msg.PoseStamped()
        angle_4.pose.orientation.x = 0.0
        angle_4.pose.orientation.y = 0.0
        angle_4.pose.orientation.z = 0.34202
        angle_4.pose.orientation.w = -0.93969

        actual_first = pregrasp_planner_utils.modify_pose_rotation(
            angle_1, offset=170, rotation_range=[0, 180]
        )
        actual_second = pregrasp_planner_utils.modify_pose_rotation(
            angle_2, offset=170, rotation_range=[0, 180]
        )
        actual_third = pregrasp_planner_utils.modify_pose_rotation(
            angle_3, offset=170, rotation_range=[0, 180]
        )
        actual_fourth = pregrasp_planner_utils.modify_pose_rotation(
            angle_4, offset=170, rotation_range=[0, 180]
        )

        # for these test angles: [35.0, 120.0, 250.0, 320.0],
        # the desired angles should be: [205.0, 290.0, 240.0, 310.0]
        # 205 degrees
        desired_1 = geometry_msgs.msg.PoseStamped()
        desired_1.pose.orientation.x = 0.0
        desired_1.pose.orientation.y = 0.0
        desired_1.pose.orientation.z = 0.9763
        desired_1.pose.orientation.w = -0.21644

        # 290 degrees
        desired_2 = geometry_msgs.msg.PoseStamped()
        desired_2.pose.orientation.x = 0.0
        desired_2.pose.orientation.y = 0.0
        desired_2.pose.orientation.z = 0.57358
        desired_2.pose.orientation.w = -0.81915

        # 240 degrees
        desired_3 = geometry_msgs.msg.PoseStamped()
        desired_3.pose.orientation.x = 0.0
        desired_3.pose.orientation.y = 0.0
        desired_3.pose.orientation.z = math.cos(math.pi / 6.0)
        desired_3.pose.orientation.w = -0.5

        # 310 degrees
        desired_4 = geometry_msgs.msg.PoseStamped()
        desired_4.pose.orientation.x = 0.0
        desired_4.pose.orientation.y = 0.0
        desired_4.pose.orientation.z = 0.42262
        desired_4.pose.orientation.w = -0.90631

        self.assertAlmostEqual(
            actual_first.pose.orientation.x, desired_1.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_first.pose.orientation.y, desired_1.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_first.pose.orientation.z, desired_1.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_first.pose.orientation.w, desired_1.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_second.pose.orientation.x, desired_2.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_second.pose.orientation.y, desired_2.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_second.pose.orientation.z, desired_2.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_second.pose.orientation.w, desired_2.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_third.pose.orientation.x, desired_3.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_third.pose.orientation.y, desired_3.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_third.pose.orientation.z, desired_3.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_third.pose.orientation.w, desired_3.pose.orientation.w,
            places=5
        )

        self.assertAlmostEqual(
            actual_fourth.pose.orientation.x, desired_4.pose.orientation.x,
            places=5
        )
        self.assertAlmostEqual(
            actual_fourth.pose.orientation.y, desired_4.pose.orientation.y,
            places=5
        )
        self.assertAlmostEqual(
            actual_fourth.pose.orientation.z, desired_4.pose.orientation.z,
            places=5
        )
        self.assertAlmostEqual(
            actual_fourth.pose.orientation.w, desired_4.pose.orientation.w,
            places=5
        )

    def test_restrict_angle_to_range_no_changes(self):
        """
        Tests that the 'restrict_angle_to_range' function returns the same angle, since
        there is no offset and rotation_range is set from 0-360 degrees.

        """
        offset = math.radians(0.0)
        rotation_range = numpy.radians([0.0, 360.0])
        test_angles = numpy.radians([0, 30, 45, 90, 100, 150, 180, 270, 300, 360])
        desired = numpy.radians([0, 30, 45, 90, 100, 150, 180, 270, 300, 0])
        actual = [
            pregrasp_planner_utils.restrict_angle_to_range(angle, offset, rotation_range)
            for angle in test_angles
        ]
        numpy.testing.assert_almost_equal(actual, desired)

    def test_restrict_angle_to_range_offset(self):
        """
        Tests that the 'restrict_angle_to_range' function returns the only angles
        between the specified range plus an offset. It also restricts the value
        to be within 360 degrees.

        """
        offset = math.radians(100.0)
        rotation_range = numpy.radians([0.0, 360.0])
        test_angles = numpy.radians([0, 30, 45, 90, 100, 150, 180, 270, 300, 360])
        desired = numpy.radians([180, 210, 225, 270, 100, 150, 180, 270, 300, 0])
        actual = [
            pregrasp_planner_utils.restrict_angle_to_range(angle, offset, rotation_range)
            for angle in test_angles
        ]
        numpy.testing.assert_almost_equal(actual, desired)

    def test_restrict_angle_to_range_rotation_easy_range(self):
        """
        Tests that the 'restrict_angle_to_range' function returns the only angles
        between the specified range. It also restricts the value
        to be within 360 degrees.

        """
        offset = math.radians(0.0)
        rotation_range = numpy.radians([0.0, 180.0])
        test_angles = numpy.radians([0, 30, 45, 90, 100, 150, 180, 270, 300, 360])
        desired = numpy.radians([0, 30, 45, 90, 100, 150, 180, 90, 120, 180])
        actual = [
            pregrasp_planner_utils.restrict_angle_to_range(angle, offset, rotation_range)
            for angle in test_angles
        ]
        numpy.testing.assert_almost_equal(actual, desired)

    def test_restrict_angle_to_range_rotation_inverted_range(self):
        """
        Tests that the 'restrict_angle_to_range' function returns the only angles
        between an 'inverted' range (i.e. the first value of the range is greater
        than the second). It also restricts the value to be within 360 degrees.

        """
        offset = math.radians(0.0)
        rotation_range = numpy.radians([270.0, 90.0])
        test_angles = numpy.radians([0, 30, 45, 90, 100, 150, 180, 270, 300, 360])
        desired = numpy.radians([0, 30, 45, 90, 280, 330, 0, 270, 300, 0])
        actual = [
            pregrasp_planner_utils.restrict_angle_to_range(angle, offset, rotation_range)
            for angle in test_angles
        ]
        numpy.testing.assert_almost_equal(actual, desired)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_simple_pregrasp_planner', TestSimplePregraspPlanner)
