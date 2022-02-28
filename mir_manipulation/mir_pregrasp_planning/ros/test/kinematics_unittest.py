#!/usr/bin/env python

import random
import unittest
from mir_pregrasp_planning_ros.kinematics import Kinematics

PKG = 'mir_pregrasp_planning'

class TestKinematics(unittest.TestCase):
    """
    Tests functions used in the simple_pregrasp_planner_utils.py module.

    """

    @classmethod
    def setUpClass(cls):
        """
        :returns: TODO

        """
        cls.kinematics = Kinematics()
        cls.angle_threshold = 0.05
        cls.pose_linear_threshold = 0.01
        cls.pose_quaternion_threshold = 0.01

    def test_fk_ik_look_at_workspace_from_near_left(self):
        # look at workspace from near left
        expected_solution = [ 1.4703, 0.4803, -0.6987, 3.0959, 0.6047]
        fk_pose = self.kinematics.forward_kinematics(expected_solution)

        # print(fk_pose)
        ik_solution = self.kinematics.inverse_kinematics(fk_pose)
        self.assertIsNotNone(ik_solution)
        self.assertTrue(self._is_joint_values_equivalent(ik_solution, expected_solution))

    def test_fk_ik_fk_random_joint_pos_100_times(self):
        successes = 0
        for i in range(100):
            expected_solution = [0.0]*5
            for i in range(5):
                r = random.random()
                expected_solution[i] = self.kinematics.lower_limit[i] \
                                       + r * (self.kinematics.upper_limit[i] \
                                              - self.kinematics.lower_limit[i])
            fk_pose = self.kinematics.forward_kinematics(expected_solution)

            # print(fk_pose)
            ik_solution = self.kinematics.inverse_kinematics(fk_pose)
            if ik_solution is not None:
                fk_pose_ik_solution = self.kinematics.forward_kinematics(ik_solution)
                try:
                    self._assert_pose_equivalent(fk_pose, fk_pose_ik_solution)
                    successes += 1
                except Exception as e:
                    pass

        self.assertGreaterEqual(successes, 80)

    def _is_joint_values_equivalent(self, joint_values_1, joint_values_2):
        equality = [abs(joint_values_1[i] - joint_values_2[i]) < self.angle_threshold for i in range(5)]
        return False not in equality

    def _assert_pose_equivalent(self, pose_1, pose_2):
        self.assertLessEqual(abs(pose_1.position.x - pose_2.position.x), self.pose_linear_threshold)
        self.assertLessEqual(abs(pose_1.position.y - pose_2.position.y), self.pose_linear_threshold)
        self.assertLessEqual(abs(pose_1.position.z - pose_2.position.z), self.pose_linear_threshold)
        self.assertLessEqual(abs(pose_1.orientation.x - pose_2.orientation.x), self.pose_quaternion_threshold)
        self.assertLessEqual(abs(pose_1.orientation.y - pose_2.orientation.y), self.pose_quaternion_threshold)
        self.assertLessEqual(abs(pose_1.orientation.z - pose_2.orientation.z), self.pose_quaternion_threshold)
        self.assertLessEqual(abs(pose_1.orientation.w - pose_2.orientation.w), self.pose_quaternion_threshold)


if __name__ == '__main__':
    unittest.main()
