#!/usr/bin/env python
"""
Integration test for the 'object_perception_functionality' node.

"""

import rospy
import numpy.testing as testing
import unittest
import rostest
import std_msgs.msg
import at_work_robot_example_ros.msg


PKG = 'mir_object_perception_functionality'


class TestObjectPerceptionFunctionality(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None
        self.log_msg = None
        self.event_in_msg = None
        self.wait_for_event = None

        # publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String)
        self.benchmark_state = rospy.Publisher(
            '~benchmark_state', at_work_robot_example_ros.msg.BenchmarkState
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', at_work_robot_example_ros.msg.BenchmarkFeedback, self.result_callback
        )

        self.logging_output = rospy.Subscriber(
            '~logging_output', std_msgs.msg.String, self.logging_callback
        )

        self.event_in = rospy.Subscriber(
            '~event_in', std_msgs.msg.String, self.event_in_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.benchmark_state.unregister()
        self.component_output.unregister()

    def test_object_perception_functionality(self):
        """
        Verifies that the state machine works as expected

        """
        benchmark_state_msg = at_work_robot_example_ros.msg.BenchmarkState()
        benchmark_state_msg.state.data = at_work_robot_example_ros.msg.BenchmarkState.RUNNING

        while not self.wait_for_result:
            self.event_out.publish('e_trigger')
            self.benchmark_state.publish(benchmark_state_msg)

        self.assertEqual(self.result.object_instance_name.data, "EM-01")
        self.assertEqual(self.result.object_class_name.data, "Containers")

        expected = "Containers EM-01 0.52 -0.2 0.0"
        self.assertEqual(self.log_msg.data, expected)

        self.result = None
        self.wait_for_result = False
        self.log_msg = None

        rospy.sleep(1.0)

        while not self.wait_for_result:
            self.benchmark_state.publish(benchmark_state_msg)

        self.assertEqual(self.result.object_instance_name.data, "EM-01")
        self.assertEqual(self.result.object_class_name.data, "Containers")

        expected = "Containers EM-01 0.52 -0.2 0.0"
        self.assertEqual(self.log_msg.data, expected)


        benchmark_state_msg.state.data = at_work_robot_example_ros.msg.BenchmarkState.FINISHED

        while not self.wait_for_event:
            self.benchmark_state.publish(benchmark_state_msg)

        self.assertEqual(self.event_in_msg.data, "e_done")

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True

    def logging_callback(self, msg):
        self.log_msg = msg

    def event_in_callback(self, msg):
        self.wait_for_event = True
        self.event_in_msg = msg


if __name__ == '__main__':
    rospy.init_node('object_perception_functionality_test')
    rostest.rosrun(PKG, 'object_perception_functionality_test', TestObjectPerceptionFunctionality)

