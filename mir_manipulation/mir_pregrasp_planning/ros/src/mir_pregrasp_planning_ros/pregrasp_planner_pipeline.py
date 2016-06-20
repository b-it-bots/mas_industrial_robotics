#!/usr/bin/env python
"""
This component takes an object's pose (target pose) and depending on different
constraints (e.g. the object is standing or laying horizontally, object's height
is short, etc.), it computes a set of poses, within a threshold, to reach the
target pose with a manipulator's end effector. From this set of poses, this component
selects a valid pose (i.e. one that reaches the target pose) and it computes the
joint configuration to reach that valid pose.

It uses the following nodes:
  * (mir_pregrasp_planning) `simple_pregrasp_planner`.
  * (mcr_pose_generation) `pose_generator`.
  * (mcr_manipulation_pose_selector) `reachability_pose_selector`.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

**Assumptions:**
  * The target pose is within the reach of the robot's manipulator.

**Input(s):**
  * `target_pose`: The pose where the manipulator's end effector is required to be.

**Output(s):**
  * `selected_pose`: The pose of the manipulator's last joint so that the end effector
  reaches the `target_pose`.
  * `configuration_out`: The joint position values for the manipulator to reach
  the `selected_pose`.

**Relevant parameter(s):**
  * `simple_grasp_planner`
    * `height_threshold`: Tolerance to decide whether an object should be re-oriented,
    based on its height (in meters).
    * `rotation_offset`: Rotation offset to add to the reference_axis of the pose
    (in degrees).
    * `rotation_range`: Range of rotation allowed as a two-element list, e.g.:
    [rotation_range[0] - rotation_range[1]] (in degrees).
  * `pose_generator`
    * `gripper_matrix`: A 4x4 matrix that defines the pose of the end effector relative
    to the pose of the manipulator's last joint, i.e. this should be modified for
    different grippers.

"""
# -*- encoding: utf-8 -*-

import rospy
import std_msgs.msg


class PregraspPlannerPipeline(object):
    """
    Components that compute a joint configuration based on a target pose
    and a set of given constraints.

    """
    def __init__(self):
        # params
        self.started_components = False
        self.event = None
        self.pose_transformer_status = None
        self.pregrasp_planner_status = None
        self.pose_generator_status = None
        self.pose_selector_status = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.start_pose_transformer = rospy.Publisher(
            '~start_pose_transformer', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_pregrasp_planner = rospy.Publisher(
            '~start_pregrasp_planner', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_pose_generator = rospy.Publisher(
            '~start_pose_generator', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_pose_selector = rospy.Publisher(
            '~start_pose_selector', std_msgs.msg.String, latch=True, queue_size=1
        )

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~pose_transformer_status", std_msgs.msg.String, self.pose_transformer_status_cb
        )
        rospy.Subscriber(
            "~pregrasp_planner_status", std_msgs.msg.String, self.grasp_planner_status_cb
        )
        rospy.Subscriber(
            "~pose_generator_status", std_msgs.msg.String, self.pose_generator_status_cb
        )
        rospy.Subscriber(
            "~pose_selector_status", std_msgs.msg.String, self.pose_selector_status_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def pose_transformer_status_cb(self, msg):
        """
        Obtains the status of the pose transformer (as an event).

        """
        self.pose_transformer_status = msg.data

    def grasp_planner_status_cb(self, msg):
        """
        Obtains the status of the pregrasp planner (as an event).

        """
        self.pregrasp_planner_status = msg.data

    def pose_generator_status_cb(self, msg):
        """
        Obtains the status of the pose generator (as an event).

        """
        self.pose_generator_status = msg.data

    def pose_selector_status_cb(self, msg):
        """
        Obtains the status of the pose selector (as an event).

        """
        self.pose_selector_status = msg.data

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'RUNNING'
        else:
            return 'INIT'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        self.toggle_components(self.event)

        if self.event == 'e_stop':
            status = 'e_stopped'
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        if self.pregrasp_planner_status == 'e_failure' \
            or self.pose_generator_status == 'e_failure' \
                or self.pose_selector_status == 'e_failure'\
                or self.pose_transformer_status == 'e_failure':
            status = 'e_failure'
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        if self.pose_selector_status == 'e_success':
            status = 'e_success'
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        else:
            return 'RUNNING'

    def toggle_components(self, event):
        """
        Starts or stops the necessary components based on the event.

        :param event: The event that determines either to start or stop the components.
        :type event: str

        """
        if event == 'e_stopped' or event == 'e_failure' or event == 'e_success':
            self.start_pose_transformer.publish('e_stop')
            self.start_pregrasp_planner.publish('e_stop')
            self.start_pose_generator.publish('e_stop')
            self.start_pose_selector.publish('e_stop')
            self.started_components = False

        if event == 'e_start' and not self.started_components:
            self.start_pose_transformer.publish('e_start')
            self.start_pregrasp_planner.publish('e_start')
            self.start_pose_generator.publish('e_start')
            self.start_pose_selector.publish('e_start')
            self.started_components = True

    def reset_component_data(self, result):
        """
        Clears the data of the component.

        :param result: The result of the component, e.g. stopped, failure, success.
        :type result: str

        """
        self.toggle_components(result)
        self.event = None
        self.pregrasp_planner_status = None
        self.pose_generator_status = None
        self.pose_selector_status = None
        self.started_components = False


def main():
    rospy.init_node("pregrasp_planner_pipeline", anonymous=True)
    pregrasp_planner_pipeline = PregraspPlannerPipeline()
    pregrasp_planner_pipeline.start()
