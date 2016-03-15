#!/usr/bin/env python
"""
# Description
This package contains components to compute a *pre-grasp* pose, based on a target pose,
for the end effector of a robot's manipulator.

## 'simple_pregrasp_planner' node:
This component computes a modified pose based on the constraints imposed by
the youBot's manipulator and gripper. Namely, the manipulator cannot reach
arbitrary 6D poses due to having only 5 degrees of freedom; furthermore, the
gripper size doesn't allow for side grasps when the object to be grasped has
a small height (defined as a height lower than the 'height_threshold').

The pose is modified further if the object is not standing up (or if it is
standing but has a 'small' height). This last modification is, for a single
rotation axis (reference_axis), by adding an offset and limiting the rotation
to be within certain range. If no rotation_range is specified, the pose is not
modified any further (except for the specified offset).

**Assumptions:**
  * The object's pose has the X axis pointing up, when the object is standing on
  a surface.

**Input(s):**
  * `pose_in`: The target pose from which to base the calculation of the *pre-grasp*
  pose.

**Output(s):**
  * `pose_out`: The *pre-grasp* pose.
  * `sampling_parameters`: A message specifying the parameters, and constraints,
  of the pose to be sampled around an object, if any.
  * `grasp_type`: The type of grasp selected given a particular `pose_in`,
  e.g. top grasp, side grasp.

**Parameter(s):**
  * `height_threshold`: Tolerance to decide whether an object should be re-oriented,
  based on its height (in meters).
  * `reference_axis`: Rotation axis of the pose to be modified (e.g. x, y, z).
  * `rotation_offset`: Rotation offset to add to the reference_axis of the pose
  (in degrees).
  * `rotation_range`: Range of rotation allowed as a two-element list, e.g.:
  [rotation_range[0] - rotation_range[1]] (in degrees).
  The rotation range might be specified with its first value (minimum)
  greater than the second value (maximum) to cover a range within the
  circle that passes through zero (e.g.: The rotation range of [270- 90] covers
  from 270 to 359 and then from 0 to 90).
  * `min_distance_to_object`: Closest distance the gripper should be to the object
  (in meters).
  * `max_distance_to_object`: Farthest distance the gripper should be to the object
  (in meters).
  * `loop_rate`: Node cycle rate (in hz).

"""
# -*- encoding: utf-8 -*-

import math
import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import mir_pregrasp_planning_ros.simple_pregrasp_planner_utils as pregrasp_planner_utils
from dynamic_reconfigure.server import Server
import mir_pregrasp_planning.cfg.SamplingAngleParamsConfig as AngleConfig


class PregraspPlanner(object):
    """
    Computes a modified pose based on the constraints imposed
    by the youBot's manipulator and gripper.

    """
    def __init__(self):
        # Params
        self.event = None
        self.pose_in = None

        # Tolerance to decide whether an object should be re-oriented,
        # based on its height (in meters).
        self.height_tolerance = rospy.get_param('~height_tolerance', 0.15)

        # Rotation axis of the pose to be modified (e.g. x, y, z).
        self.reference_axis = rospy.get_param('~reference_axis', 'z')
        # Rotation offset to add to the reference_axis of the pose (in degrees).
        self.rotation_offset = rospy.get_param('~rotation_offset', 0.0)
        # Range of rotation allowed [rotation_range[0] - rotation_range[1]] (in degrees).
        self.rotation_range = rospy.get_param('~rotation_range', None)
        if self.rotation_range is not None:
            assert (type(self.rotation_range) == list) and (len(self.rotation_range) == 2),\
                "Rotation range must a list of two elements."

        # Closest distance the gripper should be to the object (in meters).
        self.min_distance_to_object = rospy.get_param('~min_distance_to_object', 0.0)
        # Farthest distance the gripper should be to the object (in meters).
        self.max_distance_to_object = rospy.get_param('~max_distance_to_object', 0.0)

        # Sampling parameters (in degrees) from dynamic reconfiguration server.
        dynamic_reconfig_srv = Server(AngleConfig, self.dynamic_reconfig_cb)

        # Angular tolerance to check if an object is standing up (in degrees).
        self.angular_tolerance = rospy.get_param('~angular_tolerance', 2.0)

        # Node cycle rate (in hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # Publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=1)
        self.pose_out = rospy.Publisher('~pose_out', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.sampling_parameters = rospy.Publisher(
            '~sampling_parameters', mcr_manipulation_msgs.msg.SphericalSamplerParameters, queue_size=1
        )
        self.grasp_type = rospy.Publisher('~grasp_type', std_msgs.msg.String, queue_size=1)

        # Subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~pose_in', geometry_msgs.msg.PoseStamped, self.pose_in_cb)

    def dynamic_reconfig_cb(self, config, level):
        rospy.loginfo("""Reconfigure Request: {min_azimuth}, {min_zenith},\
            {min_roll}, {max_azimuth}, {max_zenith}, {max_roll}""".format(**config))
        self.min_azimuth = config.min_azimuth
        self.max_azimuth = config.max_azimuth
        self.min_zenith = config.min_zenith
        self.max_zenith = config.max_zenith
        self.min_roll = config.min_roll
        self.max_roll = config.max_roll
        return config

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def pose_in_cb(self, msg):
        """
        Obtains the target pose.

        """
        self.pose_in = msg

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
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
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        elif self.pose_in:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            self.publish_component_outputs()
            self.reset_component_data()
            return 'IDLE'

    def publish_component_outputs(self):
        """
        Publishes the component's outputs.

        """
        modified_pose, object_is_upwards = pregrasp_planner_utils.modify_pose(
            self.pose_in, self.height_tolerance, angular_tolerance=self.angular_tolerance
        )

        sampling_parameters = mcr_manipulation_msgs.msg.SphericalSamplerParameters()
        sampling_parameters.radial_distance.minimum = self.min_distance_to_object
        sampling_parameters.radial_distance.maximum = self.max_distance_to_object
        sampling_parameters.azimuth.minimum = math.radians(self.min_azimuth)
        sampling_parameters.azimuth.maximum = math.radians(self.max_azimuth)
        sampling_parameters.zenith.minimum = math.radians(self.min_zenith)
        sampling_parameters.zenith.maximum = math.radians(self.max_zenith)
        sampling_parameters.yaw.minimum = math.radians(self.min_roll)
        sampling_parameters.yaw.maximum = math.radians(self.max_roll)

        if object_is_upwards:
            self.pose_out.publish(modified_pose)
            self.sampling_parameters.publish(sampling_parameters)
            self.grasp_type.publish('side_grasp')
            self.event_out.publish('e_success')
        else:
            # the object is laying down, thus the rotation will be restricted
            # to only 180 degrees (e.g. top grasp)
            rotated_pose = pregrasp_planner_utils.modify_pose_rotation(
                modified_pose, offset=self.rotation_offset,
                reference_axis=self.reference_axis, rotation_range=self.rotation_range
            )
            if rotated_pose:
                self.pose_out.publish(rotated_pose)
                self.sampling_parameters.publish(sampling_parameters)
                self.grasp_type.publish('top_grasp')
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.pose_in = None


def main():
    rospy.init_node('pregrasp_planner', anonymous=True)
    pregrasp_planner = PregraspPlanner()
    pregrasp_planner.start()
