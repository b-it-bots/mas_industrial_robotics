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

import math
import rospy
import std_msgs.msg
import geometry_msgs.msg
import brics_actuator.msg
import mcr_manipulation_measurers_ros.pose_transformer_class
import mir_pregrasp_planning_ros.simple_pregrasp_planner_utils as pregrasp_planner_utils
import mcr_pose_generation_ros.pose_generator_class
import mcr_manipulation_pose_selector_ros.reachability_pose_selector_class
import mcr_manipulation_msgs.msg



class PregraspPlannerPipeline(object):
    """
    Components that compute a joint configuration based on a target pose
    and a set of given constraints.

    """
    def __init__(self):
        self.event = None
        self.pose_in = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # pose transformer
        self.target_frame = rospy.get_param('~target_frame', 'base_link')

        # simple pregrasp planner params
        # Tolerance to decide whether an object should be re-oriented,
        # based on its height (in meters).
        self.height_tolerance = rospy.get_param('~height_tolerance', 0.04)

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

        # Angular tolerance to check if an object is standing up (in degrees).
        self.angular_tolerance = rospy.get_param('~angular_tolerance', 2.0)

        # reachability pose selector params
        # linear offset for the X, Y and Z axis.
        self.linear_offset = rospy.get_param('~linear_offset', None)


        # somewhere
        self.min_azimuth = -3.0
        self.max_azimuth = 3.0
        self.min_zenith = -3.0
        self.max_zenith = 3.0
        self.min_roll = 0.0
        self.max_roll = 0.0

        # pose generator
        self.gripper = rospy.get_param('~gripper_config_matrix', None)
        assert self.gripper is not None, "Gripper config matrix must be specified."
        # Configuration matrix of the gripper to be used (a real 4x4 matrix)
        self.gripper_config_matrix = rospy.get_param('~' + self.gripper)

        # the sampling step for linear variables (in meters)
        self.linear_step = rospy.get_param('~linear_step', 0.01)
        # the sampling step for angular variables (in radians)
        self.angular_step = rospy.get_param('~angular_step', 0.5)
        # the maximum amount of samples to be generated
        self.max_samples = rospy.get_param('~max_samples', 50)



        self.pose_transformer = mcr_manipulation_measurers_ros.pose_transformer_class.PoseTransformer()
        self.pose_generator = mcr_pose_generation_ros.pose_generator_class.PoseGenerator()
        self.reachability_pose_selector = mcr_manipulation_pose_selector_ros.reachability_pose_selector_class.PoseSelector()


        self.pose_generator.set_linear_step(self.linear_step)
        self.pose_generator.set_angular_step(self.angular_step)
        self.pose_generator.set_max_samples(self.max_samples)
        self.pose_generator.set_gripper_config_matrix(self.gripper_config_matrix)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~pose_in', geometry_msgs.msg.PoseStamped, self.pose_cb)

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.grasp_type = rospy.Publisher('~grasp_type', std_msgs.msg.String, queue_size=1)
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=1)
        self.selected_pose = rospy.Publisher(
            "~selected_pose", geometry_msgs.msg.PoseStamped, queue_size=1
        )
        self.joint_configuration = rospy.Publisher(
            "~joint_configuration", brics_actuator.msg.JointPositions, queue_size=1
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def pose_cb(self, msg):
        """
        Obtains input pose
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
            status = 'e_stopped'
            self.event_out.publish(status)
            self.reset_component_data()
            return 'INIT'

        transformed_pose = self.pose_transformer.get_transformed_pose(self.pose_in, self.target_frame);
        if not transformed_pose:
            rospy.logerr("Unable to transform pose to {0}".format(self.target_frame))
            status = 'e_failure'
            self.event_out.publish(status)
            self.reset_component_data()
            return 'INIT'

        modified_pose, object_is_upwards = pregrasp_planner_utils.modify_pose(
                transformed_pose, self.height_tolerance, angular_tolerance=self.angular_tolerance
                )
        if not object_is_upwards:
            rotated_pose = pregrasp_planner_utils.modify_pose_rotation(
                modified_pose, offset=self.rotation_offset,
                reference_axis=self.reference_axis, rotation_range=self.rotation_range
            )
            modified_pose = rotated_pose
            grasp_type = 'top_grasp'
        else:
            grasp_type = 'side_grasp'

        sampling_parameters = mcr_manipulation_msgs.msg.SphericalSamplerParameters()
        sampling_parameters.radial_distance.minimum = self.min_distance_to_object
        sampling_parameters.radial_distance.maximum = self.max_distance_to_object
        sampling_parameters.azimuth.minimum = math.radians(self.min_azimuth)
        sampling_parameters.azimuth.maximum = math.radians(self.max_azimuth)
        sampling_parameters.zenith.minimum = math.radians(self.min_zenith)
        sampling_parameters.zenith.maximum = math.radians(self.max_zenith)
        sampling_parameters.yaw.minimum = math.radians(self.min_roll)
        sampling_parameters.yaw.maximum = math.radians(self.max_roll)
        print sampling_parameters

        pose_samples = self.pose_generator.calculate_poses_list(modified_pose, sampling_parameters)
        print pose_samples
        reachable_pose, reachable_configuration = self.reachability_pose_selector.get_reachable_pose_and_configuration(pose_samples, self.linear_offset)


        if not reachable_pose:
            rospy.logerr("Could not find IK solution")
            status = 'e_failure'
            self.event_out.publish(status)
            self.reset_component_data()
            return 'INIT'

        self.selected_pose.publish(reachable_pose)
        self.joint_configuration.publish(reachable_configuration)
        self.event_out.publish("e_success")
        self.reset_component_data()
        return 'INIT'

    def reset_component_data(self):
        self.pose_in = None
        self.event = None



def main():
    rospy.init_node("pregrasp_planner_pipeline", anonymous=True)
    pregrasp_planner_pipeline = PregraspPlannerPipeline()
    pregrasp_planner_pipeline.start()
