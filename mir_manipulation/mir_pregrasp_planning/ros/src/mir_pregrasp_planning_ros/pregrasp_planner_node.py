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
  * `selected_pose`: The pose of the manipulator's last joint so that the end\
                     effector reaches the `target_pose`.
  * `configuration_out`: The joint position values for the manipulator to reach\
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

import brics_actuator.msg
import copy
import geometry_msgs.msg
import mcr_common_converters_ros.joint_configuration_shifter
import mcr_manipulation_measurers_ros.pose_transformer
import mcr_manipulation_msgs.msg
import mcr_manipulation_pose_selector_ros.reachability_pose_selector
import mcr_pose_generation_ros.pose_generator
import mir_pregrasp_planning.cfg.PregraspPlannerParamsConfig as PregraspPlannerParamsConfig
import mir_pregrasp_planning_ros.simple_pregrasp_planner_utils as pregrasp_planner_utils
from mir_pregrasp_planning_ros.orientation_independent_ik import OrientationIndependentIK
import rospy
import std_msgs.msg
from dynamic_reconfigure.server import Server


class PregraspPlannerPipeline(object):
    """
    Components that compute a joint configuration based on a target pose
    and a set of given constraints.

    """

    def __init__(self):
        self.event = None
        self.pose_in = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param("~loop_rate", 10.0))
        # pose transformer
        self.target_frame = None

        # simple pregrasp planner params
        # Tolerance to decide whether an object should be re-oriented,
        # based on its height (in meters).
        self.height_tolerance = None

        # Rotation axis of the pose to be modified (e.g. x, y, z).
        self.reference_axis = None
        # Rotation offset to add to the reference_axis of the pose (in degrees).
        self.rotation_offset = None
        # Range of rotation allowed [rotation_range[0] - rotation_range[1]] (in degrees).
        self.rotation_range = None

        # Closest distance the gripper should be to the object (in meters).
        self.min_distance_to_object = None
        # Farthest distance the gripper should be to the object (in meters).
        self.max_distance_to_object = None

        # Angular tolerance to check if an object is standing up (in degrees).
        self.angular_tolerance = None

        # reachability pose selector params
        # linear offset for the X, Y and Z axis.
        self.linear_offset = None
        # whether a pre-pregrasp pose should be created
        self.generate_pregrasp_waypoint = None
        # joint offset for creating pre-pregrasp pose
        self.joint_offset = None

        # pose generator
        self.gripper = rospy.get_param("~gripper_config_matrix", None)
        assert self.gripper is not None, "Gripper config matrix must be specified."
        # Configuration matrix of the gripper to be used (a real 4x4 matrix)
        self.gripper_config_matrix = rospy.get_param("~" + self.gripper)

        # the sampling step for linear variables (in meters)
        self.linear_step = None
        # the sampling step for angular variables (in radians)
        self.angular_step = None
        # the maximum amount of samples to be generated
        self.max_samples = None

        self.pose_transformer = (
            mcr_manipulation_measurers_ros.pose_transformer.PoseTransformer()
        )
        self.pose_generator = mcr_pose_generation_ros.pose_generator.PoseGenerator()
        self.reachability_pose_selector = (
            mcr_manipulation_pose_selector_ros.reachability_pose_selector.PoseSelector()
        )
        self.joint_config_shifter = mcr_common_converters_ros.joint_configuration_shifter.JointConfigurationShifter(
            None
        )

        self.pose_generator.set_gripper_config_matrix(self.gripper_config_matrix)

        self.orientation_independent_ik = OrientationIndependentIK(debug=False)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~pose_in", geometry_msgs.msg.PoseStamped, self.pose_cb)

        # publishers
        self.event_out = rospy.Publisher(
            "~event_out", std_msgs.msg.String, queue_size=1
        )
        self.grasp_type = rospy.Publisher(
            "~grasp_type", std_msgs.msg.String, queue_size=1
        )
        self.event_out = rospy.Publisher(
            "~event_out", std_msgs.msg.String, queue_size=1
        )
        self.pose_samples_pub = rospy.Publisher(
            "~pose_samples", geometry_msgs.msg.PoseArray, queue_size=1
        )
        self.selected_pose = rospy.Publisher(
            "~selected_pose", geometry_msgs.msg.PoseStamped, queue_size=1
        )
        self.joint_configuration = rospy.Publisher(
            "~joint_configuration", brics_actuator.msg.JointPositions, queue_size=1
        )

        self.joint_waypoint_list_pub = rospy.Publisher(
            "~waypoint_list",
            mcr_manipulation_msgs.msg.JointSpaceWayPointsList,
            queue_size=1,
        )

        # Dynamic reconguration server for PregraspPlannerParams
        dynamic_reconfig_srv = Server(
            PregraspPlannerParamsConfig, self.dynamic_reconfig_cb
        )

    def dynamic_reconfig_cb(self, config, level):
        """
        Dynamic reconfiguration callback function
        """
        self.pose_generator.set_min_azimuth(math.radians(config.min_azimuth))
        self.pose_generator.set_max_azimuth(math.radians(config.max_azimuth))
        self.pose_generator.set_min_zenith(math.radians(config.min_zenith))
        self.pose_generator.set_max_zenith(math.radians(config.max_zenith))
        self.pose_generator.set_min_roll(math.radians(config.min_roll))
        self.pose_generator.set_max_roll(math.radians(config.max_roll))
        self.pose_generator.set_linear_step(config.linear_step)
        self.pose_generator.set_angular_step(math.radians(config.angular_step))
        self.pose_generator.set_min_distance_to_object(config.min_distance_to_object)
        self.pose_generator.set_max_distance_to_object(config.max_distance_to_object)
        self.pose_generator.set_max_samples(config.max_samples)
        self.pose_generator.set_min_height(config.min_height)
        self.pose_generator.set_max_height(config.max_height)

        self.height_tolerance = config.height_tolerance
        self.target_frame = config.target_frame
        self.reference_axis = config.reference_axis
        self.rotation_offset = config.rotation_offset
        self.angular_tolerance = config.angular_tolerance
        self.linear_offset = [
            config.linear_offset_x,
            config.linear_offset_y,
            config.linear_offset_z,
        ]
        self.generate_pregrasp_waypoint = config.generate_pregrasp_waypoint
        self.ignore_orientation = config.ignore_orientation
        self.joint_offset = [
            config.joint_1_offset,
            config.joint_2_offset,
            config.joint_3_offset,
            config.joint_4_offset,
            config.joint_5_offset,
        ]
        self.joint_offset_side_grasp = [
            config.joint_1_offset_side_grasp,
            config.joint_2_offset_side_grasp,
            config.joint_3_offset_side_grasp,
            config.joint_4_offset_side_grasp,
            config.joint_5_offset_side_grasp,
        ]
        self.rotation_range = [config.rotation_range_min, config.rotation_range_max]
        return config

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
        state = "INIT"

        while not rospy.is_shutdown():

            if state == "INIT":
                state = self.init_state()
            elif state == "IDLE":
                state = self.idle_state()
            elif state == "RUNNING":
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == "e_start":
            return "IDLE"
        else:
            return "INIT"

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == "e_stop":
            self.reset_component_data()
            self.event_out.publish("e_stopped")
            return "INIT"
        elif self.pose_in:
            return "RUNNING"
        else:
            return "IDLE"

    def is_graspable(self):
        # gripper_width = 0.1
        # obj_width = rospy.get_param("mir_perception/object_width")
        # if obj_width < (gripper_width - 0.02):
        #     return True
        # else:
        #     return False
        return True

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        if self.event == "e_stop":
            status = "e_stopped"
            self.event_out.publish(status)
            self.reset_component_data()
            return "INIT"

        # 1. transform pose to target frame
        # 2. check if pose is for laying or standing object and modify accordingly
        # 3. generate pose samples around pose
        # 4. find a reachable pose from the generated samples (also get joint config for reachable pose)
        # 5. if joint waypoint list is required publish list of waypoints

        transformed_pose = self.pose_transformer.get_transformed_pose(
            self.pose_in, self.target_frame
        )
        if not transformed_pose:
            rospy.logerr("Unable to transform pose to {0}".format(self.target_frame))
            status = "e_failure"
            self.event_out.publish(status)
            self.reset_component_data()
            return "INIT"
        rospy.loginfo('[Pregrasp Planning] Tying to find solution for default pick config.')

        modified_pose, object_is_upwards = pregrasp_planner_utils.modify_pose(
            transformed_pose,
            self.height_tolerance,
            angular_tolerance=self.angular_tolerance,
        )
        if not object_is_upwards:
            rotated_pose = pregrasp_planner_utils.modify_pose_rotation(
                modified_pose,
                offset=self.rotation_offset,
                reference_axis=self.reference_axis,
                rotation_range=self.rotation_range,
            )
            modified_pose = rotated_pose
            grasp_type = "top_grasp"
        else:
            grasp_type = "side_grasp"

        if grasp_type == "side_grasp":
            modified_pose.pose.position.y += 0.02
            modified_pose.pose.position.x -= 0.01

        self.grasp_type.publish(grasp_type)
        pose_samples = self.pose_generator.calculate_poses_list(modified_pose)
        self.pose_samples_pub.publish(pose_samples)
        (
            reachable_pose,
            brics_joint_config,
            joint_config,
        ) = self.reachability_pose_selector.get_reachable_pose_and_configuration(
            pose_samples, self.linear_offset, modified_pose
        )

        if not reachable_pose and self.is_graspable():
            rospy.logerr("[Pregrasp Planning] Could not find IK solution for default pick config.")
            rospy.loginfo('\033[92m'+"[Pregrasp Planning] Tryinng orientation independent planning.")
            input_pose = geometry_msgs.msg.PoseStamped()
            input_pose.header = transformed_pose.header
            input_pose.pose.position = transformed_pose.pose.position
            if grasp_type == "side_grasp":
                # input_pose.pose.position.y -= 0.01
                input_pose.pose.position.x -= 0.01
            solution = self.orientation_independent_ik.get_reachable_pose_and_joint_msg_from_point(
                    input_pose.pose.position.x, input_pose.pose.position.y,
                    input_pose.pose.position.z, input_pose.header.frame_id)

            if solution is None:
                rospy.logerr("[Pregrasp Planning] Could not find IK solution for orientation independent planning.")
                status = 'e_failure'
                self.event_out.publish(status)
                self.reset_component_data()
                return 'INIT'

            reachable_pose, joint_msg = solution
            rospy.loginfo('[Pregrasp Planning] Found solution')
            self.selected_pose.publish(reachable_pose)
            self.joint_configuration.publish(joint_msg)

            joint_waypoints = mcr_manipulation_msgs.msg.JointSpaceWayPointsList()
            joint_config = [p.value for p in joint_msg.positions]
            if grasp_type == "side_grasp" and self.generate_pregrasp_waypoint:
                pregrasp_input_pose = copy.deepcopy(input_pose)
                pregrasp_input_pose.pose.position.x -= 0.05
                pregrasp_solution = self.orientation_independent_ik.get_reachable_pose_and_joint_msg_from_point(
                        pregrasp_input_pose.pose.position.x,
                        pregrasp_input_pose.pose.position.y,
                        pregrasp_input_pose.pose.position.z,
                        pregrasp_input_pose.header.frame_id)

                if pregrasp_solution is None:
                    rospy.logerr("[Pregrasp Planning] Could not find IK solution for\
                            orientation independent planning for pregrasp pose.")
                    status = 'e_failure'
                    self.event_out.publish(status)
                    self.reset_component_data()
                    return 'INIT'
                pregrasp_reachable_pose, pregrasp_joint_msg = pregrasp_solution
                rospy.loginfo('[Pregrasp Planning] Found solution pregrasp')
                pregrasp_msg = std_msgs.msg.Float64MultiArray()
                pregrasp_joint_config = [p.value for p in pregrasp_joint_msg.positions]
                pregrasp_msg.data = pregrasp_joint_config
                joint_waypoints.list_of_joint_values_lists.append(pregrasp_msg)

            if len(joint_config) == 5:
                grasp_msg = std_msgs.msg.Float64MultiArray()
                grasp_msg.data = joint_config
                joint_waypoints.list_of_joint_values_lists.append(grasp_msg)
                self.joint_waypoint_list_pub.publish(joint_waypoints)

            self.event_out.publish("e_success")
            self.reset_component_data()
            return 'INIT'

        rospy.loginfo('[Pregrasp Planning] Found solution')

        self.selected_pose.publish(reachable_pose)
        self.joint_configuration.publish(brics_joint_config)

        joint_waypoints = mcr_manipulation_msgs.msg.JointSpaceWayPointsList()
        # if we want to reach a pre-pregrasp pose (specified by joint offsets)
        # before reaching the final pose, generate a waypoint list
        if abs(max(self.joint_offset, key=abs)) > 0 and self.generate_pregrasp_waypoint:
            if grasp_type == "side_grasp":
                pregrasp_waypoint = self.joint_config_shifter.shift_joint_configuration(
                    joint_config, self.joint_offset_side_grasp
                )
            elif grasp_type == "top_grasp":
                pregrasp_waypoint = self.joint_config_shifter.shift_joint_configuration(
                    joint_config, self.joint_offset
                )
            pregrasp_msg = std_msgs.msg.Float64MultiArray()
            pregrasp_msg.data = pregrasp_waypoint
            joint_waypoints.list_of_joint_values_lists.append(pregrasp_msg)

        grasp_msg = std_msgs.msg.Float64MultiArray()
        grasp_msg.data = joint_config

        joint_waypoints.list_of_joint_values_lists.append(grasp_msg)
        self.joint_waypoint_list_pub.publish(joint_waypoints)

        self.event_out.publish("e_success")
        self.reset_component_data()
        return "INIT"

    def reset_component_data(self):
        self.pose_in = None
        self.event = None


def main():
    rospy.init_node("pregrasp_planner_pipeline", anonymous=True)
    pregrasp_planner_pipeline = PregraspPlannerPipeline()
    pregrasp_planner_pipeline.start()
