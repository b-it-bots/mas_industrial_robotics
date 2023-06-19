#!/usr/bin/python

import json
import math
import re

import brics_actuator.msg
import geometry_msgs.msg
import rospy
import smach
import smach_ros
import std_msgs.msg
import std_srvs.srv

from mas_perception_msgs.msg import ObjectList, Object
from mir_pregrasp_planning_ros.orientation_independent_ik import OrientationIndependentIK
import tf
from tf.transformations import euler_from_quaternion
from mir_manipulation_msgs.msg import GripperCommand
from std_msgs.msg import String
from brics_actuator.msg import JointPositions, JointValue
from srdfdom.srdf import SRDF
from sensor_msgs.msg import JointState

class Bunch:
    def __init__(self, **kwds):
        self.__dict__.update(kwds)


class MoveitClient:

    """
    Move arm to a target. target may be fixed at construction time or set
    through userdata.

    :param move_arm_to: target where the arm should move. If it is a string, then it gives
        target name (should be availabile on the parameter server). If it as
        tuple or a list, then it is treated differently based on the length. If it
        has 7 items, then it is cartesian pose (x, y, z, r, p ,y) + the corresponding frame.
        If it has 5 items, then it is arm configuration in join space.
    :type move_arm_to: str | tuple | list

    """

    def __init__(self, moveit_group=None, target=None, timeout=10.0, joint_names=None):

        TOPIC_EVENT_IN = "moveit_client/event_in"
        TOPIC_EVENT_OUT = "moveit_client/event_out"
        TOPIC_TARGET_STRING_POSE = "moveit_client/target_string_pose"
        TOPIC_TARGET_POSE = "moveit_client/target_pose"
        TOPIC_TARGET_CONFIGURATION = "moveit_client/target_configuration"

        self.move_arm_to = target

        self._point_ik_solver = OrientationIndependentIK()

        # Eventin publisher
        self.pub_event = rospy.Publisher(
            moveit_group + TOPIC_EVENT_IN, std_msgs.msg.String, queue_size=1
        )

        # Target configuration publishers
        self.pub_target_config_name = rospy.Publisher(
            moveit_group + TOPIC_TARGET_STRING_POSE, std_msgs.msg.String, queue_size=1
        )
        self.pub_target_pose = rospy.Publisher(
            moveit_group + TOPIC_TARGET_POSE,
            geometry_msgs.msg.PoseStamped,
            queue_size=1,
        )
        self.pub_target_config = rospy.Publisher(
            moveit_group + TOPIC_TARGET_CONFIGURATION,
            brics_actuator.msg.JointPositions,
            queue_size=1,
        )

        # Eventout subscriber
        self.sub_event = rospy.Subscriber(
            moveit_group + TOPIC_EVENT_OUT, std_msgs.msg.String, self.event_cb
        )

        self.client_event = None
        self.timeout = timeout
        self.joint_names = joint_names
        self.moveit_group = moveit_group

    def event_cb(self, msg):
        self.client_event = msg.data

    def execute(self, userdata, blocking=True):
        target = self.move_arm_to or userdata.move_arm_to

        # do it twice because it probably fails the first time
        for i in range(2):
            self.client_event = None
            self.pub_event.publish("e_start")

            if type(target) is str:  # target is a string specifing a joint position
                rospy.loginfo("MOVING ARM TO: " + str(target))
                self.pub_target_config_name.publish(target)

            elif type(target) is list:  # target is a list ...
                if len(target) == 7:  # ... of 7 items: Cartesian pose (x, y, z, r, p, y, frame_id)
                    pose = geometry_msgs.msg.PoseStamped()
                    pose.header.frame_id = target[6]
                    pose.pose.position.x = float(target[0])
                    pose.pose.position.y = float(target[1])
                    pose.pose.position.z = float(target[2])

                    q = tf.transformations.quaternion_from_euler(
                        target[3], target[4], target[5]
                    )
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]

                    self.pub_target_pose.publish(pose)
                elif len(target) == 5:      # ... of 5 items: Joint space configuration
                    self.pub_target_config.publish(self.list_to_brics_joint_positions(target))
                elif len(target) == 4:      # ... of 3 items: Cartesian point (x, y, z, frame_id)
                    brics_joint_pos_msg = self._point_ik_solver.get_joint_msg_from_point(*target)
                    if brics_joint_pos_msg is None:
                        rospy.logerr("Could not find IK")
                        return 'failed'
                    self.pub_target_config.publish(brics_joint_pos_msg)
                elif len(target) == 5:  # ... of 5 items: Joint space configuration
                    self.pub_target_config.publish(
                        self.list_to_brics_joint_positions(target)
                    )
                else:
                    rospy.logerr("target list is malformed")
                    return "failed"
            else:
                rospy.logerr(
                    "no valid target specified. Target should be a string (name of a joint position) or a list of 7 items (Cartesian Pose + frame id)"
                )
                return "failed"

            timeout = rospy.Duration.from_sec(self.timeout)
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            redo = False
            while blocking:
                if (rospy.Time.now() - start_time) > timeout:
                    rospy.loginfo(
                        "Moveit " + self.moveit_group + "client node response timeout."
                    )
                    break
                if self.client_event:
                    if self.client_event == "e_success":
                        return "succeeded"
                    else:
                        return "failed"
                rate.sleep()

            if not blocking:
                return "succeeded"

        return "failed"

    def list_to_brics_joint_positions(self, joint_positions):
        brics_joint_positions = brics_actuator.msg.JointPositions()
        for idx, joint_position in enumerate(joint_positions):
            joint_value = brics_actuator.msg.JointValue()
            joint_value.timeStamp = rospy.Time.now()
            joint_value.joint_uri = self.joint_names[idx]
            joint_value.unit = "rad"
            joint_value.value = joint_position

            brics_joint_positions.positions.append(joint_value)

        return brics_joint_positions

class ArmPositionCommand:
    def __init__(self, target):
        self.robot_srdf = SRDF.from_parameter_server('/robot_description_semantic')
        self.pub_arm_position = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_cb)
        self.move_arm_to = target
        self.is_arm_moving = False
        self.zero_vel_counter = 0

    def joint_state_cb(self, msg: JointState):
        self.joint_state = msg
        # monitor the velocities
        self.joint_velocities = msg.velocity
        # if all velocities are 0.0, the arm is not moving
        if "arm_joint_1" in msg.name and all([v == 0.0 for v in self.joint_velocities]):
            self.zero_vel_counter += 1

    def get_joint_values_from_group_state(self, group_state_name):
        joint_configs = []
        for group_state in self.robot_srdf.group_states:
            if group_state.name == group_state_name:
                for joint in group_state.joints:
                    joint_configs.append((joint.name, joint.value[0]))
                return joint_configs
        return []

    def execute(self, userdata):
        joint_configs = self.get_joint_values_from_group_state(
            self.move_arm_to or userdata.move_arm_to
        )

        if len(joint_configs) == 0:
            rospy.logerr("No joint configuration found for group state " + self.move_arm_to)
            return "failed"
        else:
            joint_positions = JointPositions()
            joint_positions.positions = [
                JointValue(
                    rospy.Time.now(),
                    joint_name,
                    "rad",
                    joint_value
                )
                for joint_name, joint_value in joint_configs
            ]
            self.pub_arm_position.publish(joint_positions)
            self.zero_vel_counter = 0
            while self.zero_vel_counter < 10:
                rospy.logwarn('Waiting for arm to stop moving')
                rospy.sleep(0.1)
            return "succeeded"

class move_arm(smach.State):
    def __init__(self, target=None, blocking=True, tolerance=None, timeout=10.0, use_moveit=True):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["move_arm_to"]
        )
        joint_names = [f'arm_joint_{i}' for i in range(1, 6)]
        self.arm_moveit_client = MoveitClient("/arm_", target, timeout, joint_names)
        self.arm_position_command = ArmPositionCommand(target)
        self.blocking = blocking
        self.use_moveit = use_moveit

    def execute(self, userdata):
        if self.use_moveit:
            return self.arm_moveit_client.execute(userdata, self.blocking)
        else:
            return self.arm_position_command.execute(userdata)

class check_move_group_feedback(smach.State):
    def __init__(self, timeout=10.0):
        smach.State.__init__(self, outcomes=["succeeded, waiting, failed"])
        self.move_group_status_sub("/move_group/status", self.status_cb)
        self.status = 1

    def status_cb(self, msg):
        self.status = status

    def execute(self):
        if self.status == 3:
            return "succeeded"
        elif self.status == 1:
            return "waiting"
        else:
            return "failed"

class control_gripper(smach.State):
    def __init__(self, target=None, blocking=True, tolerance=None, timeout=5):
        smach.State.__init__(self, outcomes=["succeeded", "timeout"])

        self.timeout = rospy.Duration(timeout)
        self.command = GripperCommand()
        self.current_state = "GRIPPER_OPEN"
        self.grasped_counter = 0
        if type(target) == str:
            if 'open' in target:
                self.command.command = GripperCommand.OPEN
            elif 'close' in target:
                self.command.command = GripperCommand.CLOSE
        elif type(target) == float:
            self.command.command = target
        self.pub = rospy.Publisher('/arm_1/gripper_command', GripperCommand, queue_size=1)
        self.sub = rospy.Subscriber('/arm_1/gripper_feedback', String, self.feedback_cb)

    def feedback_cb(self, msg):
        json_obj = json.loads(msg.data)
        self.current_state = json_obj["state"]
        # if self.current_state == "OBJECT_GRASPED":
            # self.grasped_counter += 1
            # print('grasped counter 1: ', self.grasped_counter)

    def execute(self, userdata):
        self.pub.publish(self.command)
        self.grasped_counter = 0
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time < self.timeout):
            if self.current_state == "OBJECT_GRASPED":
                self.grasped_counter += 1
            if (self.current_state == "GRIPPER_OPEN" or self.current_state == "GRIPPER_INTER") and\
                    self.command.command != GripperCommand.CLOSE:
                self.grasped_counter = 0
                return "succeeded"
            elif self.current_state == "GRIPPER_CLOSED" and\
                    self.command.command == GripperCommand.CLOSE:
                self.grasped_counter = 0
                return "succeeded"
            elif self.current_state == "OBJECT_GRASPED" and\
                    self.grasped_counter > 4 and\
                    self.command.command == GripperCommand.CLOSE:
                self.grasped_counter = 0
                end_time = rospy.Time.now()
                # print the time in seconds
                print('grasped in: ', (end_time - start_time).to_sec())
                return "succeeded"
            rospy.sleep(0.05)
        rospy.logerr("Gripper open/close timed out")
        return "timeout"

class verify_object_grasped(smach.State):
    def __init__(self, timeout=2):
        smach.State.__init__(self, outcomes=["succeeded", "failed", "timeout"])
        
        self.current_state = "OBJECT_GRASPED"
        self.grasped_counter = 0
        self.timeout = rospy.Duration(timeout)
        self.sub = rospy.Subscriber('/arm_1/gripper_feedback', String, self.feedback_cb)

    def feedback_cb(self, msg):
        json_obj = json.loads(msg.data)
        self.current_state = json_obj["state"]
        if self.current_state == "OBJECT_GRASPED":
            self.grasped_counter += 1

    def execute(self, userdata):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time < self.timeout):
            rospy.sleep(0.1)
            if self.current_state == "GRIPPER_CLOSED" or\
               self.current_state == "GRIPPER_INTER" or\
               self.current_state == "GRIPPER_OPEN":
                return "failed"
            elif self.current_state == "OBJECT_GRASPED" and\
                        self.grasped_counter > 4:
                self.grasped_counter = 0
                return "succeeded"
        rospy.logerr('Grasp verification timeout')
        return "timeout"

class move_arm_and_gripper(smach.State):

    """
    Author : Abhishek Padalkar

    This states moves the arm and gripper in parallel.
    It send gripper command directly on cotroller toipc.
    Without waiting for the gripper motion to complete, it initiates arm motion with move_group.
    Grpper feedback can be added here, to do so gripper controller code in this state needed to shifted in separate state,
    take feedback from gripper_controller/state, and run move_arm state and the new gripper state concurrently.
    """

    def __init__(self, gripper_conf, target=None, blocking=True, tolerance=None, timeout=10.0, use_moveit=True):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["move_arm_to"]
        )
        joint_names = [f'arm_joint_{i}' for i in range(1, 6)]
        self.arm_moveit_client = MoveitClient("/arm_", target, timeout, joint_names)
        self.arm_position_command = ArmPositionCommand(target)
        self.blocking = blocking
        self.use_moveit = use_moveit
        self.pub = rospy.Publisher('/arm_1/gripper_command', GripperCommand, queue_size=1)
        self.gripper_conf = gripper_conf
        self.gripper_command = GripperCommand()
        if 'open' in self.gripper_conf:
            self.gripper_command.command = GripperCommand.OPEN
        elif 'close' in self.gripper_conf:
            self.gripper_command.command = GripperCommand.CLOSE
        elif type(self.gripper_conf) == float:
            self.gripper_command.command = self.gripper_conf

    def execute(self, userdata):
        self.pub.publish(self.gripper_command)
        if self.use_moveit:
            return self.arm_moveit_client.execute(userdata, self.blocking)
        else:
            return self.arm_position_command.execute(userdata)


class linear_motion(smach.State):

    """
    Should be called after visual servoing has aligned the gripper with the object.
    Should probably be renamed in the future, or seperated into linear motion and grasping/releasing.
    """

    def __init__(self, operation="grasp", offset_x=0.0):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.operation = operation
        self.result = None
        self.event_out = rospy.Publisher(
            "/arm_relative_motion_controller/event_in", std_msgs.msg.String
        )
        rospy.Subscriber(
            "/arm_relative_motion_controller/event_out",
            std_msgs.msg.String,
            self.event_cb,
        )
        self.offset_x = offset_x

    def execute(self, userdata):
        rospy.set_param(
            "/arm_relative_motion_controller/relative_distance_x", self.offset_x
        )
        self.result = None

        if self.operation == "grasp":
            gripper_command.set_named_target("open")
            gripper_command.go(wait=True)
        elif self.operation == "release":
            pass  # Don't do anything, assume the gripper is already closed

        # start the relative approach and wait for the result
        self.event_out.publish("e_start")
        while not self.result:
            rospy.sleep(0.01)

        if self.result.data != "e_success":
            return "failed"

        if self.operation == "grasp":
            gripper_command.set_named_target("close")
            gripper_command.go(wait=True)
        elif self.operation == "release":
            gripper_command.set_named_target("open")
            gripper_command.go(wait=True)

        return "succeeded"

    def event_cb(self, msg):
        self.result = msg


class compute_pregrasp_pose(smach.State):
    """
    Given an object pose compute a pregrasp position that is reachable and also
    good for the visual servoing.
    """

    FRAME_ID = "/base_link"

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "tf_transform_failed"],
            input_keys=["object_pose"],
            output_keys=["move_arm_to"],
        )
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        pose = userdata.object_pose.pose

        try:
            t = self.tf_listener.getLatestCommonTime(
                self.FRAME_ID, pose.header.frame_id
            )
            pose.header.stamp = t
            pose = self.tf_listener.transformPose(self.FRAME_ID, pose)

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.logerr("Tf error: %s" % str(e))
            return "tf_transform_failed"

        p = pose.pose.position
        o = pose.pose.orientation
        frame_id = pose.header.frame_id

        userdata.move_arm_to = [
            p.x - 0.10,
            p.y,
            p.z + 0.20,
            0,
            (0.8 * math.pi),
            0,
            frame_id,
        ]

        return "succeeded"

class update_static_elements_in_planning_scene(smach.State):

    """
    trigger component to add walls, objects, etc. to the the planning scene of moveit
    """

    def __init__(self, element, action):
        smach.State.__init__(self, outcomes=["succeeded"])

        self.walls_event_out = rospy.Publisher(
            "/mir_manipulation/arm_workspace_restricter/event_in", std_msgs.msg.String
        )

        self.action = action
        self.element = element

    def execute(self, userdata):

        # check which action to perform
        if self.action == "add":
            event_command = "e_start"
        elif self.action == "remove":
            event_command = "e_stop"

        # check which element to add/remove
        if self.element == "walls":
            self.walls_event_out.publish(event_command)

        return "succeeded"


class update_robot_planning_scene(smach.State):

    """
    trigger component to attach, detach, reattach and delete an object to/from the robot's planning scene
    """

    def __init__(self, action):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["object"])

        self.pub_event = rospy.Publisher(
            "/mir_manipulation/grasped_object_attacher/event_in", std_msgs.msg.String
        )
        self.pub_object_id = rospy.Publisher(
            "/mir_manipulation/grasped_object_attacher/object_id", std_msgs.msg.Int32
        )

        self.action = action

    def execute(self, userdata):

        self.pub_object_id.publish(userdata.object.database_id)
        self.pub_event.publish("e_" + self.action)

        return "succeeded"


class select_arm_pose(smach.State):

    """
    TODO
    """

    def __init__(self, pose_name_list=None):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed", "completed"],
            input_keys=["next_arm_pose_index"],
            output_keys=["move_arm_to", "next_arm_pose_index"],
        )
        self.pose_name_list = pose_name_list

    def execute(self, userdata):
        if type(userdata.next_arm_pose_index) is not int:
            userdata.next_arm_pose_index = 0

        if len(self.pose_name_list) <= 0:
            rospy.logerr("pose name list is empty")
            return "failed"

        if userdata.next_arm_pose_index >= len(self.pose_name_list):
            rospy.loginfo("[manipulation states] All poses covered ending loop")
            userdata.next_arm_pose_index = 0
            return "completed"

        userdata.move_arm_to = self.pose_name_list[userdata.next_arm_pose_index]
        userdata.next_arm_pose_index += 1

        return "succeeded"
