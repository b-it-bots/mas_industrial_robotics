#! usr/bin/env python
from __future__ import print_function

import copy
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion


class PickFromShelfUtils(object):

    """Util class to help pick_from_shelf_server"""

    def __init__(self):
        # ros params
        self.frame_id = rospy.get_param("~frame_id", "base_link_static")
        self.y_pos_movement_threshold = rospy.get_param(
            "~y_pos_movement_threshold", 0.15
        )
        self.start_base_pose_offset = rospy.get_param("~start_base_pose_offset", -0.7)
        self.pitch = rospy.get_param("~pitch", 2.0)
        self.arm_link_0_offset_x = rospy.get_param("~arm_link_0_offset_x", 0.223)
        self.arm_link_0_offset_y = rospy.get_param("~arm_link_0_offset_y", 0.0)
        self.intermediate_perc = rospy.get_param("~intermediate_perc", 0.8)
        self.retract_base_pose_x = rospy.get_param("~retract_base_pose_x", -0.2)

    def calc_pose_for_dbc(self, obj_pose):
        """Assumption: object pose is in `base_link_static`.
        Calculates a target pose for direct base controller necessary for the
        youbot to pick an object from a lower shelf.
        If the obj pose is too far from the middle of the workspace in y axis,
        then it is clipped. This is done so that the base does not travel too far
        away from the middle of the workspace and (possibly) collide with other
        obstacles.

        :obj_pose: geometry_msgs.PoseStamped
        :returns: geometry_msgs.PoseStamped

        """
        dbc_target_pose = PoseStamped()
        dbc_target_pose.header.stamp = rospy.Time.now()
        dbc_target_pose.header.frame_id = self.frame_id
        dbc_target_pose.pose.position.x = (
            self.start_base_pose_offset + obj_pose.pose.position.x
        )
        dbc_target_pose.pose.position.y = min(
            self.y_pos_movement_threshold,
            max(-self.y_pos_movement_threshold, obj_pose.pose.position.y),
        )
        dbc_target_pose.pose.orientation.w = 1.0
        return dbc_target_pose

    def get_arm_pose(self, obj_pose, is_intermediate=False):
        """Return a pose the youbot arm's end effector should go to based on object pose.
        The orientation is modified such that roll is always 0.0.
        Pitch is based on parameter (leaning forward for the youbot arm to be able to pick)
        and yaw is based on how far the object is from the middle of the workstation.

        If is_intermediate is True, the arm pose returned is not all the way at
        obj pose but somewhere in the middle. This mimics behaviour of pre grasp planning
        but with a bit more generalisation. It can be made a bit more general by
        considering z position.

        :obj_pose: geometry_msgs.PoseStamped
        :is_intermediate: bool
        :returns: geometry_msgs.PoseStamped

        """
        robot_current_loc = self.calc_pose_for_dbc(obj_pose)
        arm_link_0_x = robot_current_loc.pose.position.x + self.arm_link_0_offset_x
        arm_link_0_y = robot_current_loc.pose.position.y + self.arm_link_0_offset_y
        delta_x = obj_pose.pose.position.x - arm_link_0_x
        delta_y = obj_pose.pose.position.y - arm_link_0_y
        yaw = math.atan2(delta_y, delta_x)
        quat = tf.transformations.quaternion_from_euler(0.0, self.pitch, yaw)
        arm_pose = copy.deepcopy(obj_pose)
        arm_pose.pose.orientation = Quaternion(*quat)
        if is_intermediate:
            arm_pose.pose.position.x = arm_link_0_x + (self.intermediate_perc * delta_x)
            arm_pose.pose.position.y = arm_link_0_y + (self.intermediate_perc * delta_y)
        return arm_pose

    def get_retracted_dbc_pose(self):
        """Return a pose in `base_link_static` frame for the robot to back up

        :returns: geometry_msgs.PoseStamped

        """
        dbc_target_pose = PoseStamped()
        dbc_target_pose.header.stamp = rospy.Time.now()
        dbc_target_pose.header.frame_id = self.frame_id
        dbc_target_pose.pose.position.x = self.retract_base_pose_x
        dbc_target_pose.pose.position.y = 0.0
        dbc_target_pose.pose.orientation.w = 1.0
        return dbc_target_pose
