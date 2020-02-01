#! usr/bin/env python
from __future__ import print_function

import tf
import copy
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

class ManipulateDrawerUtils(object):

    """Util class to help open and close drawers"""

    def __init__(self):
        self._dbc_params = rospy.get_param('~dbc_params', None)
        self._drawer_length = rospy.get_param('~drawer_length', 0.25)
        self._frame_id = rospy.get_param('~frame_id', 'base_link_static')
        if self._dbc_params is None:
            rospy.logfatal('Params not available')

    def calc_pose_for_dbc_for_param(self, pose_in, param_name='default'):
        """Calculate pose for dbc based on the param

        :pose_in: geometry_msgs.PoseStamped
        :param_name: str
        :returns: geometry_msgs.PoseStamped

        """
        if param_name not in self._dbc_params:
            return PoseStamped()

        dbc_target_pose = PoseStamped()
        dbc_target_pose.header.stamp = rospy.Time.now()
        dbc_target_pose.header.frame_id = self._frame_id
        if pose_in is None:
            dbc_target_pose.pose.position.x = self._dbc_params[param_name]['x_offset']
            dbc_target_pose.pose.position.y = 0.0
        else:
            dbc_target_pose.pose.position.x = pose_in.pose.position.x + self._dbc_params[param_name]['x_offset']
            if param_name in ['open_drawer_post', 'open_drawer_post_safe', 'perceive_drawer', 'close_drawer_pre_safe']:
                dbc_target_pose.pose.position.x -= self._drawer_length
            dbc_target_pose.pose.position.y = pose_in.pose.position.y
        dbc_target_pose.pose.orientation.w = 1.0

        return dbc_target_pose
