#! usr/bin/env python
from __future__ import print_function

import tf
import copy
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

class PickFromDrawerUtils(object):

    """Util class to help pick_from_drawer_server"""

    def __init__(self):
        self._dbc_params = rospy.get_param('~dbc_params', None)
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

        print(self._dbc_params[param_name])

        dbc_target_pose = PoseStamped()
        dbc_target_pose.header.stamp = rospy.Time.now()
        dbc_target_pose.header.frame_id = self._frame_id
        if pose_in is None:
            dbc_target_pose.pose.position.x = self._dbc_params[param_name]['x_offset']
        else:
            dbc_target_pose.pose.position.x = pose_in.pose.position.x + self._dbc_params[param_name]['x_offset']
        dbc_target_pose.pose.position.y = 0.0
        dbc_target_pose.pose.orientation.w = 1.0

        return dbc_target_pose
