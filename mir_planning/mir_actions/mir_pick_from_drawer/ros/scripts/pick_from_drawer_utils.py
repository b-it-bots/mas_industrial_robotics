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
        if self._dbc_params is None:
            rospy.logfatal('Params not available')

    def calc_pose_for_dbc_for_param(self, pose_in, param_name='default'):
        """Calculate pose for dbc based on the param

        :pose_in: geometry_msgs.PoseStamped
        :param_name: str
        :returns: geometry_msgs.PoseStamped

        """
        if param_name in self._dbc_params:
            print(self._dbc_params[param_name])
        output_pose = PoseStamped()
        return output_pose
