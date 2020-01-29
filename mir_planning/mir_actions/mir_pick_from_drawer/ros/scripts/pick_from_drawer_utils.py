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
        pass

    def calc_pose_for_dbc_for_param(self, param_name='default'):
        """Calculate pose for dbc based on the param

        :param_name: str
        :returns: geometry_msgs.PoseStamped

        """
        pass
