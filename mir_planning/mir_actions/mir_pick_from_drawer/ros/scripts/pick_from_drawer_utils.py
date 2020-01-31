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
        self._arm_params = rospy.get_param('~arm_params', None)
        self._drawer_length = rospy.get_param('~drawer_length', 0.25)
        self._frame_id = rospy.get_param('~frame_id', 'base_link_static')
        if self._dbc_params is None or self._arm_params is None:
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
            dbc_target_pose.pose.position.y = 0.0
        else:
            dbc_target_pose.pose.position.x = pose_in.pose.position.x + self._dbc_params[param_name]['x_offset']
            if param_name in ['open_drawer_post', 'open_drawer_post_safe']:
                dbc_target_pose.pose.position.x -= self._drawer_length
            dbc_target_pose.pose.position.y = pose_in.pose.position.y
        dbc_target_pose.pose.orientation.w = 1.0

        return dbc_target_pose

    def calc_pose_for_arm_for_param(self, pose_in, param_name='default'):
        """Return a pose the youbot arm's end effector should go to based on `pose_in`.
        The orientation is modified based on param_name 

        :pose_in: geometry_msgs.PoseStamped
        :param_name: str
        :returns: geometry_msgs.PoseStamped

        """
        if param_name not in self._arm_params or pose_in is None:
            return PoseStamped()

        arm_param_dict = self._arm_params[param_name]
        print(arm_param_dict)

        arm_target_pose = copy.deepcopy(pose_in)
        arm_target_pose.header.stamp = rospy.Time.now()
        arm_target_pose.pose.position.x += arm_param_dict['x_offset']
        arm_target_pose.pose.position.y += arm_param_dict['y_offset']
        arm_target_pose.pose.position.z += arm_param_dict['z_offset']
        quat = tf.transformations.quaternion_from_euler(arm_param_dict['roll'],
                                                        arm_param_dict['pitch'],
                                                        arm_param_dict['yaw'])
        arm_target_pose.pose.orientation = Quaternion(*quat)

        return arm_target_pose
