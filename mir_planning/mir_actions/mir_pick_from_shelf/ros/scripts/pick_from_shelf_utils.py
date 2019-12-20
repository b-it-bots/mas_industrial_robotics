#! usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

class PickFromShelfUtils(object):

    """Util class to help pick from shelf server"""

    def __init__(self):
        pass

    @staticmethod
    def calc_pose_for_dbc(obj_pose):
        """Assumption: object pose is in `base_link_static`.
        Calculates a target pose for direct base controller necessary for the
        youbot to pick an object from a lower shelf

        :obj_pose: geometry_msgs.PoseStamped
        :returns: geometry_msgs.PoseStamped

        """
        x = - 0.7 + obj_pose.pose.position.x
        dbc_target_pose = PoseStamped()
        dbc_target_pose.header.stamp = rospy.Time.now()
        dbc_target_pose.header.frame_id = 'base_link_static'
        dbc_target_pose.pose.position.x = x
        dbc_target_pose.pose.position.y = obj_pose.pose.position.y
        dbc_target_pose.pose.orientation.w = 1.0
        return dbc_target_pose

    @staticmethod
    def modify_obj_pose_orientation(obj_pose):
        """Modify the orientation of the object pose such that roll and yaw are 0.0
        and pitch is leaning forward for the youbot arm to be able to pick.

        :obj_pose: geometry_msgs.PoseStamped
        :returns: geometry_msgs.PoseStamped

        """
        pitch = 2.0
        quat = tf.transformations.quaternion_from_euler(0.0, pitch, 0.0)
        obj_pose.pose.orientation = Quaternion(*quat)
        return obj_pose
        
    @staticmethod
    def get_retracted_dbc_pose():
        """Return a pose in `base_link_static` frame for the robot to back up
        :returns: geometry_msgs.PoseStamped

        """
        dbc_target_pose = PoseStamped()
        dbc_target_pose.header.stamp = rospy.Time.now()
        dbc_target_pose.header.frame_id = 'base_link_static'
        dbc_target_pose.pose.position.x = -0.2
        dbc_target_pose.pose.position.y = 0.0
        dbc_target_pose.pose.orientation.w = 1.0
        return dbc_target_pose
