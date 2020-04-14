#! usr/bin/env python
from __future__ import print_function

import rospy
import smach
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from mir_planning_msgs.msg import GenericExecuteAction, GenericExecuteResult, GenericExecuteFeedback

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

        dbc_param = self._dbc_params[param_name]
        dbc_target_pose = PoseStamped()
        dbc_target_pose.header.stamp = rospy.Time.now()
        dbc_target_pose.header.frame_id = self._frame_id
        if pose_in is None:
            dbc_target_pose.pose.position.x = dbc_param['x_offset']
            dbc_target_pose.pose.position.y = 0.0
        else:
            dbc_target_pose.pose.position.x = pose_in.pose.position.x + dbc_param['x_offset']
            dbc_target_pose.pose.position.y = pose_in.pose.position.y

        drawer_length_offset = self._drawer_length * dbc_param['drawer_length_factor']
        dbc_target_pose.pose.position.x += drawer_length_offset
        dbc_target_pose.pose.orientation.w = 1.0

        return dbc_target_pose

#===============================================================================

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                   input_keys=['handle_pose'],
                                   output_keys=['feedback', 'result', 'handle_pose'])

    def execute(self, userdata):
        userdata.handle_pose = None # reset handle pose

        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(current_state='Setup',
                                                   text='Setting up')
        return 'succeeded'

#===============================================================================

class SendSafePoseToDBC(smach.State):
    def __init__(self, target_dist=0.2):
        smach.State.__init__(self, outcomes=['succeeded'])
        self._dbc_pose_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/input_pose',
                                             PoseStamped, queue_size=10)
        laser_dist_sub = rospy.Subscriber('/mcr_navigation/laser_distances/distances',
                                          Float32MultiArray, self._laser_dist_cb)
        self._front_laser_dist = 0.0
        self._target_dist = target_dist
        rospy.sleep(0.1) # time for the publisher to register in ros network

    def _laser_dist_cb(self, msg):
        self._front_laser_dist = msg.data[0]

    def execute(self, userdata):
        x_offset = self._front_laser_dist - self._target_dist

        dbc_target_pose = PoseStamped()
        dbc_target_pose.header.stamp = rospy.Time.now()
        dbc_target_pose.header.frame_id = 'base_link_static'
        dbc_target_pose.pose.position.x = x_offset
        dbc_target_pose.pose.position.y = 0.0
        dbc_target_pose.pose.orientation.w = 1.0
        rospy.loginfo(dbc_target_pose)
        self._dbc_pose_pub.publish(dbc_target_pose)
        return 'succeeded'

#===============================================================================

class SendPoseToDBC(smach.State):
    def __init__(self, param_name='default'):
        smach.State.__init__(self, outcomes=['succeeded'],
                                   input_keys=['handle_pose'])
        self._dbc_pose_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/input_pose',
                                             PoseStamped, queue_size=10)
        self._param_name = param_name
        self.utils = ManipulateDrawerUtils()
        rospy.sleep(0.1) # time for the publisher to register in ros network

    def execute(self, userdata):
        dbc_pose = self.utils.calc_pose_for_dbc_for_param(userdata.handle_pose,
                                                          self._param_name)
        self._dbc_pose_pub.publish(dbc_pose)
        return 'succeeded'

