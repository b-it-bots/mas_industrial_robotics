PACKAGE = 'raw_generic_states'

import roslib
roslib.load_manifest(PACKAGE)

import tf
import rospy
import smach

import generic_manipulation_states as gms
import generic_navigation_states as gns
import generic_perception_states as gps


__all__ = ['load_object']


###############################################################################
#                             Helper sub-states                               #
###############################################################################

class compute_pregrasp_pose(smach.State):

    """
    Given an object pose compute a pregrasp position that is reachable and also
    good for the visual servoing.
    """

    FRAME_ID = '/base_link'

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'tf_error'],
                             input_keys=['object'],
                             output_keys=['move_arm_to'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        pose = userdata.object.pose
        try:
            t = self.tf_listener.getLatestCommonTime(self.FRAME_ID,
                                                     pose.header.frame_id)
            pose.header.stamp = t
            pose = self.tf_listener.transformPose(self.FRAME_ID, pose)
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr('Tf error: %s' % str(e))
            return 'tf_error'
        p = pose.pose.position
        o = pose.pose.orientation
        userdata.move_arm_to = [self.FRAME_ID,
                                p.x - 0.04, p.y, p.z + 0.08,
                                0, 3.14, 0]
        return 'succeeded'


class compute_base_shift_to_object(smach.State):

    FRAME_ID = '/base_link'

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'tf_error'],
                             input_keys=['object'],
                             output_keys=['move_base_by'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        pose = userdata.object.pose
        try:
            t = self.tf_listener.getLatestCommonTime(self.FRAME_ID,
                                                     pose.header.frame_id)
            pose.header.stamp = t
            relative = self.tf_listener.transformPose(self.FRAME_ID, pose)
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr('Tf error: %s' % str(e))
            return 'tf_error'
        userdata.move_base_by = (relative.pose.position.x - 0.65, relative.pose.position.y, 0)
        return 'succeeded'


###############################################################################
#                               State machine                                 #
###############################################################################

class load_object(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'failed'],
                                    input_keys=['simulation',
                                                'object',
                                                'rear_platform'],
                                    output_keys=['rear_platform'])
        with self:
            smach.StateMachine.add('OPEN_GRIPPER',
                                   gms.control_gripper('open'),
                                   transitions={'succeeded': 'COMPUTE_PREGRASP_POSE'})

            smach.StateMachine.add('COMPUTE_PREGRASP_POSE',
                                   compute_pregrasp_pose(),
                                   transitions={'succeeded': 'MOVE_ARM_TO_PREGRASP',
                                                'tf_error': 'failed'})

            smach.StateMachine.add('MOVE_ARM_TO_PREGRASP',
                                   gms.move_arm(tolerance=[0, 0.4, 0]),
                                   transitions={'succeeded': 'DO_VISUAL_SERVOING',
                                                'failed': 'COMPUTE_BASE_SHIFT_TO_OBJECT'})

            smach.StateMachine.add('COMPUTE_BASE_SHIFT_TO_OBJECT',
                                   compute_base_shift_to_object(),
                                   transitions={'succeeded': 'MOVE_BASE_RELATIVE',
                                                'tf_error': 'failed'})

            smach.StateMachine.add('MOVE_BASE_RELATIVE',
                                   gns.move_base_relative(),
                                   transitions={'succeeded': 'COMPUTE_PREGRASP_POSE',
                                                'failed': 'failed'})

            smach.StateMachine.add('DO_VISUAL_SERVOING',
                                   gps.do_visual_servoing(),
                                   transitions={'succeeded': 'GRASP_OBJECT',
                                                'failed': 'failed',
                                                'timeout': 'failed'})

            smach.StateMachine.add('GRASP_OBJECT',
                                   gms.grasp_object(),
                                   transitions={'succeeded': 'PUT_OBJECT_ON_REAR_PLATFORM',
                                                'tf_error': 'failed'})

            smach.StateMachine.add('PUT_OBJECT_ON_REAR_PLATFORM',
                                   gms.put_object_on_rear_platform(),
                                   transitions={'succeeded': 'succeeded',
                                                'rear_platform_is_full': 'failed',
                                                'failed': 'failed'})
