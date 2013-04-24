PACKAGE = 'raw_generic_states'

import roslib
roslib.load_manifest(PACKAGE)

import tf
import rospy
import smach

import generic_manipulation_states as gms
import generic_navigation_states as gns
import generic_perception_states as gps


__all__ = ['unload_object']


###############################################################################
#                               State machine                                 #
###############################################################################

class unload_object(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'failed'],
                                    input_keys=['move_arm_to',
                                                'rear_platform'],
                                    output_keys=['rear_platform'])
        with self:
            smach.StateMachine.add('PICK_OBJECT_FROM_REAR_PLATFORM',
                                   gms.pick_object_from_rear_platform(),
                                   transitions={'succeeded': 'MOVE_ARM_TO_RELEASE',
                                                'rear_platform_is_empty': 'failed',
                                                'failed': 'failed'})

            smach.StateMachine.add('MOVE_ARM_TO_RELEASE',
                                   gms.move_arm(),
                                   transitions={'succeeded': 'OPEN_GRIPPER',
                                                'failed': 'failed'})

            smach.StateMachine.add('OPEN_GRIPPER',
                                   gms.control_gripper('open'),
                                   transitions={'succeeded': 'succeeded'})
