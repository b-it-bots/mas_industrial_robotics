import tf
import rospy
import smach

import mir_states.common.manipulation_states as gms
import mir_states.common.navigation_states as gns
import mir_states.common.perception_states as gps


__all__ = ['unload_object']


###############################################################################
#                               State machine                                 #
###############################################################################

class unload_object(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'failed'],
                                    input_keys=['src_location',
                                                'dst_location',
                                                'rear_platform'],
                                    output_keys=['rear_platform'])
        with self:
            smach.StateMachine.add('PICK_OBJECT_FROM_REAR_PLATFORM',
                                   gms.pick_object_from_rear_platform(),
                                   transitions={'succeeded': 'MOVE_ARM_TO_RELEASE',
                                                'rear_platform_is_empty': 'failed',
                                                'failed': 'failed'},
                                   remapping={'location': 'src_location'})

            smach.StateMachine.add('MOVE_ARM_TO_RELEASE',
                                   gms.move_arm(),
                                   transitions={'succeeded': 'OPEN_GRIPPER',
                                                'failed': 'failed'},
                                   remapping={'move_arm_to': 'dst_location'})

            smach.StateMachine.add('OPEN_GRIPPER',
                                   gms.control_gripper('open'),
                                   transitions={'succeeded': 'succeeded'})
