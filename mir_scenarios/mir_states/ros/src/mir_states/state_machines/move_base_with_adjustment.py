import tf
import rospy
import smach

import mir_states.common.manipulation_states as gms
import mir_states.common.navigation_states as gns
import mir_states.common.perception_states as gps


__all__ = ['move_base_with_adjustment']


###############################################################################
#                               State machine                                 #
###############################################################################

class move_base_with_adjustment(smach.StateMachine):

    def __init__(self, move_base_to=None):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'failed'],
                                    input_keys=['move_base_to'],
                                    output_keys=['base_pose'])
        with self:
            smach.StateMachine.add('MOVE_BASE', gns.move_base(move_base_to),
                                   transitions={'succeeded': 'ADJUST_TO_WORKSPACE'})

            smach.StateMachine.add('ADJUST_TO_WORKSPACE', gns.adjust_to_workspace())
