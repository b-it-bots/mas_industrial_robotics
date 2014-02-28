import tf
import rospy
import smach

import mir_states.common.manipulation_states as gms
import mir_states.common.navigation_states as gns
import mir_states.common.perception_states as gps


__all__ = ['move_base_with_adjustment_and_arm']


###############################################################################
#                               State machine                                 #
###############################################################################

class move_base_with_adjustment_and_arm(smach.StateMachine):

    def __init__(self, move_base_to=None, move_arm_to=None):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'failed'],
                                    input_keys=['move_base_to', 'move_arm_to'],
                                    output_keys=['base_pose'])
        with self:
            smach.StateMachine.add('MOVE_BASE', gns.move_base(move_base_to),
                transitions={'succeeded': 'ADJUST_AND_MOVE_ARM'})

            sm_concurrent = smach.Concurrence(outcomes=['succeeded', 'failed'],
                                       default_outcome='succeeded',
                                       outcome_map={'succeeded': {'MOVE_ARM': 'succeeded',
                                                                  'ADJUST_TO_WORKSPACE': 'succeeded'},
                                                    'failed': {'ADJUST_TO_WORKSPACE': 'failed'}})

            with sm_concurrent:
                smach.Concurrence.add('MOVE_ARM', gms.move_arm(move_arm_to))
                smach.Concurrence.add('ADJUST_TO_WORKSPACE', gns.adjust_to_workspace())

            smach.StateMachine.add('ADJUST_AND_MOVE_ARM', sm_concurrent,
                transitions={'failed': 'ADJUST_AND_MOVE_ARM'})
