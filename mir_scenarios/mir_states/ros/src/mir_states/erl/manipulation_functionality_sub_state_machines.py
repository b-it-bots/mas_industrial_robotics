#!/usr/bin/python

import rospy
import smach
import smach_ros

# import of generic states
import mir_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms
import mir_states.erl.manipulation_functionality_states as mfs

class sub_sm_pickup_object(smach.StateMachine):
    def __init__(self, use_mockup=None):
        smach.StateMachine.__init__(self, outcomes=['success',
                                                    'failure'],
                                          input_keys=['is_object_grasped',
                                                       'end_effector_pose'],
                                          output_keys=['is_object_grasped',
                                                       'end_effector_pose'])
        with self:
            smach.StateMachine.add('SET_NAMED_CONFIG_PREGRASP', gbs.set_named_config('pregrasp_laying_object'),
                transitions={'success':'PLAN_ARM_MOTION',
                             'failure':'failure',
			     'timeout': 'SET_NAMED_CONFIG_PREGRASP'})

            smach.StateMachine.add('PLAN_ARM_MOTION', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/pregrasp_planner_pipeline/event_in','e_start')],
                                    event_out_list=[('/pregrasp_planner_pipeline/event_out','e_success', True)]),
                transitions={'success':'STOP_PLAN_ARM_MOTION',
                             'timeout':'failure',
                             'failure':'failure'})

            smach.StateMachine.add('STOP_PLAN_ARM_MOTION', gbs.send_event([('/pregrasp_planner_pipeline/event_in','e_stop')]),
                transitions={'success':'MOVE_ARM_TO_OBJECT'})

            smach.StateMachine.add('MOVE_ARM_TO_OBJECT', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/move_arm_planned/event_in','e_start')],
                                    event_out_list=[('/move_arm_planned/event_out','e_success', True)],
                                    timeout_duration=15),
                transitions={'success':'STOP_MOVE_ARM_TO_OBJECT',
                             'timeout':'failure',
                             'failure':'failure'})

            smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT', gbs.send_event([('/move_arm_planned/event_in','e_stop')]),
                transitions={'success':'SAVE_GRIPPER_POSE'})

            smach.StateMachine.add('SAVE_GRIPPER_POSE', mfs.save_gripper_pose(),
                transitions={'success':'CLOSE_GRIPPER'})

            smach.StateMachine.add('CLOSE_GRIPPER', gms.control_gripper('close'),
                transitions={'succeeded':'VERIFY_GRIPPER_CLOSED'})

            smach.StateMachine.add('VERIFY_GRIPPER_CLOSED', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/gripper_state_monitor/event_in','e_trigger')],
                                    event_out_list=[('/gripper_state_monitor/event_out','e_gripper_closed', True)],
                                    timeout_duration=5),
                transitions={'success':'SET_NAMED_CONFIG_LIFT_OBJECT',
                             'timeout':'CLOSE_GRIPPER',
                             'failure':'CLOSE_GRIPPER'})

            smach.StateMachine.add('SET_NAMED_CONFIG_LIFT_OBJECT', gbs.set_named_config('lift_object'),
                transitions={'success':'PLAN_LIFT_OBJECT',
                             'failure':'failure',
			     'timeout': 'SET_NAMED_CONFIG_LIFT_OBJECT'})

            smach.StateMachine.add('PLAN_LIFT_OBJECT', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/poses_to_move_wrtbase/event_in','e_start')],
                                    event_out_list=[('/poses_to_move_wrtbase/event_out','e_success', True)],
                                    timeout_duration=3),
                transitions={'success':'LIFT_OBJECT',
                             'timeout':'LIFT_OBJECT',
                             'failure':'LIFT_OBJECT'})

            smach.StateMachine.add('LIFT_OBJECT', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/cartesian_controller_demo/event_in','e_start')],
                                    event_out_list=[('/cartesian_controller_demo/event_out','e_success', True)],
                                    timeout_duration=10),
                transitions={'success':'VERIFY_GRASPED',
                             'timeout':'VERIFY_GRASPED',
                             'failure':'VERIFY_GRASPED'})

            smach.StateMachine.add('VERIFY_GRASPED', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/gripper_controller/grasp_monitor/event_in','e_trigger')],
                                    event_out_list=[('/gripper_controller/grasp_monitor/event_out','e_object_grasped', True)],
                                    timeout_duration=10),
                transitions={'success':'SET_OBJECT_GRASPED',
                             'timeout':'SET_OBJECT_NOT_GRASPED',
                             'failure':'SET_OBJECT_NOT_GRASPED'})

            smach.StateMachine.add('SET_OBJECT_GRASPED', mfs.set_is_object_grasped(True),
                transitions={'success':'success'})

            smach.StateMachine.add('SET_OBJECT_NOT_GRASPED', mfs.set_is_object_grasped(False),
                transitions={'success':'success'})


class sub_sm_place_object_and_reset(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success',
                                                    'failure'],
                                          output_keys=['is_object_grasped',
                                                       'end_effector_pose'])
        with self:
            smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open'),
                transitions={'succeeded':'LOOK_AT_WORKSPACE'})

            smach.StateMachine.add('LOOK_AT_WORKSPACE', gms.move_arm('look_at_workspace'),
                transitions={'succeeded': 'success',
                             'failed': 'failure'})
