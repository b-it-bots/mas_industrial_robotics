#!/usr/bin/python

import rospy
import smach
import smach_ros

# import of generic states
import mir_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms
import mir_states.rockin.manipulation_functionality_states as mfs

class sub_sm_pickup_plate(smach.StateMachine):
    def __init__(self, use_mockup=None):
        smach.StateMachine.__init__(self, outcomes=['success',
                                                    'failure'])
        with self:

            #Opening the gripper
            smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open'),
                transitions={'succeeded':'STOP_POSE_SHIFTER'})
            # Sending STOP to all components 
            smach.StateMachine.add('STOP_POSE_SHIFTER', gbs.send_event([('/plate_pose_shifter/event_in','e_stop')]),
                transitions={'success':'SET_NAMED_CONFIG_POSE_SHIFT'})

            smach.StateMachine.add('SET_NAMED_CONFIG_POSE_SHIFT', gbs.set_named_config('plate_pose_shifter'),
                transitions={'success':'SHIFT_PLATE_POSE',
                             'failure':'failure',
			                 'timeout': 'SET_NAMED_CONFIG_POSE_SHIFT'})

            smach.StateMachine.add('SHIFT_PLATE_POSE', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/plate_pose_shifter/event_in','e_start')],
                                    event_out_list=[('/plate_pose_shifter/event_out','e_success', True)],
                                    timeout_duration=5),
                transitions={'success':'SET_NAMED_CONFIG_PREGRASP',
                             'timeout':'failure',
                             'failure':'failure'})

            smach.StateMachine.add('SET_NAMED_CONFIG_PREGRASP', gbs.set_named_config('conveyor_belt_plate_pickup'),
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
                                    timeout_duration=10),
                transitions={'success':'STOP_MOVE_ARM_TO_OBJECT',
                             'timeout':'failure',
                             'failure':'failure'})

            smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT', gbs.send_event([('/move_arm_planned/event_in','e_stop')]),
                transitions={'success':'SET_NAMED_CONFIG_APPROACH_PLATE'})

            smach.StateMachine.add('SET_NAMED_CONFIG_APPROACH_PLATE', gbs.set_named_config('approach_plate'),
                transitions={'success':'PLAN_APPROACH_PLATE',
                             'failure':'failure',
		            	     'timeout': 'SET_NAMED_CONFIG_APPROACH_PLATE'})

            smach.StateMachine.add('PLAN_APPROACH_PLATE', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/poses_to_move_wrtgripper/event_in','e_start')],
                                    event_out_list=[('/poses_to_move_wrtgripper/event_out','e_success', True)],
                                    timeout_duration=10),
                transitions={'success':'APPROACH_PLATE',
                             'timeout':'APPROACH_PLATE',
                             'failure':'APPROACH_PLATE'})

            smach.StateMachine.add('APPROACH_PLATE', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/cartesian_controller_demo/event_in','e_start')],
                                    event_out_list=[('/cartesian_controller_demo/event_out','e_success', True)],
                                    timeout_duration=10),
                transitions={'success':'CLOSE_GRIPPER',
                             'timeout':'CLOSE_GRIPPER',
                             'failure':'CLOSE_GRIPPER'})


            smach.StateMachine.add('CLOSE_GRIPPER', gms.control_gripper('close'),
                transitions={'succeeded':'VERIFY_GRIPPER_CLOSED'})

            smach.StateMachine.add('VERIFY_GRIPPER_CLOSED', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/gripper_state_monitor/event_in','e_trigger')],
                                    event_out_list=[('/gripper_state_monitor/event_out','e_gripper_closed', True)],
                                    timeout_duration=5),
                transitions={'success':'STOP_APPROACH_PLATE',
                             'timeout':'CLOSE_GRIPPER',
                             'failure':'CLOSE_GRIPPER'})

            smach.StateMachine.add('STOP_APPROACH_PLATE', gbs.send_event([('/cartesian_controller_demo/event_in','e_stop'),
                                                                          ('/poses_to_move_wrtgripper/event_in','e_stop')]),
                transitions={'success':'SET_NAMED_CONFIG_RETREAT_PLATE'})

            smach.StateMachine.add('SET_NAMED_CONFIG_RETREAT_PLATE', gbs.set_named_config('retreat_plate'),
                transitions={'success':'PLAN_RETREAT_PLATE',
                             'failure':'failure',
		            	     'timeout': 'SET_NAMED_CONFIG_RETREAT_PLATE'})

            smach.StateMachine.add('PLAN_RETREAT_PLATE', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/poses_to_move_wrtgripper/event_in','e_start')],
                                    event_out_list=[('/poses_to_move_wrtgripper/event_out','e_success', True)],
                                    timeout_duration=10),
                transitions={'success':'RETREAT_PLATE',
                             'timeout':'RETREAT_PLATE',
                             'failure':'RETREAT_PLATE'})

            smach.StateMachine.add('RETREAT_PLATE', gbs.send_and_wait_events_combined(
                                    event_in_list=[('/cartesian_controller_demo/event_in','e_start')],
                                    event_out_list=[('/cartesian_controller_demo/event_out','e_success', True)],
                                    timeout_duration=10),
                transitions={'success':'STOP_RETREAT_PLATE',
                             'timeout':'STOP_RETREAT_PLATE',
                             'failure':'STOP_RETREAT_PLATE'})

            smach.StateMachine.add('STOP_RETREAT_PLATE', gbs.send_event([('/cartesian_controller_demo/event_in','e_stop'),
                                                                          ('/poses_to_move_wrtgripper/event_in','e_stop')]),
                transitions={'success':'MOVE_TO_HOLD'})

            smach.StateMachine.add('MOVE_TO_HOLD', gms.move_arm('look_at_workspace'),
                transitions={'succeeded': 'success',
                             'failed': 'failure'})
