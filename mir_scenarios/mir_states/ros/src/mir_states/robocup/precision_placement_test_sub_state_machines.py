#!/usr/bin/python

import rospy
import smach
import smach_ros

# import of generic states
import mir_states.common.navigation_states as gns
import mir_states.common.manipulation_states as gms
import mir_states.common.perception_states as gps
import mcr_states.common.basic_states as gbs

#import robocup specific states
import mir_states.robocup.basic_transportation_test_states as btts
import mir_states.robocup.precision_placement_test_states as ppts


################################################################
class sub_sm_place_in_holes(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=['succeeded', 'failed', 'no_object_for_ppt_platform'],
            input_keys=['base_pose_to_approach',
                        'last_grasped_obj',
                        'move_arm_to',
                        'move_base_by',
                        'object_pose',
                        'rear_platform_free_poses',
                        'rear_platform_occupied_poses',
                        'selected_objects',
                        'next_arm_pose_index',
                        'cavity_pose',
                        'task_list'],
            output_keys=['base_pose_to_approach',
                         'last_grasped_obj',
                         'move_arm_to',
                         'move_base_by',
                         'rear_platform_free_poses',
                         'rear_platform_occupied_poses',
                         'selected_objects',
                         'next_arm_pose_index',
                         'cavity_pose',
                         'task_list'])

        with self:

            # TODO: ENSURE WE HAVE ENOUGH SPACE IN THE ARENA 
            smach.StateMachine.add('ADJUST_POSE_WRT_WORKSPACE_AT_SOURCE', gns.adjust_to_workspace(0.2),
                transitions={'succeeded': 'SELECT_OBJECTS_TO_PLACE',
                             'failed': 'ADJUST_POSE_WRT_WORKSPACE_AT_SOURCE'})


            # Select object from task list which needs to be place in cavities.
            smach.StateMachine.add('SELECT_OBJECTS_TO_PLACE', ppts.select_objects_to_place(),
                transitions={'objects_selected': 'CLEAR_CAVITIES',
                             'no_more_obj_for_this_workspace': 'no_object_for_ppt_platform'})

            smach.StateMachine.add('CLEAR_CAVITIES', ppts.clear_cavities(),
                transitions={'succeeded': 'SELECT_NEXT_LOOK_POSE'})

            # CAVITY RECOGNITION PIPELINE
            smach.StateMachine.add('SELECT_NEXT_LOOK_POSE', gms.select_arm_pose(['look_at_workspace_right', 'look_at_workspace', 'look_at_workspace_left']),
                    transitions={'succeeded': 'LOOK_AROUND',
                                'failed': 'CHECK_FOUND_CAVITIES'})

            smach.StateMachine.add('LOOK_AROUND', gms.move_arm(),
                transitions={'succeeded': 'FIND_CAVITIES_LOOP',
                             'failed': 'LOOK_AROUND'})

            smach.StateMachine.add('FIND_CAVITIES_LOOP', gbs.loop_for(3),
                                      transitions={'loop': 'FIND_CAVITIES',
                                                   'continue': 'CHECK_FOUND_CAVITIES'})

            smach.StateMachine.add('FIND_CAVITIES', gps.find_cavities(),
                transitions={'succeeded': 'TRANSFORM_CAVITIES',
                             'failed': 'SELECT_NEXT_LOOK_POSE'})

            smach.StateMachine.add('TRANSFORM_CAVITIES', gps.transform_object_poses(frame_id='/odom'),
                transitions={'succeeded': 'FIND_BEST_MATCHED_CAVITIES'},
                remapping={'found_objects': 'found_cavities'})

            smach.StateMachine.add('FIND_BEST_MATCHED_CAVITIES', gps.find_best_matched_cavities(),
                transitions={'succeeded': 'SELECT_NEXT_LOOK_POSE',
                             'complete': 'CHECK_FOUND_CAVITIES'})            

            smach.StateMachine.add('CHECK_FOUND_CAVITIES', gps.check_found_cavities(),
                transitions={'cavities_found': 'STOP_ALL_COMPONENTS_AT_START',
                             'no_cavities_found': 'failed'})
           
 
            smach.StateMachine.add('STOP_ALL_COMPONENTS_AT_START', gbs.send_event([('/gripper_to_object_pose_error_calculator/event_in','e_stop'),
                                                                          ('/mcr_common/relative_displacement_calculator/event_in','e_stop'),
                                                                          ('/pregrasp_planner/event_in','e_stop'),
                                                                          ('/pregrasp_arm_monitor/event_in', 'e_stop'),
                                                                          ('/mcr_navigation/relative_base_controller/event_in','e_stop'),
                                                                          ('/planned_motion/event_in','e_stop')]),
                transitions={'success':'SELECT_OBJECT_TO_PLACE'})

            # OBJECT PLACING pipeline
            # select cavitiy based on recognized cavities and list of objects to place in cavity.
            smach.StateMachine.add('SELECT_OBJECT_TO_PLACE', ppts.select_object_to_place(),
                transitions={'object_selected': 'COMPUTE_ARM_BASE_SHIFT_TO_CAVITY',
                             'no_more_objects' : 'succeeded',
                             'no_more_cavities': 'succeeded'},
                remapping={'found_objects': 'best_matched_cavities'})


            # ARM AND BASE ALIGNMENT TO CAVITY PIPELINE
            smach.StateMachine.add('COMPUTE_ARM_BASE_SHIFT_TO_CAVITY', gbs.send_event([('/pregrasp_planner/event_in','e_start'),
                                                                                       ('/gripper_to_object_pose_error_calculator/event_in','e_start'),
                                                                                       ('/mcr_common/relative_displacement_calculator/event_in','e_start')]),
                transitions={'success':'WAIT_COMPUTE_ARM_BASE_SHIFT_TO_CAVITY'})

            smach.StateMachine.add('WAIT_COMPUTE_ARM_BASE_SHIFT_TO_CAVITY', gbs.wait_for_events([('/pregrasp_planner/event_out','e_success', True),
                                                                                                 ('/gripper_to_object_pose_error_calculator/event_out','e_success', True),
                                                                                                 ('/mcr_common/relative_displacement_calculator/event_out','e_done', True)]),
                transitions={'success':'STOP_COMPUTE_ARM_BASE_SHIFT_TO_CAVITY',
                             'timeout': 'STOP_ALL_COMPONENTS', #should we try again for compute arm base shift
                             'failure':'STOP_ALL_COMPONENTS'}) #should we try again for compute arm base shift 

            smach.StateMachine.add('STOP_COMPUTE_ARM_BASE_SHIFT_TO_CAVITY', gbs.send_event([('/gripper_to_object_pose_error_calculator/event_in','e_stop'),
                                                                                            ('/pregrasp_planner/event_in','e_stop'),
                                                                                            ('/mcr_common/relative_displacement_calculator/event_in','e_stop')]),
                transitions={'success':'ALIGN_BASE_WITH_CAVITY'})

            smach.StateMachine.add('ALIGN_BASE_WITH_CAVITY', gbs.send_event([('/mcr_navigation/relative_base_controller/event_in','e_start')]),
                transitions={'success':'WAIT_ALIGN_BASE_WITH_CAVITY'})

            smach.StateMachine.add('WAIT_ALIGN_BASE_WITH_CAVITY', gbs.wait_for_events([('/mcr_navigation/relative_base_controller/event_out','e_done', True),
                                                                                     ('/mcr_navigation/collision_velocity_filter/event_out','e_zero_velocities_forwarded', False)]),
                transitions={'success':'STOP_ALIGN_BASE_WITH_CAVITY',
                             'timeout': 'STOP_ALL_COMPONENTS',  #should we try again for compute and aligh arm and base to object 
                             'failure':'STOP_ALL_COMPONENTS'})  #should we try again for compute and aligh arm and base to object

            smach.StateMachine.add('STOP_ALIGN_BASE_WITH_CAVITY', gbs.send_event([('/mcr_navigation/relative_base_controller/event_in','e_stop')]),
                transitions={'success':'GRASP_OBJECT_FOR_CAVITY_FROM_PLTF' })
            
            # make sure that gripper is open  
            smach.StateMachine.add('GRASP_OBJECT_FOR_CAVITY_FROM_PLTF', ppts.grasp_obj_for_hole_from_pltf(),
                transitions={'object_grasped': 'MOVE_ARM_TO_CAVITY',
                             'no_more_obj_for_this_workspace': 'no_object_for_ppt_platform'})

            smach.StateMachine.add('GO_TO_PREGRASP', gms.move_arm('pre_grasp'),
                transitions={'succeeded': 'MOVE_ARM_TO_CAVITY',
                             'failed': 'MOVE_ARM_TO_CAVITY'})

            smach.StateMachine.add('MOVE_ARM_TO_CAVITY', gbs.send_event([('/planned_motion/event_in', 'e_start'),
            #smach.StateMachine.add('MOVE_ARM_TO_CAVITY', gbs.send_event([('/planned_motion/event_in', 'e_start')]),
                                                                          ('/pregrasp_arm_monitor/event_in', 'e_start')]),
                transitions={'success':'WAIT_MOVE_ARM_TO_CAVITY'})

            smach.StateMachine.add('WAIT_MOVE_ARM_TO_CAVITY', gbs.wait_for_events([('/pregrasp_arm_monitor/event_out','e_done', True)]),
  #          smach.StateMachine.add('WAIT_MOVE_ARM_TO_CAVITY', gbs.wait_for_events([('/planned_motion/event_out','e_success', True)]),
#                                                                             ('/pregrasp_arm_monitor/event_out', 'e_done', True)]),
                transitions={'success':'STOP_MOVE_ARM_TO_CAVITY',
                             'timeout': 'STOP_ALL_COMPONENTS',  #should we try again for compute and aligh arm and base to object 
                             'failure':'STOP_ALL_COMPONENTS'})  #should we try again for compute and aligh arm and base to object

            #smach.StateMachine.add('STOP_MOVE_ARM_TO_CAVITY', gbs.send_event([('/planned_motion/event_in','e_stop')]),
            smach.StateMachine.add('STOP_MOVE_ARM_TO_CAVITY', gbs.send_event([('/planned_motion/event_in','e_stop'),
                                                                             ('/pregrasp_arm_monitor/event_in', 'e_stop')]),
                transitions={'success':'MOVE_ARM_TO_INTERMEDIATE_POSE' })

            smach.StateMachine.add('PLACE_OBJECT', gms.linear_motion(operation='release'),
                transitions={'succeeded':'WIGGLE_ARM_LEFT',
                             'failed':'PLACE_OBJECT'})

            smach.StateMachine.add('WIGGLE_ARM_LEFT', ppts.ppt_wiggle_arm(wiggle_offset=-0.12),
                transitions={'succeeded':'WIGGLE_ARM_RIGHT',
                             'failed':'WIGGLE_ARM_RIGHT'})

            smach.StateMachine.add('WIGGLE_ARM_RIGHT', ppts.ppt_wiggle_arm(wiggle_offset=0.24),
                transitions={'succeeded':'MOVE_ARM_TO_INTERMEDIATE_POSE',
                             'failed':'MOVE_ARM_TO_INTERMEDIATE_POSE'})

            smach.StateMachine.add('MOVE_ARM_TO_INTERMEDIATE_POSE', gms.move_arm('look_at_workspace'),
                transitions={'succeeded': 'SELECT_OBJECT_TO_PLACE',
                             'failed': 'MOVE_ARM_TO_INTERMEDIATE_POSE'})
            
            #normal placing if arm and base alignment pipeline fails
            smach.StateMachine.add('STOP_ALL_COMPONENTS', gbs.send_event([('/gripper_to_object_pose_error_calculator/event_in','e_stop'),
                                                                          ('/pregrasp_arm_monitor/event_in', 'e_stop'),
                                                                          ('/mcr_common/relative_displacement_calculator/event_in','e_stop'),
                                                                          ('/mcr_navigation/relative_base_controller/event_in','e_stop'),
                                                                          ('/planned_motion/event_in','e_stop')]),
                transitions={'success':'SELECT_OBJECT_TO_PLACE'})
