#!/usr/bin/python

import rospy
import smach
import smach_ros

# import of generic states
import mir_states.common.basic_states as gbs
import mir_states.common.navigation_states as gns
import mir_states.common.manipulation_states as gms
import mir_states.common.perception_states as gps
import mir_states.common.perception_mockup_util as perception_mockup_util

#import robocup specific states
import mir_states.robocup.basic_transportation_test_states as btts


################################################################
class sub_sm_go_and_pick(smach.StateMachine):
    def __init__(self, use_mockup=None):
        smach.StateMachine.__init__(self, outcomes=['pose_skipped_but_platform_limit_reached',
                                                    'no_more_free_poses',
                                                    'no_more_free_poses_at_robot_platf',
                                                    'no_more_task_for_given_type'],
                                          input_keys=['base_pose_to_approach',
                                                      'desired_distance_to_workspace',
                                                      'found_objects',
                                                      'lasttask',
                                                      'move_arm_to',
                                                      'move_base_by',
						                              'next_arm_pose_index',
                                                      'object_pose',
                                                      'object_to_be_adjust_to',
                                                      'object_to_grasp',
                                                      'objects_to_be_grasped',
                                                      'prev_vs_result',
                                                      'rear_platform_free_poses',
                                                      'rear_platform_occupied_poses',
                                                      'recognized_objects',
                                                      'source_visits',
                                                      'task_list',
						                              'test',
                                                      'vscount'],
                                          output_keys=['base_pose_to_approach',
                                                       'found_objects',
                                                       'lasttask',
                                                       'move_arm_to',
                                                       'move_base_by',
						                               'next_arm_pose_index',
                                                       'object_to_be_adjust_to',
                                                       'object_to_grasp',
                                                       'objects_to_be_grasped',
                                                       'prev_vs_result',
                                                       'rear_platform_free_poses',
                                                       'rear_platform_occupied_poses',
                                                       'source_visits',
                                                       'task_list',
						                               'test',
                                                       'vscount'])

        self.use_mockup = use_mockup
        self.use_visual_servoing = True

        with self:
            smach.StateMachine.add('SELECT_SOURCE_SUBTASK', btts.select_btt_subtask(type="source"),
                transitions={'task_selected': 'MOVE_TO_SOURCE_LOCATION_SAFE',
                             'no_more_task_for_given_type': 'no_more_task_for_given_type'})

            # required before any call to gns.approach_pose, moves arm within footprint
            smach.StateMachine.add('MOVE_TO_SOURCE_LOCATION_SAFE', gms.move_arm('look_at_workspace_straight'),
                transitions={'succeeded': 'MOVE_TO_SOURCE_LOCATION',
                             'failed': 'MOVE_TO_SOURCE_LOCATION_SAFE'})

            smach.StateMachine.add('MOVE_TO_SOURCE_LOCATION', gns.approach_pose(),
                transitions={'succeeded': 'STOP_ALL_COMPONENTS_AT_START',
                             'failed': 'MOVE_TO_SOURCE_LOCATION'})

            smach.StateMachine.add('STOP_ALL_COMPONENTS_AT_START', gbs.send_event([('/gripper_to_object_pose_error_calculator/event_in','e_stop'),
                                                                          ('/mcr_common/relative_displacement_calculator/event_in','e_stop'),
                                                                          ('/pregrasp_planner/event_in','e_stop'),
                                                                          ('/mcr_navigation/relative_base_controller/event_in','e_stop'),
                                                                          ('/planned_motion/event_in','e_stop')]),
                transitions={'success':'ADJUST_POSE_WRT_WORKSPACE_AT_SOURCE'})


            smach.StateMachine.add('ADJUST_POSE_WRT_WORKSPACE_AT_SOURCE', gns.adjust_to_workspace(0.2),
                transitions={'succeeded':'SELECT_NEXT_LOOK_POSE',
                             'failed':'ADJUST_POSE_WRT_WORKSPACE_AT_SOURCE'})

            smach.StateMachine.add('SELECT_NEXT_LOOK_POSE', gms.select_arm_pose(['look_at_workspace_right_ppt', 'look_at_workspace_straight_ppt','look_at_workspace_left_ppt']),
                    transitions={'succeeded': 'LOOK_AROUND',
                    'failed': 'RECOGNIZE_OBJECTS'})

            smach.StateMachine.add('LOOK_AROUND', gms.move_arm(),
                transitions={'succeeded': 'RECOGNIZE_OBJECTS',
                             'failed': 'LOOK_AROUND'})

            smach.StateMachine.add('RECOGNIZE_OBJECTS', gps.find_objects(retries=1),
                transitions={'objects_found': 'ACCUMULATE_RECOGNIZED_LISTS',
                            'no_objects_found':'ACCUMULATE_RECOGNIZED_LISTS'},
                remapping={'found_objects':'recognized_objects'})

            smach.StateMachine.add('ACCUMULATE_RECOGNIZED_LISTS', gps.accumulate_recognized_objects_list(),
                transitions={'complete': 'SELECT_OBJECT_TO_BE_GRASPED',
                             'merged':'SELECT_NEXT_LOOK_POSE'})

            smach.StateMachine.add('SELECT_OBJECT_TO_BE_GRASPED', btts.select_object_to_be_grasped(),
                transitions={'obj_selected':'COMPUTE_ARM_BASE_SHIFT_TO_OBJECT',
                            'no_obj_selected':'SKIP_SOURCE_POSE',
                            'no_more_free_poses_at_robot_platf':'no_more_free_poses_at_robot_platf'})

            smach.StateMachine.add('COMPUTE_ARM_BASE_SHIFT_TO_OBJECT', gbs.send_event([('/pregrasp_planner/event_in','e_start'),
                                                                                       ('/gripper_to_object_pose_error_calculator/event_in','e_start'),
                                                                                       ('/mcr_common/relative_displacement_calculator/event_in','e_start')]),
                transitions={'success':'WAIT_COMPUTE_ARM_BASE_SHIFT_TO_OBJECT'})

            smach.StateMachine.add('WAIT_COMPUTE_ARM_BASE_SHIFT_TO_OBJECT', gbs.wait_for_events([('/pregrasp_planner/event_out','e_success', True),
                                                                                                 ('/gripper_to_object_pose_error_calculator/event_out','e_success', True),
                                                                                                 ('/mcr_common/relative_displacement_calculator/event_out','e_done', True)]),
                transitions={'success':'STOP_COMPUTE_ARM_BASE_SHIFT_TO_OBJECT',
                             'timeout': 'STOP_COMPUTE_ARM_BASE_SHIFT_TO_OBJECT_WITH_FAILURE',
                             'failure':'STOP_COMPUTE_ARM_BASE_SHIFT_TO_OBJECT_WITH_FAILURE'})

            smach.StateMachine.add('STOP_COMPUTE_ARM_BASE_SHIFT_TO_OBJECT', gbs.send_event([('/gripper_to_object_pose_error_calculator/event_in','e_stop'),
                                                                                            ('/pregrasp_planner/event_in','e_stop'),
                                                                                            ('/mcr_common/relative_displacement_calculator/event_in','e_stop')]),
                transitions={'success':'MOVE_ARM_AND_BASE_TO_OBJECT'})

            smach.StateMachine.add('STOP_COMPUTE_ARM_BASE_SHIFT_TO_OBJECT_WITH_FAILURE', gbs.send_event([('/gripper_to_object_pose_error_calculator/event_in','e_stop'),
                                                                                            ('/pregrasp_planner/event_in','e_stop'),
                                                                                            ('/mcr_common/relative_displacement_calculator/event_in','e_stop')]),
                transitions={'success':'DELETE_FROM_RECOGNIZED_OBJECTS'})

            smach.StateMachine.add('MOVE_ARM_AND_BASE_TO_OBJECT', gbs.send_event([('/mcr_navigation/relative_base_controller/event_in','e_start'),
                                                                                  ('/planned_motion/event_in', 'e_start')]),
                transitions={'success':'WAIT_ARM_AND_BASE_TO_OBJECT'})

            smach.StateMachine.add('WAIT_ARM_AND_BASE_TO_OBJECT', gbs.wait_for_events([('/mcr_navigation/relative_base_controller/event_out','e_done', True),
                                                                                       ('/mcr_navigation/collision_velocity_filter/event_out','e_zero_velocities_forwarded', False),
                                                                                       ('/planned_motion/event_out', 'e_success', True)]),
                transitions={'success':'STOP_MOVE_ARM_BASE_TO_OBJECT',
                             'timeout': 'STOP_MOVE_ARM_BASE_TO_OBJECT_WITH_FAILURE',
                             'failure':'STOP_MOVE_ARM_BASE_TO_OBJECT_WITH_FAILURE'})

            smach.StateMachine.add('STOP_MOVE_ARM_BASE_TO_OBJECT', gbs.send_event([('/mcr_navigation/relative_base_controller/event_in','e_stop'),
                                                                                   ('/planned_motion/event_in','e_stop')]),
#                transitions={'success':'ALIGN_WITH_OBJECT_LOOP'}) # uncomment to use visual servoing
                transitions={'success':'GRASP_OBJ'}) # comment out if using visual servoing

            smach.StateMachine.add('STOP_MOVE_ARM_BASE_TO_OBJECT_WITH_FAILURE', gbs.send_event([('/mcr_navigation/relative_base_controller/event_in','e_stop'),
                                                                                                ('/planned_motion/event_in','e_stop')]),
                transitions={'success':'DELETE_FROM_RECOGNIZED_OBJECTS'})

            smach.StateMachine.add('ALIGN_WITH_OBJECT_LOOP', gbs.loop_for_vs(1),
                                      transitions={'loop': 'START_VISUAL_SERVOING',
                                                   'continue': 'GRASP_OBJ'})

            smach.StateMachine.add('START_VISUAL_SERVOING', gbs.send_event([('/mir_controllers/visual_servoing/event_in','e_start')]),
                transitions={'success':'WAIT_VISUAL_SERVOING'})

            smach.StateMachine.add('WAIT_VISUAL_SERVOING', gbs.wait_for_events([('/mcr_monitoring/component_wise_pose_error_monitor/event_out','e_stop', True)], timeout_duration=30),
                transitions={'success':'STOP_VISUAL_SERVOING',
                             'timeout': 'STOP_VISUAL_SERVOING_WITH_FAILURE',
                             'failure':'STOP_VISUAL_SERVOING_WITH_FAILURE'})

            smach.StateMachine.add('STOP_VISUAL_SERVOING', gbs.send_event([('/mir_controllers/visual_servoing/event_in','e_stop')]),
                transitions={'success':'SET_VISUAL_SERVOING_SUCCESS'})

            smach.StateMachine.add('STOP_VISUAL_SERVOING_WITH_FAILURE', gbs.send_event([('/mir_controllers/visual_servoing/event_in','e_stop')]),
                transitions={'success':'SET_VISUAL_SERVOING_FAILURE'})

            smach.StateMachine.add('SET_VISUAL_SERVOING_SUCCESS', gbs.set_vs_status(status=True),
                transitions={'success':'OPEN_GRIPPER_FOR_APPROACH_OBJECT'})

            smach.StateMachine.add('SET_VISUAL_SERVOING_FAILURE', gbs.set_vs_status(status=False),
                transitions={'success':'SELECT_OBJECT_TO_BE_GRASPED'})

            smach.StateMachine.add('OPEN_GRIPPER_FOR_APPROACH_OBJECT', gms.control_gripper('open'),
                transitions={'succeeded':'APPROACH_OBJECT'})

            smach.StateMachine.add('APPROACH_OBJECT', gbs.send_event([('/guarded_approach_pose/coordinator/event_in', 'e_start')]),
                transitions={'success':'WAIT_APPROACH_OBJECT'})

            smach.StateMachine.add('WAIT_APPROACH_OBJECT', gbs.wait_for_events([('/guarded_approach_pose/coordinator/event_out','e_success', True)], timeout_duration=6),
                transitions={'success':'STOP_APPROACH_OBJECT',
                             'timeout': 'STOP_APPROACH_OBJECT',
                             'failure':'STOP_APPROACH_OBJECT'})

            smach.StateMachine.add('STOP_APPROACH_OBJECT', gbs.send_event([('/guarded_approach_pose/coordinator/event_in', 'e_stop')]),
                transitions={'success':'CLOSE_GRIPPER'})

            smach.StateMachine.add('CLOSE_GRIPPER', gms.control_gripper('close'),
                transitions={'succeeded':'PRE_PLACE_OBJ_ON_REAR_PLATFORM'})

            smach.StateMachine.add('GRASP_OBJ', gms.linear_motion(operation='grasp'),
                transitions={'succeeded':'PRE_PLACE_OBJ_ON_REAR_PLATFORM',
                             'failed':'DELETE_FROM_RECOGNIZED_OBJECTS'}) # TODO: this could loop

            smach.StateMachine.add('GRASP_OBJ_VISUAL_SERVOING', gms.linear_motion(operation='grasp', offset_x=-0.05),
                transitions={'succeeded':'PRE_PLACE_OBJ_ON_REAR_PLATFORM',
                             'failed':'DELETE_FROM_RECOGNIZED_OBJECTS'}) # TODO: this could loop

            smach.StateMachine.add('PRE_PLACE_OBJ_ON_REAR_PLATFORM', btts.pre_place_obj_on_rear_platform_btt(),
                transitions={'succeeded':'VERIFY_GRASPED',
                             'no_more_free_poses':'no_more_free_poses'})

            smach.StateMachine.add('VERIFY_GRASPED', gbs.send_event([('/gripper_controller/grasp_monitor/event_in', 'e_trigger')]),
                transitions={'success':'WAIT_FOR_VERIFY_GRASPED'})

            smach.StateMachine.add('WAIT_FOR_VERIFY_GRASPED', gbs.wait_for_events([('/gripper_controller/grasp_monitor/event_out','e_object_grasped', True)], timeout_duration=5),
                transitions={'success':'PLACE_OBJ_ON_REAR_PLATFORM',
                             'timeout':'OPEN_GRIPPER_FOR_GRASP_FAILURE',
                             'failure':'OPEN_GRIPPER_FOR_GRASP_FAILURE'})

            smach.StateMachine.add('OPEN_GRIPPER_FOR_GRASP_FAILURE', gms.control_gripper('open'),
                transitions={'succeeded':'DELETE_FROM_RECOGNIZED_OBJECTS'})

            # THESE ARE NEVER ENTERED?
            # It would mean we've grasped the object - and don't have somewhere to place it.
            # If we don't have somewhere to place it, we won't grasp in the first place?
            smach.StateMachine.add('PLACE_OBJ_ON_REAR_PLATFORM', btts.place_obj_on_rear_platform_btt(),
                transitions={'succeeded':'DELETE_FROM_RECOGNIZED_OBJECTS',
                             'no_more_free_poses':'no_more_free_poses'})

            smach.StateMachine.add('DELETE_FROM_RECOGNIZED_OBJECTS', btts.delete_from_recognized_objects(),
                transitions={'succeeded':'SELECT_OBJECT_TO_BE_GRASPED'},
                remapping={'object_to_delete': 'object_to_grasp'})

            smach.StateMachine.add('ADJUST_POSE_WRT_WORKSPACE_NEXT', gns.adjust_to_workspace(0.1),
                transitions={'succeeded':'SELECT_OBJECT_TO_BE_GRASPED',
                             'failed':'ADJUST_POSE_WRT_WORKSPACE_NEXT'})

            smach.StateMachine.add('NO_MORE_FREE_POSES_SAFE',gms.move_arm('look_at_workspace_straight'),
                transitions={'succeeded':'no_more_free_poses',
                             'failed':'SKIP_SOURCE_POSE_SAFE'})

		            # MISC STATES
            smach.StateMachine.add('SKIP_SOURCE_POSE_SAFE', gms.move_arm('look_at_workspace_straight'),
                transitions={'succeeded':'SKIP_SOURCE_POSE',
                             'failed':'SKIP_SOURCE_POSE_SAFE'})

            smach.StateMachine.add('SKIP_SOURCE_POSE', btts.skip_pose('source'),
                transitions={'pose_skipped':'SELECT_SOURCE_SUBTASK',
                             'pose_skipped_but_platform_limit_reached':'pose_skipped_but_platform_limit_reached'})



################################################################
class sub_sm_go_to_destination(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['destination_reached',
                                                    'overall_done'],
                                          input_keys=['base_pose_to_approach',
                                                      'desired_distance_to_workspace',
                                                      'objects_goal_configuration',
                                                      'objects_to_be_grasped',
                                                      'rear_platform_occupied_poses',
                                                      'task_list'],
                                          output_keys=['base_pose_to_approach',
                                                       'objects_goal_configuration',
                                                       'objects_to_be_grasped',
                                                       'rear_platform_occupied_poses',
                                                       'task_list'])

        with self:
            smach.StateMachine.add('REMOVE_WALLS_FROM_PLANNING_SCENE', gms.update_static_elements_in_planning_scene("walls", "remove"),
                transitions={'succeeded':'SELECT_DELIVER_WORKSTATION'})

            smach.StateMachine.add('SELECT_DELIVER_WORKSTATION', btts.select_delivery_workstation(),
                transitions={'success':'MOVE_TO_DESTINATION_LOCATION_SAFE',
                             'no_more_dest_tasks':'MOVE_TO_EXIT_SAFE'})

            smach.StateMachine.add('MOVE_TO_DESTINATION_LOCATION_SAFE', gms.move_arm('out_of_view'),
                transitions={'succeeded': 'MOVE_TO_DESTINATION_LOCATION',
                             'failed': 'MOVE_TO_DESTINATION_LOCATION_SAFE'})



            smach.StateMachine.add('MOVE_TO_DESTINATION_LOCATION', gns.approach_pose(),
                transitions={'succeeded':'ADJUST_POSE_WRT_WORKSPACE_AT_DESTINATION',
                             'failed':'MOVE_TO_DESTINATION_LOCATION'})

            smach.StateMachine.add('ADJUST_POSE_WRT_WORKSPACE_AT_DESTINATION', gns.adjust_to_workspace(0.12),
                transitions={'succeeded':'destination_reached',
                             'failed':'MOVE_TO_DESTINATION_LOCATION'})

            smach.StateMachine.add('MOVE_TO_EXIT_SAFE', gms.move_arm('out_of_view'),
                transitions={'succeeded': 'MOVE_TO_EXIT',
                             'failed': 'MOVE_TO_EXIT_SAFE'})


            smach.StateMachine.add('MOVE_TO_EXIT', gns.approach_pose("EXIT"),
                transitions={'succeeded':'overall_done',
                             'failed':'MOVE_TO_EXIT'})


################################################################
class sub_sm_place(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'no_more_obj_for_this_workspace'],
                                          input_keys=['base_pose_to_approach',
                                                      'destinaton_free_poses',
                                                      'last_grasped_obj',
                                                      'move_arm_to',
                                                      'obj_goal_configuration_poses',
                                                      'objects_goal_configuration',
                                                      'rear_platform_free_poses',
                                                      'rear_platform_occupied_poses',
                                                      'task_list'],
                                          output_keys=['base_pose_to_approach',
                                                      'destinaton_free_poses',
                                                      'last_grasped_obj',
                                                      'objects_goal_configuration',
                                                      'rear_platform_free_poses',
                                                      'rear_platform_occupied_poses',
                                                      'task_list'])

        with self:
            smach.StateMachine.add('ADD_WALLS_TO_PLANNING_SCENE', gms.update_static_elements_in_planning_scene("walls", "add"),
                transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF'})

            smach.StateMachine.add('GRASP_OBJECT_FROM_PLTF', btts.grasp_obj_from_pltf_btt(),
                transitions={'object_grasped':'REATTACH_OBJECT_TO_ROBOT',
                             'no_more_obj_for_this_workspace':'REMOVE_WALLS_FROM_PLANNING_SCENE'})

            smach.StateMachine.add('REATTACH_OBJECT_TO_ROBOT', gms.update_robot_planning_scene("unload"),
                transitions={'succeeded':'MOVE_TO_INTERMEDIATE_POSE'},
                remapping={'object': 'last_grasped_obj'})

            smach.StateMachine.add('MOVE_TO_INTERMEDIATE_POSE', gms.move_arm('platform_intermediate'),
                transitions={'succeeded':'PLACE_OBJ_IN_CONFIGURATION',
                             'failed':'MOVE_TO_INTERMEDIATE_POSE'})

            smach.StateMachine.add('PLACE_OBJ_IN_CONFIGURATION', btts.place_object_in_configuration_btt(),
                transitions={'succeeded':'DELETE_OBJECT_FROM_ROBOT_1',
                             'no_more_cfg_poses':'DELETE_OBJECT_FROM_ROBOT_2'})

            smach.StateMachine.add('DELETE_OBJECT_FROM_ROBOT_1', gms.update_robot_planning_scene("detach"),
                transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF'},
                remapping={'object': 'last_grasped_obj'})

            smach.StateMachine.add('DELETE_OBJECT_FROM_ROBOT_2', gms.update_robot_planning_scene("detach"),
                transitions={'succeeded':'MOVE_ARM_INSIDE_BASE_BOUNDARIES'},
                remapping={'object': 'object_to_grasp'})

            #smach.StateMachine.add('AVOID_WALLS_PRE_3', gms.move_arm('candle'),
            #    transitions={'succeeded': 'MOVE_ARM_INSIDE_BASE_BOUNDARIES',
            #                 'failed': 'AVOID_WALLS_PRE_3'})

            smach.StateMachine.add('MOVE_ARM_INSIDE_BASE_BOUNDARIES', gms.move_arm('platform_intermediate'),
                transitions={'succeeded':'succeeded',
                             'failed':'MOVE_ARM_INSIDE_BASE_BOUNDARIES'})

            smach.StateMachine.add('REMOVE_WALLS_FROM_PLANNING_SCENE', gms.update_static_elements_in_planning_scene("walls", "remove"),
                transitions={'succeeded':'no_more_obj_for_this_workspace'})


