#!/usr/bin/python

import rospy
import smach
import smach_ros

# import of generic states
import mir_states.common.navigation_states as gns
import mir_states.common.manipulation_states as gms
import mir_states.common.perception_states as gps

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
                        'cavity_pose',
                        'task_list'],
            output_keys=['base_pose_to_approach',
                         'last_grasped_obj',
                         'move_arm_to',
                         'move_base_by',
                         'rear_platform_free_poses',
                         'rear_platform_occupied_poses',
                         'selected_objects',
                         'cavity_pose',
                         'task_list'])

        with self:
            smach.StateMachine.add('ADD_WALLS_TO_PLANNING_SCENE', gms.update_static_elements_in_planning_scene("walls", "add"),
                transitions={'succeeded': 'ADJUST_POSE_WRT_WORKSPACE_AT_SOURCE'})

            # TODO: ENSURE WE HAVE ENOUGH SPACE IN THE ARENA 
            smach.StateMachine.add('ADJUST_POSE_WRT_WORKSPACE_AT_SOURCE', gns.adjust_to_workspace(0.4),
                transitions={'succeeded': 'SELECT_OBJECTS_TO_PLACE',
                             'failed': 'ADJUST_POSE_WRT_WORKSPACE_AT_SOURCE'})

            # Could to this while doing adjustment.
            smach.StateMachine.add('SELECT_OBJECTS_TO_PLACE', ppts.select_objects_to_place(),
                transitions={'objects_selected': 'CLEAR_CAVITIES',
                             'no_more_obj_for_this_workspace': 'no_object_for_ppt_platform'})

            smach.StateMachine.add('CLEAR_CAVITIES', ppts.clear_cavities(),
                transitions={'succeeded': 'LOOK_AT_WORKSPACE'})

            smach.StateMachine.add('LOOK_AT_WORKSPACE', gms.move_arm('out_of_view'),
                transitions={'succeeded': 'FIND_CAVITIES',
                             'failed': 'LOOK_AT_WORKSPACE'})

            # TODO: IF NOT ALL HAVE BEEN FOUND ? What then?
            smach.StateMachine.add('FIND_CAVITIES', gps.find_cavities(frame_id='/odom'),
                transitions={'succeeded': 'LOOK_LEFT',
                             'not_all_cavities_found': 'LOOK_LEFT',
                             'timeout': 'LOOK_LEFT'})

            smach.StateMachine.add('LOOK_LEFT', gms.move_arm('look_at_workspace_left'),
                transitions={'succeeded': 'FIND_CAVITIES_LEFT',
                             'failed': 'LOOK_LEFT'})

            # TODO: IF NOT ALL HAVE BEEN FOUND ? What then?
            smach.StateMachine.add('FIND_CAVITIES_LEFT', gps.find_cavities(frame_id='/odom'),
                transitions={'succeeded': 'LOOK_RIGHT',
                             'not_all_cavities_found': 'LOOK_RIGHT',
                             'timeout': 'LOOK_RIGHT'})

            smach.StateMachine.add('LOOK_RIGHT', gms.move_arm('look_at_workspace_right'),
                transitions={'succeeded': 'FIND_CAVITIES_RIGHT',
                             'failed': 'LOOK_RIGHT'})

            # TODO: IF NOT ALL HAVE BEEN FOUND ? What then?
            smach.StateMachine.add('FIND_CAVITIES_RIGHT', gps.find_cavities(frame_id='/odom'),
                transitions={'succeeded': 'SELECT_OBJECT_TO_PLACE',
                             'not_all_cavities_found': 'failed',
                             'timeout': 'failed'})

            smach.StateMachine.add('SELECT_OBJECT_TO_PLACE', ppts.select_object_to_place(),
                transitions={'object_selected': 'MOVE_ARM_TO_PREGRASP',
                             'no_more_objects' : 'succeeded',
                             'no_more_cavities': 'succeeded'})

#            smach.StateMachine.add('TRANSFORM_POSE_INTO_REFERENCE_FRAME', btts.transform_pose_to_reference_frame(frame_id='/base_link'),
#               transitions={'succeeded': 'MOVE_ARM_TO_PREGRASP',
#                             'tf_error': 'failed'},
#               remapping={'object_pose': 'cavity_pose'})

            smach.StateMachine.add('MOVE_ARM_TO_PREGRASP', gms.move_arm("pre_grasp"),
                transitions={'succeeded': 'COMPUTE_ARM_BASE_SHIFT_TO_OBJECT',
                             'failed': 'MOVE_ARM_TO_PREGRASP'})

            smach.StateMachine.add('COMPUTE_ARM_BASE_SHIFT_TO_OBJECT', btts.compute_arm_base_shift_to_object('/base_link', '/gripper_tip_link', 0.0, -0.02),
                transitions={'succeeded': 'MOVE_BASE_RELATIVE',
                             'tf_error': 'COMPUTE_ARM_BASE_SHIFT_TO_OBJECT'},
                remapping={'object_pose': 'cavity_pose'})

            smach.StateMachine.add('MOVE_BASE_RELATIVE', gns.move_base_relative(),
                transitions={'succeeded': 'GRASP_OBJECT_FOR_HOLE_FROM_PLTF',
                             'unreachable': 'GRASP_OBJECT_FOR_HOLE_FROM_PLTF',
                             'timeout': 'MOVE_BASE_RELATIVE'})

#            smach.StateMachine.add('MOVE_ARM_TO', gms.move_arm(),
#                        transitions={'succeeded': 'REMOVE_OBJECT_FROM_LIST',
#                                     'failed': 'MOVE_ARM_TO'})

            # TODO Check if we need intermediate poses
            smach.StateMachine.add('GRASP_OBJECT_FOR_HOLE_FROM_PLTF', ppts.grasp_obj_for_hole_from_pltf(),
                transitions={'object_grasped': 'MOVE_ARM',
                             'no_more_obj_for_this_workspace': 'no_object_for_ppt_platform'})

            ## DO WE NEED THIS !?!?! BECAUSE OTHERWISE IT WILL HIT THE CAMERA AND ALIGN THE OBJECT "PROPERLY" !?!??!
#            smach.StateMachine.add('MOVE_TO_INTERMEDIATE_POSE', gms.move_arm('platform_intermediate'),
#                transitions={'succeeded': 'MOVE_TO_PLACE_POSE',
#                             'failed': 'MOVE_TO_INTERMEDIATE_POSE'})

#            smach.StateMachine.add('MOVE_TO_PLACE_POSE', gms.move_arm('pre_grasp'),
#                transitions={'succeeded': 'MOVE_GRIPPER',
#                             'failed': 'MOVE_TO_PLACE_POSE'})

#            smach.StateMachine.add('SELECT_ARM_POSITION', ppts.select_arm_position(),
#                transitions={'arm_pose_selected': 'MOVE_ARM'})


            smach.StateMachine.add('MOVE_ARM', gms.move_arm(),
                transitions={'succeeded': 'RELEASE_OBJECT',
                             'failed': 'MOVE_ARM'})

            # TODO check if this, and next could cause infinite loop?
            smach.StateMachine.add('RELEASE_OBJECT', gms.linear_motion(operation='release'),
                transitions={'succeeded':'WIGGLE_ARM_LEFT',
                             'failed':'RELEASE_OBJECT'})

            smach.StateMachine.add('WIGGLE_ARM_LEFT', btts.ppt_wiggle_arm(wiggle_offset=-0.12),
                transitions={'succeeded':'WIGGLE_ARM_RIGHT',
                             'failed':'WIGGLE_ARM_RIGHT'})

            smach.StateMachine.add('WIGGLE_ARM_RIGHT', btts.ppt_wiggle_arm(wiggle_offset=0.24),
                transitions={'succeeded':'MOVE_ARM_TO_INTERMEDIATE_POSE',
                             'failed':'MOVE_ARM_TO_INTERMEDIATE_POSE'})

            smach.StateMachine.add('MOVE_ARM_TO_INTERMEDIATE_POSE', gms.move_arm('out_of_view'),
                transitions={'succeeded': 'SELECT_OBJECT_TO_PLACE',
                             'failed': 'MOVE_ARM_TO_INTERMEDIATE_POSE'})

