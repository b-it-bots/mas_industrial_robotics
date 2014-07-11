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
            input_keys=['all_found_holes',
                        'base_pose_to_approach',
                        'last_grasped_obj',
                        'move_arm_to',
                        'move_base_by',
                        'object_pose',
                        'rear_platform_free_poses',
                        'rear_platform_occupied_poses',
                        'selected_hole',
                        'selected_hole_pose',
                        'task_list'],
            output_keys=['all_found_holes',
                         'base_pose_to_approach',
                         'last_grasped_obj',
                         'move_arm_to',
                         'move_base_by',
                         'rear_platform_free_poses',
                         'rear_platform_occupied_poses',
                         'selected_hole',
                         'selected_hole_pose',
                         'task_list'])

        with self:
            smach.StateMachine.add('ADD_WALLS_TO_PLANNING_SCENE', gms.configure_planning_scene("walls", "add"),
                transitions={'succeeded': 'MOVE_ARM_OUT_OF_VIEW'})

            smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', gms.move_arm('out_of_view'),
                transitions={'succeeded': 'FIND_HOLES',
                             'failed': 'MOVE_ARM_OUT_OF_VIEW'})

            smach.StateMachine.add('FIND_HOLES', gps.find_holes(),
                transitions={'found_holes': 'SELECT_HOLE_TO_ADJUST_TO',
                             'found_no_holes': 'failed',      # perform a BTT placement
                             'timeout': 'failed'})

            smach.StateMachine.add('SELECT_HOLE_TO_ADJUST_TO', ppts.select_hole(),
                transitions={'hole_selected': 'COMPUTE_BASE_SHIFT_TO_OBJECT',
                             'no_match': 'failed',        # perform a BTT placement
                             'no_more_obj_for_this_workspace': 'no_object_for_ppt_platform'})

            smach.StateMachine.add('COMPUTE_BASE_SHIFT_TO_OBJECT', btts.compute_base_shift_to_object(),
                transitions={'succeeded': 'MOVE_BASE_RELATIVE',
                             'tf_error': 'COMPUTE_BASE_SHIFT_TO_OBJECT'},
                remapping={'object_pose': 'selected_hole'})  # TODO: can we do such a remapping, if not, we need a separate "selected_hole_pose" item in the userdata

            smach.StateMachine.add('MOVE_BASE_RELATIVE', gns.move_base_relative(),
                transitions={'succeeded': 'GRASP_OBJECT_FOR_HOLE_FROM_PLTF',
                             'timeout': 'MOVE_BASE_RELATIVE'})

            smach.StateMachine.add('GRASP_OBJECT_FOR_HOLE_FROM_PLTF', ppts.grasp_obj_for_hole_from_pltf(),
                transitions={'object_grasped': 'MOVE_TO_INTERMEDIATE_POSE',
                             'no_more_obj_for_this_workspace': 'no_object_for_ppt_platform'})

            ###### DO WE NEED THIS !?!?! BECAUSE OTHERWISE IT WILL HIT THE CAMERA AND ALIGN THE OBJECT "PROPERLY" !?!??!
            smach.StateMachine.add('MOVE_TO_INTERMEDIATE_POSE', gms.move_arm('platform_intermediate'),
                transitions={'succeeded': 'SELECT_ARM_POSITION',
                             'failed': 'MOVE_TO_INTERMEDIATE_POSE'})

            smach.StateMachine.add('SELECT_ARM_POSITION', ppts.select_arm_position(),
                transitions={'arm_pose_selected': 'MOVE_ARM'})

            smach.StateMachine.add('MOVE_ARM', gms.move_arm(),
                transitions={'succeeded': 'MOVE_GRIPPER',
                             'failed': 'MOVE_ARM'})

            smach.StateMachine.add('MOVE_GRIPPER', gms.control_gripper('open'),
                transitions={'succeeded': 'MOVE_ARM_TO_INTERMEDIATE_2'})

            smach.StateMachine.add('MOVE_ARM_TO_INTERMEDIATE_2', gms.move_arm('platform_intermediate'),
                transitions={'succeeded': 'succeeded',
                             'failed': 'MOVE_ARM_TO_INTERMEDIATE_2'})

