#!/usr/bin/python
import roslib; roslib.load_manifest('raw_generic_states')
import rospy

import smach
import smach_ros

from generic_perception_states import *
from generic_manipulation_states import *
from generic_navigation_states import *

class sm_grasp_random_object(smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['object_grasped', 'failed'])
        
        with self:
            smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
                transitions={'succeeded':'FIND_OBJECT'})
                      
            smach.StateMachine.add('FIND_OBJECT', detect_object(),
                transitions={'succeeded':'GRASP_OBJECT',  
                             'failed':'failed'})
                
            smach.StateMachine.add('GRASP_OBJECT', grasp_random_object(),
                transitions={'succeeded':'object_grasped', 
                            'failed':'failed'})
            
            
class sm_grasp_drawer(smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['drawer_grasped', 'drawer_not_found', 'base_placement_failed'])
        
        with self:
            smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
                transitions={'succeeded':'FIND_DRAWER_AT_SOURCE'})
            
            smach.StateMachine.add('FIND_DRAWER_AT_SOURCE', find_drawer(),
                transitions={'found_drawer':'SELECT_DRAWER',
                            'no_drawer_found':'FIND_DRAWER_AT_SOURCE',
                            'srv_call_failed':'FIND_DRAWER_AT_SOURCE'})
            
            smach.StateMachine.add('SELECT_DRAWER', select_drawer('drawer_1'),
                transitions={'drawer_selected':'PLACE_BASE_IN_FRONT_OF_DRAWER',
                            'drawer_not_selected':'FIND_DRAWER_AT_SOURCE'})
                    
            smach.StateMachine.add('PLACE_BASE_IN_FRONT_OF_DRAWER', place_base_in_front_of_object(),
                transitions={'succeeded':'GRASP_DRAWER',
                             'srv_call_failed':'PLACE_BASE_IN_FRONT_OF_DRAWER'},
                remapping={'object_pose':'object_to_grasp'})
                
            smach.StateMachine.add('GRASP_DRAWER', grasp_drawer(),
                transitions={'succeeded':'drawer_grasped',
                            'failed':'FIND_DRAWER_AT_SOURCE'},
                remapping={'drawer_pose':'object_to_grasp'})

class select_drawer(smach.State):

    def __init__(self, drawer_name):
        smach.State.__init__(self, outcomes=['drawer_selected', 'drawer_not_selected'], 
                                   input_keys=['drawer_pose_list'], 
                                   output_keys=['object_to_grasp'])

        self.drawer_name = drawer_name
                
    def execute(self, userdata):   
                
        for drawer in userdata.drawer_pose_list:
            if drawer.name == self.drawer_name:
                userdata.object_to_grasp = drawer.pose
                return 'drawer_selected'
            
        return 'drawer_not_selected'