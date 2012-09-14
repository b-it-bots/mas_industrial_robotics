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
                transitions={'succeeded':'ENABLE_PERCEPTION'})
            
            smach.StateMachine.add('ENABLE_PERCEPTION', enable_object_finder(),
                transitions={'succeeded':'FIND_OBJECT',  
                             'failed':'failed'})
            
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
            smach.StateMachine.add('FIND_DRAWER_AT_SOURCE', find_drawer(),
                transitions={'succeeded':'PLACE_BASE_IN_FRONT_OF_DRAWER',
                            'failed':'drawer_not_found'})
                    
            smach.StateMachine.add('PLACE_BASE_IN_FRONT_OF_DRAWER', place_base_in_front_of_object(),
                transitions={'succeeded':'GRASP_DRAWER',
                            'failed':'base_placement_failed'})
            
            smach.StateMachine.add('GRASP_DRAWER', grasp_drawer(),
                transitions={'succeeded':'drawer_grasped',
                            'failed':'FIND_DRAWER_AT_SOURCE'})

