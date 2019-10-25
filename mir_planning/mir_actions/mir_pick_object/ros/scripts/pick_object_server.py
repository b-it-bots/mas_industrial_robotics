#!/usr/bin/python
import sys
import rospy
import smach

# import of generic states
import mir_states.common.manipulation_states as gms
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper, IntrospectionServer

from std_msgs.msg import String
from mir_planning_msgs.msg import GenericExecuteAction, GenericExecuteResult, GenericExecuteFeedback
from mir_actions.utils import Utils
        
#===============================================================================

class SelectObject(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['goal'],
                                    output_keys=['feedback', 'result'])
        self.publisher = rospy.Publisher(topic_name, String, queue_size=10)
        rospy.sleep(0.1) # time for the publisher to register in ros network

    def execute(self, userdata):
        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(current_state='SelectObject',
                                                   text='selecting object')

        obj = Utils.get_value_of(userdata.goal.parameters, 'object')
        self.publisher.publish(String(data=obj))
        rospy.sleep(0.2) # let the topic to survive for some time
        return 'succeeded'
        
#===============================================================================

class CheckIfLocationIsShelf(smach.State):
    def __init__(self ):
        smach.State.__init__(self,  outcomes=['shelf', 'not_shelf'], 
                                    input_keys=['goal'], output_keys=[])

    def execute(self, userdata):
        location = Utils.get_value_of(userdata.goal.parameters, 'location')
        if ((location == "SH01") or (location == "SH02")):
            return 'shelf'
        else:
            return 'not_shelf'

#===============================================================================

# TODO: also checks for shelf as closed loop is not yet implemented for top grasp
class CheckIfPickShouldBeClosedLoop(smach.State):
    def __init__(self ):
        smach.State.__init__(self,  outcomes=['closed_loop', 'normal'], 
                                    input_keys=['use_closed_loop_pick', 'goal'])

    def execute(self, userdata):
        location = Utils.get_value_of(userdata.goal.parameters, 'location')
        if userdata.use_closed_loop_pick and location in ['SH01', 'SH02']:
            return 'closed_loop'
        else:
            return 'normal'

#===============================================================================

def main():
    # Open the container
    rospy.init_node('pick_object_wbc_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['goal'],
            output_keys = ['feedback', 'result'])

    sm.userdata.use_closed_loop_pick = rospy.get_param('~use_closed_loop_pick', False)

    with sm:
        smach.StateMachine.add('SELECT_OBJECT', SelectObject(
                    '/mcr_perception/object_selector/input/object_name'),
                transitions={'succeeded':'GENERATE_OBJECT_POSE'})
      
        # generates a pose of object
        smach.StateMachine.add('GENERATE_OBJECT_POSE', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_selector/event_in','e_trigger')],
                event_out_list=[('/mcr_perception/object_selector/event_out','e_selected', True)],
                timeout_duration=10),
                transitions={'success':'CHECK_IF_SHELF',
                             'timeout':'OVERALL_FAILED',
                             'failure':'OVERALL_FAILED'})

        # Check if the current location is a shelf
        smach.StateMachine.add('CHECK_IF_SHELF', CheckIfLocationIsShelf(),
                transitions={'shelf':'MOVE_ARM_TO_INTERMEDIATE',
                             'not_shelf':'OPEN_GRIPPER'}) 

        # If location is a shelf, go to "shelf_intermediate" arm pose first
        smach.StateMachine.add('MOVE_ARM_TO_INTERMEDIATE', gms.move_arm("shelf_intermediate"),
                transitions={'succeeded' : 'OPEN_GRIPPER',
                             'failed' : 'MOVE_ARM_TO_INTERMEDIATE'})

        smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open'),
            transitions={'succeeded': 'SET_DBC_PARAMS'})

        smach.StateMachine.add('SET_DBC_PARAMS', gbs.set_named_config('dbc_pick_object'),
                transitions={'success':'MOVE_ROBOT_AND_PICK',
                             'timeout':'OVERALL_FAILED',
                             'failure':'OVERALL_FAILED'})

        # whole body control command. It moves direct base controller and
        # calls pre-grasp planner, and (optionally) moves arm to object pose
        smach.StateMachine.add('MOVE_ROBOT_AND_PICK', gbs.send_and_wait_events_combined(
                event_in_list=[('/wbc/event_in','e_start')],
                event_out_list=[('/wbc/event_out','e_success', True)],
                timeout_duration=50),
                transitions={'success':'CHECK_IF_CLOSED_LOOP_PICK',
                             'timeout':'STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE',
                             'failure':'STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE'})

        smach.StateMachine.add('STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE', gbs.send_event(
                    [('/waypoint_trajectory_generation/event_in','e_start'),
                     ('/wbc/event_in', 'e_stop')]),
                transitions={'success':'CHECK_IF_SHELF_FOR_FAILURE'})

        # Check if closed loop pick is to be used or not
        smach.StateMachine.add('CHECK_IF_CLOSED_LOOP_PICK', CheckIfPickShouldBeClosedLoop(),
                transitions={'closed_loop' : 'MOVE_ARM_TO_CLOSED_LOOP_PICK_START',
                             'normal' : 'CLOSE_GRIPPER'}) 
        
        # move arm to predefined start pose for closed loop pick
        smach.StateMachine.add('MOVE_ARM_TO_CLOSED_LOOP_PICK_START', gms.move_arm("closed_loop_pick_start"), 
               transitions={'succeeded':'MOVE_BASE_BASED_ON_CLOSED_LOOP', 
                            'failed':'MOVE_ARM_TO_CLOSED_LOOP_PICK_START'})
                               
        smach.StateMachine.add('MOVE_BASE_BASED_ON_CLOSED_LOOP', gbs.send_and_wait_events_combined(
                event_in_list=[('/closed_loop_pick_base_controller/event_in','e_start')],
                event_out_list=[('/closed_loop_pick_base_controller/event_out','e_done', True)],
                timeout_duration=30),
                transitions={'success':'CLOSE_GRIPPER',
                             'timeout':'STOP_MOVE_BASE_BASED_ON_CLOSED_LOOP',
                             'failure':'STOP_MOVE_BASE_BASED_ON_CLOSED_LOOP'})

        smach.StateMachine.add('STOP_MOVE_BASE_BASED_ON_CLOSED_LOOP', gbs.send_event(
                    [('/closed_loop_pick_base_controller/event_in','e_stop')]),
                transitions={'success':'CHECK_IF_SHELF_FOR_FAILURE'})

        smach.StateMachine.add('CLOSE_GRIPPER', gms.control_gripper('close'),
                transitions={'succeeded': 'CHECK_IF_SHELF_FINAL'})

        # Check if the current location is a shelf
        smach.StateMachine.add('CHECK_IF_SHELF_FINAL', CheckIfLocationIsShelf(),
                transitions={'shelf': 'MOVE_ARM_TO_POSTGRASP_FINAL',
                             'not_shelf': 'MOVE_ARM_TO_HOLD'}) 

        smach.StateMachine.add('MOVE_ARM_TO_POSTGRASP_FINAL', gms.move_arm("shelf_post_grasp"), 
                transitions={'succeeded':'MOVE_ARM_TO_INTERMEDIATE_FINAL', 
                             'failed':'MOVE_ARM_TO_POSTGRASP_FINAL'})

        smach.StateMachine.add('MOVE_ARM_TO_INTERMEDIATE_FINAL', gms.move_arm("shelf_intermediate"), 
                transitions={'succeeded':'VERIFY_OBJECT_GRASPED', 
                             'failed':'MOVE_ARM_TO_INTERMEDIATE_FINAL'})

        # move arm to HOLD position
        smach.StateMachine.add('MOVE_ARM_TO_HOLD', gms.move_arm("look_at_turntable"), 
                transitions={'succeeded':'VERIFY_OBJECT_GRASPED', 
                             'failed':'MOVE_ARM_TO_HOLD'})

        smach.StateMachine.add('VERIFY_OBJECT_GRASPED', gbs.send_and_wait_events_combined(                 
                event_in_list=[('/gripper_controller/grasp_monitor/event_in','e_trigger')],                                  
                event_out_list=[('/gripper_controller/grasp_monitor/event_out','e_object_grasped', True)],                          
                timeout_duration=5),                                                                                   
                transitions={'success':'OVERALL_SUCCESS',                                                       
                             'timeout':'OVERALL_FAILED',                                                       
                             'failure':'OVERALL_FAILED'})
        
        # Check if the current WS location is a shelf
        smach.StateMachine.add('CHECK_IF_SHELF_FOR_FAILURE', CheckIfLocationIsShelf(),
                transitions={'shelf': 'MOVE_ARM_TO_INTERMEDIATE_FAILURE',
                             'not_shelf': 'OVERALL_FAILED'}) 
        
        # If location is a shelf, go to "shelf_pre_grasp" arm pose 
        smach.StateMachine.add('MOVE_ARM_TO_INTERMEDIATE_FAILURE', gms.move_arm("shelf_intermediate"), 
                transitions={'succeeded':'OVERALL_FAILED',
                             'failed' : 'MOVE_ARM_TO_INTERMEDIATE_FAILURE'})
 
    # smach viewer
    if rospy.get_param('~viewer_enabled', False):
        sis = IntrospectionServer('pick_object_smach_viewer', sm, '/PICK_OBJECT_SMACH_VIEWER')
        sis.start()
    
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'wbc_pick_object_server',
        action_spec = GenericExecuteAction,
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'goal',
        feedback_key = 'feedback',
        result_key   = 'result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()
        
if __name__ == '__main__':
   main()
