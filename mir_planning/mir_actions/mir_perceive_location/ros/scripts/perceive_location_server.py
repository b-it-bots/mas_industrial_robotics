#!/usr/bin/python

import rospy
import smach

# import of generic states
import mir_states.common.navigation_states as gns
import mir_states.common.manipulation_states as gms
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper, IntrospectionServer
from mir_actions.utils import Utils
from mir_planning_msgs.msg import GenericExecuteAction, GenericExecuteResult, GenericExecuteFeedback
from diagnostic_msgs.msg import KeyValue

# perception object list
from mas_perception_msgs.msg import ObjectList

#===============================================================================

class SetupMoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pose_set','tried_all'],
                                   input_keys=['arm_pose_index', 'arm_pose_list'],
                                   output_keys=['arm_pose_index', 'move_arm_to'])

    def execute(self, userdata):
        if userdata.arm_pose_index >= len(userdata.arm_pose_list):
            return 'tried_all'
        # set arm pose to next pose in list
        userdata.move_arm_to = userdata.arm_pose_list[userdata.arm_pose_index]
        userdata.arm_pose_index += 1

        return 'pose_set'

#===============================================================================

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                   input_keys=[],
                                   output_keys=['feedback', 'result', 'arm_pose_index'])

    def execute(self, userdata):
        userdata.arm_pose_index = 0 # reset arm position for new request

        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(current_state='Setup',
                                                   text='Setting up')
        return 'succeeded'

#===============================================================================

class PopulateResultWithObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['goal'],
                                    output_keys=['feedback', 'result'])
        self.objects_sub = rospy.Subscriber('/mcr_perception/object_list_merger/output_object_list',
                                            ObjectList, self.objects_callback)
        self.perceived_obj_names = []

    def objects_callback(self, msg):
        self.perceived_obj_names = [str(obj.name) for obj in msg.objects]

    def execute(self, userdata):
        result = GenericExecuteResult()
        for i, obj in enumerate(self.perceived_obj_names):
            result.results.append(KeyValue(key='obj_'+str(i+1), value=obj))
        userdata.result = result

        userdata.feedback = GenericExecuteFeedback() #place holder

        self.perceived_obj_names = [] # clear perceived objects for next call
        return 'succeeded'

#===============================================================================

def main():
    rospy.init_node('perceive_location_server')
    sleep_time = rospy.get_param('~sleep_time', 1.0)
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['goal'],
            output_keys = ['feedback', 'result'])
    # Open the container
    sm.userdata.arm_pose_list = ['look_at_workspace_from_near',
                                 'look_at_workspace_from_near_left',
                                 'look_at_workspace_from_near_right']
    sm.userdata.arm_pose_index = 0
    with sm:
        # approach to platform
        smach.StateMachine.add('SETUP', Setup(),
                transitions={'succeeded':'PUBLISH_REFERENCE_FRAME'})
        
        # publish a static frame which will be used as reference for perceived objs
        smach.StateMachine.add('PUBLISH_REFERENCE_FRAME', gbs.send_event
                ([('/static_transform_publisher_node/event_in', 'e_start')]),
                transitions={'success':'START_OBJECT_LIST_MERGER'})

        smach.StateMachine.add('START_OBJECT_LIST_MERGER', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/event_in', 'e_start'),
                               ('/mcr_perception/object_selector/event_in', 'e_start')],
                event_out_list=[('/mcr_perception/object_list_merger/event_out', 'e_started', True)],
                timeout_duration=5),
                transitions={'success': 'SET_APPROPRIATE_ARM_POSE',
                             'timeout': 'OVERALL_FAILED',
                             'failure': 'OVERALL_FAILED'})

        smach.StateMachine.add('SET_APPROPRIATE_ARM_POSE', SetupMoveArm(),
                transitions={'pose_set': 'MOVE_ARM',
                             'tried_all': 'POPULATE_RESULT_WITH_OBJECTS'})

        # move arm to appropriate position
        smach.StateMachine.add('MOVE_ARM', gms.move_arm_and_gripper('open'),
                transitions={'succeeded': 'START_OBJECT_RECOGNITION',
                             'failed': 'MOVE_ARM'})
        
        # New perception pipeline state machine
        smach.StateMachine.add('START_OBJECT_RECOGNITION', gbs.send_and_wait_events_combined(
                event_in_list=[('/mir_perception/multimodal_object_recognition/event_in', 'e_start')],
                event_out_list=[('/mir_perception/multimodal_object_recognition/event_out', 'e_done', True)],
                timeout_duration=10),
                transitions={'success': 'STOP_RECOGNITION',
                             'timeout': 'OVERALL_FAILED',
                             'failure': 'OVERALL_FAILED'})
         
        
        smach.StateMachine.add('STOP_RECOGNITION', gbs.send_and_wait_events_combined(
                event_in_list=[('/mir_perception/multimodal_object_recognition/event_in', 'e_stop')],
                event_out_list=[('/mir_perception/multimodal_object_recognition/event_out', 'e_stopped', True)],
                timeout_duration=5),
                transitions={'success': 'STOP_OBJECT_LIST_MERGER',
                             'timeout': 'STOP_OBJECT_LIST_MERGER',
                             'failure': 'STOP_OBJECT_LIST_MERGER'})

        smach.StateMachine.add('STOP_OBJECT_LIST_MERGER', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/event_in', 'e_stop')],
                event_out_list=[('/mcr_perception/object_list_merger/event_out', 'e_stopped', True)],
                timeout_duration=5),
                transitions={'success': 'PUBLISH_MERGED_OBJECT_LIST',
                             'timeout': 'OVERALL_FAILED',
                             'failure': 'OVERALL_FAILED'})
        
        smach.StateMachine.add('PUBLISH_MERGED_OBJECT_LIST', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/event_in', 'e_trigger')],
                event_out_list=[('/mcr_perception/object_list_merger/event_out', 'e_done', True)],
                timeout_duration=5),
                transitions={'success': 'CHECK_IF_OBJECTS_FOUND',
                             'timeout': 'CHECK_IF_OBJECTS_FOUND',
                             'failure': 'CHECK_IF_OBJECTS_FOUND'})
        
        smach.StateMachine.add('CHECK_IF_OBJECTS_FOUND', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/object_found_event_in', 'e_trigger')],
                event_out_list=[('/mcr_perception/object_list_merger/object_found_event_out', 'e_objects_found', True)],
                timeout_duration=10.0),
                transitions={'success': 'POPULATE_RESULT_WITH_OBJECTS',
                             'timeout': 'OVERALL_FAILED',
                             'failure': 'SET_APPROPRIATE_ARM_POSE'})

        # populate action server result with perceived objects
        smach.StateMachine.add('POPULATE_RESULT_WITH_OBJECTS', PopulateResultWithObjects(), 
                transitions={'succeeded':'OVERALL_SUCCESS'})

    # smach viewer
    if rospy.get_param('~viewer_enabled', False):
        sis = IntrospectionServer('perceive_location_smach_viewer', sm, '/PERCEIVE_LOCATION_SMACH_VIEWER')
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'perceive_location_server',
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
