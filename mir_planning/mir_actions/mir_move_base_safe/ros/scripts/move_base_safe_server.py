#!/usr/bin/env python
import rospy
import smach
import smach_ros

import mir_states.common.manipulation_states as gms # move the arm
import mcr_states.common.basic_states as gbs
from mir_actions.utils import Utils

# action lib
from smach_ros import ActionServerWrapper
from mir_planning_msgs.msg import GenericExecuteAction, GenericExecuteFeedback, GenericExecuteResult

from actionlib import SimpleActionClient

# to publish pose stamped from param server
from geometry_msgs.msg import PoseStamped
import param_server_utils

#===============================================================================

class AlignWithWorkspace(smach.State): # inherit from the State base class
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded','failed'],
                                    input_keys=['goal'],
                                    output_keys=['feedback', 'result'])
        self.align = rospy.get_param("~align", False)
        if self.align:
            self.client = SimpleActionClient('/align_with_workspace_server', AlignWithWorkspaceAction)
            self.client.wait_for_server()

    def execute(self, userdata):
        if not self.align:
            return 'succeeded'
        else:
            return 'succeeded'

        # goal = AlignWithWorkspaceActionGoal()

        # goal.goal.destination_location = userdata.goal.destination_location
        # timeout = 10.0
        # rospy.loginfo('Sending action lib goal to align_with_workspace_server, destination : ' + goal.goal.destination_location)
        # self.client.send_goal(goal.goal)
        # self.client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        # self.client.cancel_goal()
        # server_result = self.client.get_result()
        # if server_result:
        #     rospy.loginfo("Alignment result: " + str(server_result.success));
        # else:
        #     rospy.loginfo("Alignment result: failed");
        #     return 'failed'
        # if server_result.success:
        #     return 'succeeded'
        # else:
        #     return 'failed'

#===============================================================================

class CheckDontBeSafe(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['safe','unsafe'],
                                    input_keys=['goal'])

    def execute(self, userdata):
        dont_be_safe_flag = Utils.get_value_of(userdata.goal.parameters, 'dont_be_safe')
        if dont_be_safe_flag is not None and dont_be_safe_flag.upper() == 'TRUE':
            return 'unsafe'
        else:
            return 'safe'


#===============================================================================

class SetupMoveArmAfterMoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded','failed', 'skipped'],
                                    input_keys=['goal'],
                                    output_keys=['feedback', 'result', 'move_arm_to'])

    def execute(self, userdata):
        if Utils.get_value_of(userdata.goal.parameters, 'next_action') == 'PERCEIVE':
            arm_goal = 'look_at_workspace_from_far'
        elif Utils.get_value_of(userdata.goal.parameters, 'next_action') == 'UNSTAGE':
            arm_goal = "platform_middle_pre"
        else:
            return 'skipped'
        # giving feedback to the user
        feedback = GenericExecuteFeedback()
        feedback.current_state = 'MOVE_ARM'
        feedback.text='[move_base_safe] Moving the arm to ' + arm_goal
        userdata.feedback = feedback
        userdata.move_arm_to = arm_goal
        return 'succeeded'

#===============================================================================

class SetupMoveBase(smach.State):
    """
    Obtains nav goal from action lib and publishes a pose stamped to the specified topic
    """
    def __init__(self, topic_name):
        smach.State.__init__(self,  outcomes=['succeeded','failed','preempted'],
                                    input_keys=['goal'],
                                    output_keys=['feedback', 'result'])
        self.pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        self.dbc_pose_pub = rospy.Publisher('/mcr_navigation/direct_base_controller/input_pose',
                                            PoseStamped,
                                            queue_size=1)
        rospy.sleep(0.1)

    def execute(self, userdata):
        if self.preempt_requested():
            rospy.logwarn('preemption requested!!!')
            self.recall_preempt() # reset preemption flag for next request
            return 'preempted'

        base_goal = Utils.get_value_of(userdata.goal.parameters, 'destination_location')
        base_orientation = Utils.get_value_of(userdata.goal.parameters, 'destination_orientation')
        rospy.loginfo("Destination: " + str(base_goal))

        pose = None
        pose = param_server_utils.get_pose_from_param_server(base_goal)
        if base_orientation is not None:
            pose.pose.orientation = param_server_utils.get_orientation_from_param_server(base_orientation)

        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()

        # giving feedback to the user
        feedback = GenericExecuteFeedback()
        feedback.current_state = 'MOVE_BASE'
        feedback.text = '[move_base_safe] moving the base to ' + base_goal
        userdata.feedback = feedback
        if pose:
            self.pub.publish(pose)
            self.dbc_pose_pub.publish(pose)
            return 'succeeded'
        else:
            return 'failed'

#===============================================================================

def main():
    rospy.init_node('move_base_safe_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED','OVERALL_PREEMPTED'],
            input_keys = ['goal'],
            output_keys = ['feedback', 'result'])
    with sm:
        smach.StateMachine.add('CHECK_DONT_BE_SAFE', CheckDontBeSafe(),
                transitions={'safe': 'START_BARRIER_TAPE_DETECTION',
                             'unsafe': 'SETUP_MOVE_BASE'})

        smach.StateMachine.add('START_BARRIER_TAPE_DETECTION', gbs.send_event([('/mir_perception/barrier_tape_detection/event_in', 'e_start')]),
                transitions={'success':'MOVE_ARM'})

        smach.StateMachine.add('MOVE_ARM', gms.move_arm('barrier_tape', blocking=False),
                transitions={'succeeded': 'SETUP_MOVE_BASE',
                             'failed': 'MOVE_ARM'})
        # get pose from action lib as string, convert to pose stamped and publish
        smach.StateMachine.add('SETUP_MOVE_BASE', SetupMoveBase('/move_base_wrapper/pose_in'),
                transitions={'succeeded': 'SET_DBC_PARAMS',
                             'failed': 'STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE',
                             'preempted':'OVERALL_PREEMPTED'})

        smach.StateMachine.add('SET_DBC_PARAMS', gbs.set_named_config('dbc_move_base'),
                transitions={'success':'MOVE_BASE',
                             'timeout':'OVERALL_FAILED',
                             'failure':'OVERALL_FAILED'})

        # send event_in to move base to a pose
        # add events to mir_look at and mcr_converter
        smach.StateMachine.add('MOVE_BASE', gbs.send_and_wait_events_combined(
                event_in_list=[('/move_base_wrapper/event_in','e_start'),
                               ('/mcr_common/twist_to_motion_direction_conversion/event_in','e_start'),
                               ('/mir_manipulation/look_at_point/event_in','e_start')],
                event_out_list=[('/move_base_wrapper/event_out','e_success',True)],
                timeout_duration=50),
                transitions={'success':'SETUP_MOVE_ARM_AFTER_MOVE_BASE',
                             'timeout':'RESET_BARRIER_TAPE',
                             'failure':'RESET_BARRIER_TAPE'})

        smach.StateMachine.add('RESET_BARRIER_TAPE', gbs.send_event([('/mir_perception/barrier_tape_detection/event_in', 'e_reset')]),
                transitions={'success':'SETUP_MOVE_BASE_AGAIN'})

        smach.StateMachine.add('SETUP_MOVE_BASE_AGAIN', SetupMoveBase('/move_base_wrapper/pose_in'),
                transitions={'succeeded': 'MOVE_BASE_AGAIN',
                             'failed': 'STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE',
                             'preempted':'OVERALL_PREEMPTED'})

        smach.StateMachine.add('MOVE_BASE_AGAIN', gbs.send_and_wait_events_combined(
                event_in_list=[('/move_base_wrapper/event_in','e_start'),
                               ('/mcr_common/twist_to_motion_direction_conversion/event_in','e_start'),
                               ('/mir_manipulation/look_at_point/event_in','e_start')],
                event_out_list=[('/move_base_wrapper/event_out','e_success',True)],
                timeout_duration=50),
                transitions={'success':'SETUP_MOVE_ARM_AFTER_MOVE_BASE',
                             'timeout':'STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE',
                             'failure':'STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE'})

        # send arm to a position depending on next action
        smach.StateMachine.add('SETUP_MOVE_ARM_AFTER_MOVE_BASE', SetupMoveArmAfterMoveBase(),
                transitions={'succeeded': 'MOVE_ARM_AFTER_MOVE_BASE',
                             'skipped': 'ADJUST_BASE',
                             'failed': 'SETUP_MOVE_ARM_AFTER_MOVE_BASE'})

        smach.StateMachine.add('MOVE_ARM_AFTER_MOVE_BASE', gms.move_arm(blocking=False),
                        transitions={'succeeded': 'ADJUST_BASE',
                                    'failed': 'ADJUST_BASE'})

        # call direct base controller to fine adjust the base to the final desired pose
        # (navigation tolerance is set to a wide tolerance)
        smach.StateMachine.add('ADJUST_BASE', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_start'),
                               ('/mir_manipulation/look_at_point/event_in','e_stop')],
                event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_success', True)],
                timeout_duration=5), # this is a tradeoff between speed and accuracy, set a higher value for accuracy increase
                transitions={'success':'STOP_CONTROLLER_WITH_SUCCESS',
                             'timeout':'STOP_CONTROLLER_WITH_SUCCESS',
                             'failure':'STOP_CONTROLLER_WITH_FAILURE'})

        # stop controller with success
        smach.StateMachine.add('STOP_CONTROLLER_WITH_SUCCESS', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_stop')],
                event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_stopped', True)],
                timeout_duration=1),
                transitions={'success':'STOP_BARRIER_TAPE_DETECTION_WITH_SUCCESS',
                             'timeout':'STOP_BARRIER_TAPE_DETECTION_WITH_SUCCESS',
                             'failure':'STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE'})

        # stop controller with failure
        smach.StateMachine.add('STOP_CONTROLLER_WITH_FAILURE', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_stop')],
                event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_stopped', True)],
                timeout_duration=1),
                transitions={'success':'STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE',
                             'timeout':'STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE',
                             'failure':'STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE'})

        smach.StateMachine.add('STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE', gbs.send_event([('/mir_perception/barrier_tape_detection/event_in', 'e_stop')]),
                transitions={'success':'OVERALL_FAILED'})

        smach.StateMachine.add('STOP_BARRIER_TAPE_DETECTION_WITH_SUCCESS', gbs.send_event([('/mir_perception/barrier_tape_detection/event_in', 'e_stop')]),
                transitions={'success':'ALIGN_WITH_WORKSPACE'})

        smach.StateMachine.add('ALIGN_WITH_WORKSPACE', AlignWithWorkspace(),
                               transitions={'succeeded':'OVERALL_SUCCESS',
                                            'failed': 'OVERALL_SUCCESS'})

    # smach viewer
    if rospy.get_param('~viewer_enabled', False):
        sis = smach_ros.IntrospectionServer('move_base_safe_smach_viewer', sm, '/MOVE_BASE_SAFE_SMACH_VIEWER')
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'move_base_safe_server',
        action_spec = GenericExecuteAction,
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['OVERALL_PREEMPTED'],
        goal_key     = 'goal',
        feedback_key = 'feedback',
        result_key   = 'result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()
