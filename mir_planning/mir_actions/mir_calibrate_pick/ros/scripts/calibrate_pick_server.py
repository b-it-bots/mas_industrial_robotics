#!/usr/bin/python3
import sys

import mcr_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms
import rospy
import smach
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
)
from smach_ros import ActionServerWrapper, IntrospectionServer

class WaitForEventFromUser(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo("Have you placed the mockup pose? press y/n ")
        input_from_user = input()
        if ('y' == input_from_user):
            rospy.loginfo("Received YES .. moving arm ")
            return 'success'
        else:
            rospy.loginfo("Received NO .. EXITING MOVING ARM ")
            return 'failure'


def main():
    # Open the container
    rospy.init_node("calibrate_pick_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )

    with sm:
        # publish a static frame which will be used as reference for perceived objs
        smach.StateMachine.add(
            "STOP_REFERENCE_FRAME_PUB",
            gbs.send_event([("/static_transform_publisher_node/event_in", "e_stop")]),
            transitions={"success": "OPEN_GRIPPER"},
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ARM"},
        )
        smach.StateMachine.add('MOVE_ARM', gms.move_arm('look_at_workspace_from_near'),
                transitions={'succeeded': 'PUBLISH_REFERENCE_FRAME',
                             'failed': 'MOVE_ARM'})
        # publish a static frame which will be used as reference for perceived objs
        smach.StateMachine.add(
            "PUBLISH_REFERENCE_FRAME",
            gbs.send_event([("/static_transform_publisher_node/event_in", "e_start")]),
            transitions={"success": "WAIT_FOR_EVENT_FROM_USER"},
        )

        smach.StateMachine.add('WAIT_FOR_EVENT_FROM_USER', WaitForEventFromUser(),
                 transitions={'success' : 'MOVE_ROBOT_AND_PICK',
			      'failure' : 'OVERALL_FAILED'})

        # whole body control command. It moves direct base controller and
        # calls pre-grasp planner, and (optionally) moves arm to object pose
        smach.StateMachine.add(
            "MOVE_ROBOT_AND_PICK",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_start")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=50,
            ),
            transitions={
                "success": "OVERALL_SUCCESS",
                "timeout": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
                "failure": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
            },
        )

        smach.StateMachine.add(
            "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
            gbs.send_event(
                [
                    ("/waypoint_trajectory_generation/event_in", "e_start"),
                    ("/wbc/event_in", "e_stop"),
                ]
            ),
            transitions={"success": "OVERALL_FAILED"},
        )


    # smach viewer
    if rospy.get_param("~viewer_enabled", False):
        sis = IntrospectionServer(
            "calibrate_pick_smach_viewer", sm, "/CALIBRATE_PICK_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="calibrate_pick_server",
        action_spec=GenericExecuteAction,
        wrapped_container=sm,
        succeeded_outcomes=["OVERALL_SUCCESS"],
        aborted_outcomes=["OVERALL_FAILED"],
        preempted_outcomes=["PREEMPTED"],
        goal_key="goal",
        feedback_key="feedback",
        result_key="result",
    )
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()


if __name__ == "__main__":
    main()
