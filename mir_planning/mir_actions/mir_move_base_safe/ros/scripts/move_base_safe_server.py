#!/usr/bin/env python
import mcr_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms
import param_server_utils
import rospy
import smach
import smach_ros
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
)
from smach_ros import ActionServerWrapper

# ===============================================================================


class AlignWithWorkspace(smach.State):  # inherit from the State base class
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )
        self.align = rospy.get_param("~align", False)
        if self.align:
            self.client = SimpleActionClient(
                "/align_with_workspace_server", AlignWithWorkspaceAction
            )
            self.client.wait_for_server()

    def execute(self, userdata):
        if not self.align:
            return "succeeded"
        else:
            return "succeeded"

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


# ===============================================================================


class CheckBeSafe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["safe", "unsafe"], input_keys=["goal"])

    def execute(self, userdata):
        be_safe = rospy.get_param("~be_safe", True)
        return "safe" if be_safe else "unsafe"


# ===============================================================================


class PrepareArmForNextAction(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed", "skipped"],
            input_keys=["goal"],
            output_keys=["feedback", "result", "move_arm_to"],
        )

    def execute(self, userdata):
        prepare_arm_for_next_action = rospy.get_param("~prepare_arm_for_next_action", True)
        if not prepare_arm_for_next_action:
            return "skipped"
        if Utils.get_value_of(userdata.goal.parameters, "next_action") == "PERCEIVE":
            arm_goal = "look_at_workspace_from_near"
        elif Utils.get_value_of(userdata.goal.parameters, "next_action") == "UNSTAGE":
            arm_goal = "stage_intermediate"
        else:
            return "skipped"
        # giving feedback to the user
        feedback = GenericExecuteFeedback()
        feedback.current_state = "MOVE_ARM"
        feedback.text = "[move_base_safe] Moving the arm to " + arm_goal
        userdata.feedback = feedback
        userdata.move_arm_to = arm_goal
        return "succeeded"


# ===============================================================================


class SetupMoveBase(smach.State):
    """
    Obtains nav goal from action lib and publishes a pose stamped to the specified topic
    """

    def __init__(self, topic_name):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed", "preempted"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )
        self.pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        self.dbc_pose_pub = rospy.Publisher(
            "/mcr_navigation/direct_base_controller/input_pose",
            PoseStamped,
            queue_size=1,
        )
        rospy.sleep(0.1)

    def execute(self, userdata):
        if self.preempt_requested():
            rospy.logwarn("preemption requested!!!")
            self.recall_preempt()  # reset preemption flag for next request
            return "preempted"

        base_goal = Utils.get_value_of(userdata.goal.parameters, "destination_location")
        base_orientation = Utils.get_value_of(
            userdata.goal.parameters, "destination_orientation"
        )
        rospy.loginfo("Destination: " + str(base_goal))

        pose = None
        pose = param_server_utils.get_pose_from_param_server(base_goal)
        if base_orientation is not None:
            pose.pose.orientation = param_server_utils.get_orientation_from_param_server(
                base_orientation
            )

        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()

        # giving feedback to the user
        feedback = GenericExecuteFeedback()
        feedback.current_state = "MOVE_BASE"
        feedback.text = "[move_base_safe] moving the base to " + base_goal
        userdata.feedback = feedback
        if pose:
            self.pub.publish(pose)
            self.dbc_pose_pub.publish(pose)
            return "succeeded"
        else:
            return "failed"

# ===============================================================================

class StartMoveBase(smach.State):
    def __init__(self,
            event_in_topic='/move_base_wrapper/event_in',
            event_out_topic='/move_base_wrapper/event_out',
            timeout_duration=50):
        smach.State.__init__(
            self,
            outcomes=["success", "failure", "timeout", "stopped", "preempted"]
        )
        self.pub = rospy.Publisher(event_in_topic, String, queue_size=1)
        self.sub = rospy.Subscriber(event_out_topic, String, self.event_cb)
        self.event = None
        self.timeout_duration = rospy.Duration.from_sec(timeout_duration)
        rospy.sleep(0.1)

    def execute(self, userdata):
        if self.preempt_requested():
            rospy.logwarn("preemption requested!!!")
            self.recall_preempt()  # reset preemption flag for next request
            return "preempted"
        self.event = None
        self.pub.publish('e_start')
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time < self.timeout_duration):
            if self.preempt_requested():
                rospy.logwarn("preemption requested!!!")
                self.recall_preempt()  # reset preemption flag for next request
                self.pub.publish('e_stop')
                return "preempted"
            if self.event is not None:
                if self.event == 'e_success':
                    return 'success'
                elif self.event == 'e_failure':
                    return 'failure'
                elif self.event == 'e_stopped':
                    return 'stopped'
            rospy.sleep(0.01)
        return 'timeout'

    def event_cb(self, msg):
        self.event = msg.data


# ===============================================================================

def transition_cb(*args, **kwargs):
    userdata = args[0]
    sm_state = args[1][0]
    publisher = args[2]
    publisher.publish(sm_state)

    feedback = GenericExecuteFeedback()
    feedback.current_state = sm_state
    userdata.feedback = feedback

def start_cb(*args, **kwargs):
    userdata = args[0]
    sm_state = args[1][0]
    publisher = args[2]
    publisher.publish(sm_state)

    feedback = GenericExecuteFeedback()
    feedback.current_state = sm_state
    userdata.feedback = feedback
# ===============================================================================


def main():
    rospy.init_node("move_base_safe_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED", "OVERALL_PREEMPTED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )

    with sm:
        smach.StateMachine.add(
            "CHECK_IF_BARRIER_TAPE_ENABLED",
            CheckBeSafe(),
            transitions={
                "safe": "START_BARRIER_TAPE_DETECTION",
                "unsafe": "SETUP_MOVE_BASE",
            },
        )

        smach.StateMachine.add(
            "START_BARRIER_TAPE_DETECTION",
            gbs.send_event(
                [("/mir_perception/barrier_tape_detection/event_in", "e_start",)]
            ),
            transitions={"success": "MOVE_ARM_TO_DETECT_BARRIER_TAPE"},
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_DETECT_BARRIER_TAPE",
            gms.move_arm("platform_middle_pre", use_moveit=False),
            transitions={"succeeded": "SETUP_MOVE_BASE", 
                         "failed": "MOVE_ARM_TO_DETECT_BARRIER_TAPE"},
        )
        # get pose from action lib as string, convert to pose stamped and publish
        smach.StateMachine.add(
            "SETUP_MOVE_BASE",
            SetupMoveBase("/move_base_wrapper/pose_in"),
            transitions={
                "succeeded": "SET_DIRECT_BASE_CONTROLLER_PARAMETERS",
                "failed": "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
                "preempted": "OVERALL_PREEMPTED",
            },
        )

        smach.StateMachine.add(
            "SET_DIRECT_BASE_CONTROLLER_PARAMETERS",
            gbs.set_named_config("dbc_move_base"),
            transitions={
                "success": "START_MOVE_BASE",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "START_MOVE_BASE",
            StartMoveBase(
                event_in_topic="/move_base_wrapper/event_in",
                event_out_topic="/move_base_wrapper/event_out",
                timeout_duration=50,
            ),
            transitions={
                "success": "PREPARE_ARM_FOR_NEXT_ACTION",
                "timeout": "RESET_BARRIER_TAPE",
                "failure": "RESET_BARRIER_TAPE",
                "stopped": "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
                "preempted": "OVERALL_PREEMPTED",
            },
        )

        smach.StateMachine.add(
            "RESET_BARRIER_TAPE",
            gbs.send_event(
                [("/mir_perception/barrier_tape_detection/event_in", "e_reset",),
                ("/move_base_wrapper/event_in", "e_stop",)]
            ),
            transitions={"success": "SETUP_MOVE_BASE_AGAIN"},
        )

        smach.StateMachine.add(
            "SETUP_MOVE_BASE_AGAIN",
            SetupMoveBase("/move_base_wrapper/pose_in"),
            transitions={
                "succeeded": "START_MOVE_BASE_AGAIN",
                "failed": "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
                "preempted": "OVERALL_PREEMPTED",
            },
        )

        smach.StateMachine.add(
            "START_MOVE_BASE_AGAIN",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/move_base_wrapper/event_in", "e_start")],
                event_out_list=[("/move_base_wrapper/event_out", "e_success", True)],
                timeout_duration=50,
            ),
            transitions={
                "success": "PREPARE_ARM_FOR_NEXT_ACTION",
                "timeout": "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
                "failure": "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
            },
        )

        # send arm to a position depending on next action
        smach.StateMachine.add(
            "PREPARE_ARM_FOR_NEXT_ACTION",
            PrepareArmForNextAction(),
            transitions={
                "succeeded": "MOVE_ARM_FOR_NEXT_ACTION",
                "skipped": "REACH_DESTINATION_PRECISELY",
                "failed": "PREPARE_ARM_FOR_NEXT_ACTION",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_FOR_NEXT_ACTION",
            gms.move_arm(blocking=False),
            transitions={"succeeded": "REACH_DESTINATION_PRECISELY", 
                         "failed": "REACH_DESTINATION_PRECISELY"},
        )

        # call direct base controller to fine adjust the base to the final desired pose
        # (navigation tolerance is set to a wide tolerance)
        smach.StateMachine.add(
            "REACH_DESTINATION_PRECISELY",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_in",
                        "e_start",
                    ),
                ],
                event_out_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=5,
            ),  # this is a tradeoff between speed and accuracy, set a higher value for accuracy increase
            transitions={
                "success": "STOP_CONTROLLER_WITH_SUCCESS",
                "timeout": "STOP_CONTROLLER_WITH_SUCCESS",
                "failure": "STOP_CONTROLLER_WITH_FAILURE",
            },
        )

        # stop controller with success
        smach.StateMachine.add(
            "STOP_CONTROLLER_WITH_SUCCESS",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_in",
                        "e_stop",
                    )
                ],
                event_out_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_out",
                        "e_stopped",
                        True,
                    )
                ],
                timeout_duration=1,
            ),
            transitions={
                "success": "STOP_BARRIER_TAPE_DETECTION_WITH_SUCCESS",
                "timeout": "STOP_BARRIER_TAPE_DETECTION_WITH_SUCCESS",
                "failure": "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
            },
        )

        # stop controller with failure
        smach.StateMachine.add(
            "STOP_CONTROLLER_WITH_FAILURE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_in",
                        "e_stop",
                    )
                ],
                event_out_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_out",
                        "e_stopped",
                        True,
                    )
                ],
                timeout_duration=1,
            ),
            transitions={
                "success": "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
                "timeout": "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
                "failure": "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
            },
        )

        smach.StateMachine.add(
            "STOP_BARRIER_TAPE_DETECTION_WITH_FAILURE",
            gbs.send_event(
                [("/mir_perception/barrier_tape_detection/event_in", "e_stop"),
                ("/move_base_wrapper/event_in", "e_stop",)]
            ),
            transitions={"success": "OVERALL_FAILED"},
        )

        smach.StateMachine.add(
            "STOP_BARRIER_TAPE_DETECTION_WITH_SUCCESS",
            gbs.send_event(
                [("/mir_perception/barrier_tape_detection/event_in", "e_stop")]
            ),
            transitions={"success": "ALIGN_WITH_WORKSPACE"},
        )

        smach.StateMachine.add(
            "ALIGN_WITH_WORKSPACE",
            AlignWithWorkspace(),
            transitions={"succeeded": "OVERALL_SUCCESS", "failed": "OVERALL_SUCCESS",},
        )

    state_publisher = rospy.Publisher('~current_state', String, queue_size=1)
    sm.register_transition_cb(transition_cb, [state_publisher])
    sm.register_start_cb(start_cb, [state_publisher])

    # smach viewer
    if rospy.get_param("~viewer_enabled", False):
        sis = smach_ros.IntrospectionServer(
            "move_base_safe_smach_viewer", sm, "/MOVE_BASE_SAFE_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="move_base_safe_server",
        action_spec=GenericExecuteAction,
        wrapped_container=sm,
        succeeded_outcomes=["OVERALL_SUCCESS"],
        aborted_outcomes=["OVERALL_FAILED"],
        preempted_outcomes=["OVERALL_PREEMPTED"],
        goal_key="goal",
        feedback_key="feedback",
        result_key="result",
    )
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()


if __name__ == "__main__":
    main()
