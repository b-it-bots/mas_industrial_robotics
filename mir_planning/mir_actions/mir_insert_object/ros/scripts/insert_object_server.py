#!/usr/bin/python
import mcr_states.common.basic_states as gbs
import mir_states.common.action_states as gas
import mir_states.common.manipulation_states as gms
import rospy
import smach
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
)
from smach_ros import ActionServerWrapper, IntrospectionServer
from std_msgs.msg import String

# ===============================================================================


class SelectObject(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )
        self.publisher = rospy.Publisher(topic_name, String, queue_size=10)
        rospy.sleep(0.1)  # time for the publisher to register in ros network

    def execute(self, userdata):
        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="SelectObject", text="selecting object"
        )

        obj = Utils.get_value_of(userdata.goal.parameters, "hole")
        peg = Utils.get_value_of(userdata.goal.parameters, "peg") # peg is the object to be inserted
        self.publisher.publish(String(data=obj))
        rospy.sleep(0.2)  # let the topic survive for some time
        return "succeeded"


# ===============================================================================

def transition_cb(*args, **kwargs):
    userdata = args[0]
    sm_state = args[1][0]

    feedback = GenericExecuteFeedback()
    feedback.current_state = sm_state
    userdata.feedback = feedback

def start_cb(*args, **kwargs):
    userdata = args[0]
    sm_state = args[1][0]

    feedback = GenericExecuteFeedback()
    feedback.current_state = sm_state
    userdata.feedback = feedback
# ===============================================================================


def main():
    # Open the container
    rospy.init_node("insert_object_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )
    with sm:
        smach.StateMachine.add(
            "SET_PREGRASP_PARAMS",
            gbs.set_named_config("pregrasp_planner_no_sampling"),
            transitions={
                "success": "PERCEIVE_LOCATION",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

         # start perception
        smach.StateMachine.add(
            "PERCEIVE_LOCATION",
            gas.perceive_location(obj_category="multimodal_object_recognition_atwork"),
            transitions={
                "success": "SELECT_OBJECT",
                "failed": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SELECT_OBJECT",
            SelectObject("/mcr_perception/object_selector/input/object_name"),
            transitions={"succeeded": "GENERATE_OBJECT_POSE"},
        )

        # generates a pose of object
        smach.StateMachine.add(
            "GENERATE_OBJECT_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_perception/object_selector/event_in", "e_trigger")
                ],
                event_out_list=[
                    ("/mcr_perception/object_selector/event_out", "e_selected", True,)
                ],
                timeout_duration=10,
            ),
            transitions={
                "success": "SET_DBC_PARAMS",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SET_DBC_PARAMS",
            gbs.set_named_config("dbc_pick_object"),
            transitions={
                "success": "MOVE_ROBOT_AND_TRY_PLACING",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "MOVE_ROBOT_AND_TRY_PLACING",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_try")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=50,
            ),
            transitions={
                "success": "UNSTAGE_OBJECT",
                "timeout": "UNSTAGE_OBJECT",
                "failure": "UNSTAGE_OBJECT",
            },
        )

        smach.StateMachine.add(
            "UNSTAGE_OBJECT",
            gas.unstage_object(),
            transitions={
                "success": "RE_GENERATE_OBJECT_POSE",
                "failed": "OVERALL_FAILED",
            },
        )

        # generates a pose based on the previous string object topic received
        smach.StateMachine.add(
            "RE_GENERATE_OBJECT_POSE",
            gbs.send_event(
                [("/mcr_perception/object_selector/event_in", "e_re_trigger")]
            ),
            transitions={"success": "MOVE_ROBOT_AND_PLACE"},
        )

        # execute robot motion
        smach.StateMachine.add(
            "MOVE_ROBOT_AND_PLACE",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_start")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=20,
            ),
            transitions={
                "success": "OPEN_GRIPPER",
                "timeout": "MOVE_ARM_TO_DEFAULT_PLACE",
                "failure": "MOVE_ARM_TO_DEFAULT_PLACE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_DEFAULT_PLACE",
            gms.move_arm("look_at_turntable"),
            transitions={
                "succeeded": "MOVE_ARM_CARTESIAN",
                "failed": "MOVE_ARM_TO_DEFAULT_PLACE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_CARTESIAN",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/insert_object_workaroud/event_in", "e_start")],
                event_out_list=[
                    ("/insert_object_workaroud/event_out", "e_success", True)
                ],
                timeout_duration=10,
            ),
            transitions={
                "success": "OPEN_GRIPPER",
                "timeout": "STOP_ARM_CARTESIAN_MOTION",
                "failure": "OPEN_GRIPPER",
            },
        )

        smach.StateMachine.add(
            "STOP_ARM_CARTESIAN_MOTION",
            gbs.send_event([("/insert_object_workaroud/event_in", "e_stop")]),
            transitions={"success": "OPEN_GRIPPER"},
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper(0.25),
            transitions={"succeeded": "MOVE_ARM_TO_HOLD",
                         "timeout": "MOVE_ARM_TO_HOLD"},
        )

        # move arm to HOLD position
        smach.StateMachine.add(
            "MOVE_ARM_TO_HOLD",
            gms.move_arm("look_at_turntable"),
            transitions={"succeeded": "OVERALL_SUCCESS", "failed": "MOVE_ARM_TO_HOLD",},
        )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

    # smach viewer
    if rospy.get_param("~viewer_enabled", False):
        sis = smach_ros.IntrospectionServer(
            "insert_object_smach_viewer", sm, "/INSERT_OBJECT_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="insert_object_server",
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
