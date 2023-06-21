#!/usr/bin/env python

import mir_states.common.manipulation_states as gms  # move the arm, and gripper
import mir_states.common.basic_states as mir_gbs
import rospy
import smach
import smach_ros
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
)
from smach_ros import ActionServerWrapper

# ===============================================================================

class SetupMoveArm(smach.State):
    def __init__(self, arm_target):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["goal"],
            output_keys=["feedback", "result", "move_arm_to"],
        )
        self.arm_target = arm_target

    def execute(self, userdata):
        platform = Utils.get_value_of(userdata.goal.parameters, "platform")
        if platform is None:
            rospy.logwarn('Missing parameter "platform". Using default.')
            platform = "PLATFORM_LEFT"
        platform = platform.lower()

        if self.arm_target == "pre":
            platform += "_pre"

        userdata.move_arm_to = platform

        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="SetupMoveArm", text="Moving arm to " + platform
        )
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
    rospy.init_node("stage_object_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )


    with sm:
        # add states to the container
        smach.StateMachine.add(
            "MOVE_ARM_TO_STAGE_INTERMEDIATE",
            gms.move_arm("pre_place", use_moveit=False),
            transitions={
                "succeeded": "SETUP_MOVE_ARM_PRE_STAGE",
                "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE",
            },
        )

        smach.StateMachine.add(
            "SETUP_MOVE_ARM_PRE_STAGE",
            SetupMoveArm("pre"),
            transitions={
                "succeeded": "MOVE_ARM_PRE_STAGE",
                "failed": "SETUP_MOVE_ARM_PRE_STAGE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_PRE_STAGE",
            gms.move_arm(use_moveit=False),
            transitions={
                "succeeded": "SETUP_MOVE_ARM_STAGE",
                "failed": "MOVE_ARM_PRE_STAGE",
            },
        )

        smach.StateMachine.add(
            "SETUP_MOVE_ARM_STAGE",
            SetupMoveArm("final"),
            transitions={
                "succeeded": "MOVE_ARM_STAGE",
                "failed": "SETUP_MOVE_ARM_STAGE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_STAGE",
            gms.move_arm(use_moveit=False),
            transitions={
                "succeeded": "OPEN_GRIPPER",
                "failed": "MOVE_ARM_STAGE"
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper("open_narrow"),
            transitions={"succeeded": "SETUP_MOVE_ARM_RETRACT",
                         "timeout": "SETUP_MOVE_ARM_RETRACT"},
        )

        smach.StateMachine.add(
            "SETUP_MOVE_ARM_RETRACT",
            SetupMoveArm("pre"),
            transitions={
                "succeeded": "MOVE_ARM_RETRACT",
                "failed": "SETUP_MOVE_ARM_RETRACT",
            },
        )
        smach.StateMachine.add(
            "MOVE_ARM_RETRACT",
            gms.move_arm(use_moveit=False),
            transitions={
                "succeeded": "MOVE_ARM_TO_STAGE_INTERMEDIATE_RETRACT",
                "failed": "MOVE_ARM_RETRACT"
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_STAGE_INTERMEDIATE_RETRACT",
            gms.move_arm("pre_place", use_moveit=False),
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE_RETRACT",
            },
        )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

    # smach viewer
    if rospy.get_param("~viewer_enabled", False):
        sis = smach_ros.IntrospectionServer(
            "stage_object_smach_viewer", sm, "/STAGE_OBJECT_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="stage_object_server",
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