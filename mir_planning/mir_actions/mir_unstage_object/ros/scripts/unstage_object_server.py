#!/usr/bin/env python

import mir_states.common.manipulation_states as gms  # move the arm, and gripper
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
    def __init__(self, arm_target, is_heavy=False):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["goal"],
            output_keys=["feedback", "result", "move_arm_to"],
        )
        self.arm_target = arm_target
        self.is_heavy = is_heavy

    def execute(self, userdata):
        platform = Utils.get_value_of(userdata.goal.parameters, "platform")
        if platform is None:
            rospy.logwarn('Missing parameter "platform". Using default.')
            platform = "PLATFORM_MIDDLE"
        platform = platform.lower()

        if self.arm_target == "pre":
            platform += "_pre"

        if self.is_heavy:
            platform += "_heavy"
        userdata.move_arm_to = platform

        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="SetupMoveArm", text="Moving arm to " + platform
        )
        return "succeeded"


# ===============================================================================

class IsObjectHeavy(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["heavy", "light"],
            input_keys=["goal", "heavy_objects"],
            output_keys=[],
        )

    def execute(self, userdata):
        obj = Utils.get_value_of(userdata.goal.parameters, "object")
        if obj is None:
            rospy.logwarn('Missing parameter "object". Using default.')
            return "light"
        for heavy_object in userdata.heavy_objects:
            if heavy_object.upper() in obj.upper():
                return "heavy"
        return "light"


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
    rospy.init_node("unstage_object_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )

    # read heavy object list
    sm.userdata.heavy_objects = rospy.get_param("~heavy_objects", ["m20_100"])

    with sm:
        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper("open_narrow"),
            transitions={"succeeded": "SETUP_MOVE_ARM_STAGE"},
        )
        # add states to the container

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
            gms.move_arm(),
            transitions={
                "succeeded": "CLOSE_GRIPPER",
                "failed": "MOVE_ARM_STAGE"
            },
        )

        smach.StateMachine.add(
            "CLOSE_GRIPPER",
            gms.control_gripper("close"),
            transitions={"succeeded": "VERIFY_OBJECT_GRASPED"},
        )

        smach.StateMachine.add(
            "VERIFY_OBJECT_GRASPED",
            gms.verify_object_grasped(5),
            transitions={
                "succeeded": "SETUP_MOVE_ARM_PRE_STAGE_AGAIN",
                "failed": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SETUP_MOVE_ARM_PRE_STAGE_AGAIN",
            SetupMoveArm("pre"),
            transitions={
                "succeeded": "MOVE_ARM_PRE_STAGE_AGAIN",
                "failed": "SETUP_MOVE_ARM_PRE_STAGE_AGAIN",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_PRE_STAGE_AGAIN",
            gms.move_arm(blocking=True),
            transitions={
                "succeeded": "MOVE_ARM_TO_STAGE_INTERMEDIATE_FINAL",
                "failed": "MOVE_ARM_PRE_STAGE_AGAIN",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_STAGE_INTERMEDIATE_FINAL",
            gms.move_arm("stage_intermediate"),
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE_FINAL",
            },
        )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

    # smach viewer
    if rospy.get_param("~viewer_enabled", True):
        sis = smach_ros.IntrospectionServer(
            "unstage_object_smach_viewer", sm, "/UNSTAGE_OBJECT_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="unstage_object_server",
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
