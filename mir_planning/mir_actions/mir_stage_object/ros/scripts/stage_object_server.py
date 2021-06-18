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


def main():
    rospy.init_node("stage_object_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )

    # read heavy object list
    sm.userdata.heavy_objects = rospy.get_param("~heavy_objects", ["m20_100"])

    with sm:
        # add states to the container
        smach.StateMachine.add(
            "MOVE_ARM_TO_STAGE_INTERMEDIATE",
            gms.move_arm("stage_intermediate"),
            transitions={
                "succeeded": "CHECK_IF_OBJECT_HEAVY",
                "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE",
            },
        )

        smach.StateMachine.add(
            "CHECK_IF_OBJECT_HEAVY",
            IsObjectHeavy(),
            transitions={
                "heavy": "MOVE_ARM_TO_STAGE_INTERMEDIATE_2",
                "light": "SETUP_MOVE_ARM_PRE_STAGE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_STAGE_INTERMEDIATE_2",
            gms.move_arm("stage_intermediate_2"),
            transitions={
                "succeeded": "SETUP_MOVE_ARM_PRE_STAGE_HEAVY",
                "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE_2",
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
            gms.move_arm(),
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
            gms.move_arm(),
            transitions={
                "succeeded": "OPEN_GRIPPER",
                "failed": "MOVE_ARM_STAGE"
            },
        )

        smach.StateMachine.add(
            "SETUP_MOVE_ARM_PRE_STAGE_HEAVY",
            SetupMoveArm("pre", is_heavy=True),
            transitions={
                "succeeded": "MOVE_ARM_PRE_STAGE_HEAVY",
                "failed": "SETUP_MOVE_ARM_PRE_STAGE_HEAVY",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_PRE_STAGE_HEAVY",
            gms.move_arm(),
            transitions={
                "succeeded": "SETUP_MOVE_ARM_STAGE_HEAVY",
                "failed": "MOVE_ARM_PRE_STAGE_HEAVY",
            },
        )

        smach.StateMachine.add(
            "SETUP_MOVE_ARM_STAGE_HEAVY",
            SetupMoveArm("final", is_heavy=True),
            transitions={
                "succeeded": "MOVE_ARM_STAGE_HEAVY",
                "failed": "SETUP_MOVE_ARM_STAGE_HEAVY",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_STAGE_HEAVY",
            gms.move_arm(),
            transitions={
                "succeeded": "OPEN_GRIPPER",
                "failed": "MOVE_ARM_STAGE_HEAVY"
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper("open_narrow"),
            transitions={"succeeded": "CHECK_IF_OBJECT_HEAVY_AGAIN"},
        )

        smach.StateMachine.add(
            "CHECK_IF_OBJECT_HEAVY_AGAIN",
            IsObjectHeavy(),
            transitions={
                "heavy": "SETUP_MOVE_ARM_PRE_STAGE_HEAVY_AGAIN",
                "light": "SETUP_MOVE_ARM_PRE_STAGE_AGAIN",
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
            "SETUP_MOVE_ARM_PRE_STAGE_HEAVY_AGAIN",
            SetupMoveArm("pre", is_heavy=True),
            transitions={
                "succeeded": "MOVE_ARM_PRE_STAGE_HEAVY_AGAIN",
                "failed": "SETUP_MOVE_ARM_PRE_STAGE_HEAVY_AGAIN",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_PRE_STAGE_HEAVY_AGAIN",
            gms.move_arm(blocking=False),
            transitions={
                "succeeded": "MOVE_ARM_TO_STAGE_INTERMEDIATE_2_FINAL",
                "failed": "MOVE_ARM_PRE_STAGE_HEAVY_AGAIN",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_STAGE_INTERMEDIATE_2_FINAL",
            gms.move_arm("stage_intermediate_2"),
            transitions={
                "succeeded": "MOVE_ARM_TO_STAGE_INTERMEDIATE_FINAL",
                "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE_2_FINAL",
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
