#!/usr/bin/python

import mcr_perception_states.common.perception_states as gps
import mcr_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms
import rospy
import smach
from diagnostic_msgs.msg import KeyValue
from mas_perception_msgs.msg import ObjectList
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
)
from smach_ros import ActionServerWrapper, IntrospectionServer

# ===============================================================================


class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=[],
            output_keys=["feedback", "result"],
        )

    def execute(self, userdata):
        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="Setup", text="Setting up"
        )
        return "succeeded"


# ===============================================================================


class PopulateResultWithCavities(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )
        self.cavities_sub = rospy.Subscriber(
            "/mcr_perception/cavity_finder/output/cavities_list",
            ObjectList,
            self.objects_callback,
        )
        self.perceived_cavity_names = []

    def objects_callback(self, msg):
        self.perceived_cavity_names = [str(obj.name) for obj in msg.objects]

    def execute(self, userdata):
        result = GenericExecuteResult()
        for i, obj in enumerate(self.perceived_cavity_names):
            result.results.append(KeyValue(key="cavity_" + str(i + 1), value=obj))
        rospy.loginfo(result)
        userdata.result = result

        self.perceived_obj_names = []  # clear perceived objects for next call
        return "succeeded"


# ===============================================================================


def main():
    rospy.init_node("perceive_cavity_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )
    sm.userdata.next_arm_pose_index = 0
    # Open the container
    with sm:
        # approach to platform
        smach.StateMachine.add(
            "SETUP",
            Setup(),
            transitions={"succeeded": "PUBLISH_REFERENCE_FRAME_FOR_WBC"},
        )

        # generates a pose based on the previous string object topic received
        smach.StateMachine.add(
            "PUBLISH_REFERENCE_FRAME_FOR_WBC",
            gbs.send_event([("/static_transform_publisher_node/event_in", "e_start")]),
            transitions={"success": "START_POSE_SELECTOR"},
        )

        # start the cavity pose selector to accumulate the poses
        smach.StateMachine.add(
            "START_POSE_SELECTOR",
            gbs.send_event(
                [("/mcr_perception/cavity_pose_selector/event_in", "e_start")]
            ),
            transitions={"success": "LOOK_AROUND"},
        )

        # move arm to selected pose
        smach.StateMachine.add(
            "LOOK_AROUND",
            gms.move_arm("look_at_workspace_from_near", blocking=True),
            transitions={"succeeded": "RECOGNIZE_CAVITIES", "failed": "LOOK_AROUND",},
        )

        # trigger perception pipeline
        smach.StateMachine.add(
            "RECOGNIZE_CAVITIES",
            gps.find_cavities(retries=1),
            transitions={
                "cavities_found": "POPULATE_RESULT_WITH_CAVITIES",
                "no_cavities_found": "OVERALL_FAILED",
            },
        )

        # populate action server result with perceived objects
        smach.StateMachine.add(
            "POPULATE_RESULT_WITH_CAVITIES",
            PopulateResultWithCavities(),
            transitions={"succeeded": "OVERALL_SUCCESS"},
        )

    # smach viewer
    if rospy.get_param("~viewer_enabled", False):
        sis = IntrospectionServer(
            "perceive_cavity_smach_viewer", sm, "/PERCEIVE_CAVITY_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="perceive_cavity_server",
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
