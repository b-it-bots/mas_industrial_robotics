#!/usr/bin/python

import mcr_perception_states.common.perception_states as gps
import mcr_states.common.basic_states as gbs
import mir_states.common.basic_states as mir_gbs
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


class SetupMoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["pose_set", "tried_all"],
            input_keys=["arm_pose_index", "arm_pose_list"],
            output_keys=["arm_pose_index", "move_arm_to"],
        )

    def execute(self, userdata):
        if userdata.arm_pose_index >= len(userdata.arm_pose_list):
            return "tried_all"
        # set arm pose to next pose in list
        userdata.move_arm_to = userdata.arm_pose_list[userdata.arm_pose_index]
        userdata.arm_pose_index += 1

        return "pose_set"


# ===============================================================================
# ===============================================================================


class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["goal", "arm_pose_list"],
            output_keys=["feedback", "result", "arm_pose_index", "arm_pose_list"],
        )

    def execute(self, userdata):
        userdata.arm_pose_index = 0  # reset arm position for new request
        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="Setup", text="Setting up"
        )
	# This server runs in two modes: 
        # 1. three_view mode - perceive using motion of arm in three directions <straight, left, right>
        # 2. single_view mode - only perceiving in one direction <straight>
        perception_mode = Utils.get_value_of(userdata.goal.parameters, "perception_mode")
        if perception_mode is not None and perception_mode == "single_view":
            userdata.arm_pose_list = ["pre_place"]
        else:
            userdata.arm_pose_list = [
                "ppt_cavity_middle",
                "ppt_cavity_left",
                "ppt_cavity_far_left",
                "ppt_cavity_right",
                "ppt_cavity_far_right"
            ]

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
       # self.perceived_cavity_names.extend([str(obj.name) for obj in msg.objects])
       for obj in msg.objects:
            if str(obj.name) not in self.perceived_cavity_names:
                self.perceived_cavity_names.append(str(obj.name))
            else:
                pass 

    def execute(self, userdata):
        result = GenericExecuteResult()
        for i, obj in enumerate(self.perceived_cavity_names):
            result.results.append(KeyValue(key="cavity_" + str(i + 1), value=obj))
        rospy.loginfo(result)
        userdata.result = result

        self.perceived_cavity_names = []  # clear perceived objects for next call
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
    rospy.init_node("perceive_cavity_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )
    # sm.userdata.next_arm_pose_index = 0

    #+++
    # this array is populated in the Setup state
    sm.userdata.arm_pose_list = []
    sm.userdata.arm_pose_index = 0
    #+++

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
            transitions={"success": "SET_APPROPRIATE_ARM_POSE"},
        )

        #+++
        # move arm to appropriate position
        smach.StateMachine.add(
            "SET_APPROPRIATE_ARM_POSE",
            SetupMoveArm(),
            transitions={
                "pose_set": "MOVE_ARM_TO_SELECTED_POSE",
                "tried_all": "POPULATE_RESULT_WITH_CAVITIES",
            },
        )

        
        smach.StateMachine.add(
            "MOVE_ARM_TO_SELECTED_POSE",
            gms.move_arm_and_gripper("open"),
            transitions={
                "succeeded": "WAIT_FOR_ARM_TO_STABILIZE",
                "failed": "MOVE_ARM_TO_SELECTED_POSE",
            },
        )

        smach.StateMachine.add(
            "WAIT_FOR_ARM_TO_STABILIZE",
            mir_gbs.wait_for(0.5),
            transitions={
                "succeeded": "RECOGNIZE_CAVITIES",
            },
        )
        #+++

        #---
        # move arm to selected pose
        # smach.StateMachine.add(
        #     "LOOK_AROUND",
        #     gms.move_arm("look_at_workspace_from_near", blocking=True),
        #     transitions={"succeeded": "RECOGNIZE_CAVITIES", "failed": "LOOK_AROUND",},
        # )
        #---

        # trigger perception pipeline
        smach.StateMachine.add(
            "RECOGNIZE_CAVITIES",
            gps.find_cavities(retries=1),
            transitions={
                "cavities_found": "SET_APPROPRIATE_ARM_POSE",
                "no_cavities_found": "SET_APPROPRIATE_ARM_POSE",
            },
        )

        # populate action server result with perceived objects
        smach.StateMachine.add(
            "POPULATE_RESULT_WITH_CAVITIES",
            PopulateResultWithCavities(),
            transitions={"succeeded": "VISUALIZE_CAVITIES"},
        )

        # populate action server result with perceived objects
        smach.StateMachine.add(
            "VISUALIZE_CAVITIES",
            gbs.send_event(
                [("/mcr_perception/cavity_pose_selector/event_in", "e_visualize")]
            ),
            transitions={"success": "OVERALL_SUCCESS"},
        )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

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
