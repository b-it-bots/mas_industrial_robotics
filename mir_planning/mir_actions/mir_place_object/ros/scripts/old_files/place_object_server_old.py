#!/usr/bin/env python

import mcr_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms  # move the arm, and gripper
import rospy
import smach
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
)
from smach_ros import ActionServerWrapper, IntrospectionServer
from std_msgs.msg import String


# ===============================================================================
class GetPoseToPlaceOject(smach.State):  # inherit from the State base class
    def __init__(self, topic_name_pub, topic_name_sub, event_sub, timeout_duration):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["goal", "feedback"],
            output_keys=["feedback", "result", "move_arm_to"],
        )

        self.timeout = rospy.Duration.from_sec(timeout_duration)
        # create publisher
        self.platform_name_pub = rospy.Publisher(topic_name_pub, String, queue_size=10)
        rospy.Subscriber(topic_name_sub, String, self.pose_cb)
        rospy.Subscriber(event_sub, String, self.event_cb)
        rospy.sleep(0.1)  # time for publisher to register
        self.place_pose = None
        self.status = None

    def pose_cb(self, msg):
        self.place_pose = msg.data

    def event_cb(self, msg):
        self.status = msg.data

    def execute(self, userdata):
        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="GetPoseToPlaceOject", text="Getting pose to place obj",
        )

        location = Utils.get_value_of(userdata.goal.parameters, "location")
        # location = 'WS05' # TODO: remove it later 
        if location is None:
            rospy.logwarn('"location" not provided. Using default.')
            return "failed"

        self.place_pose = None
        self.status = None
        self.platform_name_pub.publish(String(data=location))

        # wait for messages to arrive
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10hz
        while not (rospy.is_shutdown()):
            if rospy.Time.now() - start_time > self.timeout:
                break
            if self.place_pose is not None and self.status is not None:
                break
            rate.sleep()

        if (
            self.place_pose is not None
            and self.status is not None
            and self.status == "e_success"
        ):
            userdata.move_arm_to = self.place_pose
            return "succeeded"
        else:
            return "failed"


# ===============================================================================


class CheckIfLocationIsShelf(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["shelf", "not_shelf"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )

    def execute(self, userdata):
        location = Utils.get_value_of(userdata.goal.parameters, "location")
        print("[Place Object Server] Location received : ", location)
        if (location == "SH01") or (location == "SH02"):
            return "shelf"
        else:
            return "not_shelf"


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
    rospy.init_node("place_object_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal", "feedback", "result"],
        output_keys=["feedback", "result"],
    )
    with sm:
        # add states to the container
        smach.StateMachine.add(
            "CHECK_IF_SHELF_INITIAL",
            CheckIfLocationIsShelf(),
            transitions={
                "shelf": "MOVE_ARM_TO_SHELF_INTERMEDIATE",
                "not_shelf": "MOVE_ARM_TO_PRE_PLACE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE",
            gms.move_arm("shelf_intermediate"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_2",
            gms.move_arm("shelf_intermediate_2"),
            transitions={
                "succeeded": "MOVE_ARM_TO_PRE_GRASP_LOWER",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_GRASP_LOWER",
            gms.move_arm("shelf_pre_grasp_lower"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_PLACE_FINAL",
                "failed": "MOVE_ARM_TO_PRE_GRASP_LOWER",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_PLACE_FINAL",
            gms.move_arm("shelf_place_final"),
            transitions={
                "succeeded": "OPEN_GRIPPER_SHELF",
                "failed": "MOVE_ARM_TO_SHELF_PLACE_FINAL",
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_SHELF",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT"},
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT",
            gms.move_arm("shelf_place_final"),
            transitions={
                "succeeded": "MOVE_ARM_TO_PRE_GRASP_LOWER_RETRACT",
                "failed": "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_GRASP_LOWER_RETRACT",
            gms.move_arm("shelf_pre_grasp_lower"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2_RETRACT",
                "failed": "MOVE_ARM_TO_PRE_GRASP_LOWER_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_2_RETRACT",
            gms.move_arm("shelf_intermediate_2"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            gms.move_arm("shelf_intermediate"),
            transitions={
                "succeeded": "MOVE_ARM_TO_NEUTRAL",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            },
        )


        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_PLACE",
            gms.move_arm("look_at_turntable"),
            transitions={
                "succeeded": "START_PLACE_POSE_SELECTOR",
                "failed": "MOVE_ARM_TO_PRE_PLACE",
            },
        )

        smach.StateMachine.add(
            "START_PLACE_POSE_SELECTOR",
            gbs.send_event(
                [("/mcr_perception/place_pose_selector/event_in", "e_start")]
            ),
            transitions={"success": "GET_POSE_TO_PLACE_OBJECT"},
        )

        smach.StateMachine.add(
            "GET_POSE_TO_PLACE_OBJECT",
            GetPoseToPlaceOject(
                "/mcr_perception/place_pose_selector/platform_name",
                "/mcr_perception/place_pose_selector/place_pose",
                "/mcr_perception/place_pose_selector/event_out",
                15.0,
            ),
            transitions={
                "succeeded": "MOVE_ARM_TO_PLACE_OBJECT",
                "failed": "MOVE_ARM_TO_DEFAULT_PLACE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_DEFAULT_PLACE",
            gms.move_arm("15cm/pose4"),
            transitions={
                "succeeded": "OPEN_GRIPPER",
                "failed": "MOVE_ARM_TO_DEFAULT_PLACE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_PLACE_OBJECT",
            gms.move_arm(),
            transitions={"succeeded": "OPEN_GRIPPER", "failed": "OPEN_GRIPPER",},
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper("open"),
            transitions={"succeeded": "STOP_PLACE_POSE_SELECTOR"},
        )

        smach.StateMachine.add(
            "STOP_PLACE_POSE_SELECTOR",
            gbs.send_event(
                [("/mcr_perception/place_pose_selector/event_in", "e_stop")]
            ),
            transitions={"success": "MOVE_ARM_TO_NEUTRAL"},
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_NEUTRAL",
            gms.move_arm("barrier_tape"),
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "failed": "MOVE_ARM_TO_NEUTRAL",
            },
        )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

    # smach viewer
    if rospy.get_param("~viewer_enabled", False):
        sis = IntrospectionServer(
            "place_object_smach_viewer", sm, "/STAGE_OBJECT_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="place_object_server",
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
