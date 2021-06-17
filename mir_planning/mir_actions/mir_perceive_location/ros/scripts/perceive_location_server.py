#!/usr/bin/python

import mcr_states.common.basic_states as gbs
import mir_states.common.basic_states as mir_gbs
import mir_states.common.manipulation_states as gms
import mir_states.common.navigation_states as gns
import rospy
import smach
import tf
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import PoseStamped, Quaternion
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


class SetupMoveBaseWithDBC(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["pose_set", "tried_all"],
            input_keys=[
                "base_pose_index",
                "base_pose_list",
                "arm_pose_list",
                "arm_pose_index",
            ],
            output_keys=["base_pose_index", "move_arm_to"],
        )
        self._dbc_pose_pub = rospy.Publisher(
            "/mcr_navigation/direct_base_controller/input_pose",
            PoseStamped,
            queue_size=1,
        )
        rospy.sleep(0.1)  # time for the publisher to register in ros network

    def execute(self, userdata):
        if userdata.base_pose_index >= len(userdata.base_pose_list):
            return "tried_all"

        target_pose_dict = userdata.base_pose_list[userdata.base_pose_index]
        # set base pose to next pose in list
        dbc_pose = PoseStamped()
        dbc_pose.header.stamp = rospy.Time.now()
        dbc_pose.header.frame_id = "base_link_static"
        dbc_pose.pose.position.x = target_pose_dict["x"]
        dbc_pose.pose.position.y = target_pose_dict["y"]
        quat = tf.transformations.quaternion_from_euler(
            0.0, 0.0, target_pose_dict["theta"]
        )
        dbc_pose.pose.orientation = Quaternion(*quat)

        userdata.base_pose_index += 1
        print(dbc_pose)
        self._dbc_pose_pub.publish(dbc_pose)
        userdata.move_arm_to = userdata.arm_pose_list[userdata.arm_pose_index]
        return "pose_set"


# ===============================================================================


class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=[],
            output_keys=["feedback", "result", "arm_pose_index", "base_pose_index",],
        )

    def execute(self, userdata):
        userdata.arm_pose_index = 0  # reset arm position for new request
        userdata.base_pose_index = 0  # reset base position for new request

        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="Setup", text="Setting up"
        )
        return "succeeded"


# ===============================================================================


class PopulateResultWithObjects(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )
        self.objects_sub = rospy.Subscriber(
            "/mcr_perception/object_list_merger/output_object_list",
            ObjectList,
            self.objects_callback,
        )
        self.perceived_obj_names = []

    def objects_callback(self, msg):
        self.perceived_obj_names = [str(obj.name) for obj in msg.objects]

    def execute(self, userdata):
        result = GenericExecuteResult()
        for i, obj in enumerate(self.perceived_obj_names):
            result.results.append(KeyValue(key="obj_" + str(i + 1), value=obj))
        userdata.result = result

        userdata.feedback = GenericExecuteFeedback()  # place holder

        self.perceived_obj_names = []  # clear perceived objects for next call
        return "succeeded"


# ===============================================================================


class GetMotionType(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["base_motion", "arm_motion"])

    def execute(self, userdata):
        base_motion_enabled = rospy.get_param("~base_motion_enabled", False)
        return "base_motion" if base_motion_enabled else "arm_motion"


# ===============================================================================


def main():
    rospy.init_node("perceive_location_server")
    sleep_time = rospy.get_param("~sleep_time", 1.0)
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )
    # Open the container
    sm.userdata.arm_pose_list = [
        "look_at_workspace_from_near",
        "look_at_workspace_from_near_left",
        "look_at_workspace_from_near_right",
    ]
    sm.userdata.arm_pose_index = 0

    base_x_offset = rospy.get_param("~base_x_offset", 0.0)
    base_y_offset = rospy.get_param("~base_y_offset", 0.25)
    base_theta_offset = rospy.get_param("~base_theta_offset", 0.0)
    sm.userdata.base_pose_list = [
        {"x": 0.0, "y": 0.0, "theta": 0.0},
        {"x": base_x_offset, "y": base_y_offset, "theta": base_theta_offset},
        {"x": base_x_offset, "y": -base_y_offset, "theta": -base_theta_offset},
    ]
    sm.userdata.base_pose_index = 0

    with sm:
        # approach to platform
        smach.StateMachine.add(
            "SETUP", Setup(), transitions={"succeeded": "PUBLISH_REFERENCE_FRAME"},
        )

        # publish a static frame which will be used as reference for perceived objs
        smach.StateMachine.add(
            "PUBLISH_REFERENCE_FRAME",
            gbs.send_event([("/static_transform_publisher_node/event_in", "e_start")]),
            transitions={"success": "SET_DBC_PARAMS"},
        )

        # publish a static frame which will be used as reference for perceived objs
        smach.StateMachine.add(
            "SET_DBC_PARAMS",
            gbs.set_named_config("dbc_pick_object"),
            transitions={
                "success": "START_OBJECT_LIST_MERGER",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "START_OBJECT_LIST_MERGER",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_perception/object_list_merger/event_in", "e_start"),
                    ("/mcr_perception/object_selector/event_in", "e_start"),
                ],
                event_out_list=[
                    ("/mcr_perception/object_list_merger/event_out", "e_started", True,)
                ],
                timeout_duration=5,
            ),
            transitions={
                "success": "GET_MOTION_TYPE",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "GET_MOTION_TYPE",
            GetMotionType(),
            transitions={
                "base_motion": "SET_APPROPRIATE_BASE_POSE",
                "arm_motion": "SET_APPROPRIATE_ARM_POSE",
            },
        )

        smach.StateMachine.add(
            "SET_APPROPRIATE_BASE_POSE",
            SetupMoveBaseWithDBC(),
            transitions={
                "pose_set": "MOVE_BASE_WITH_DBC",
                "tried_all": "POPULATE_RESULT_WITH_OBJECTS",
            },
        )

        smach.StateMachine.add(
            "MOVE_BASE_WITH_DBC",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_in",
                        "e_start",
                    )
                ],
                event_out_list=[
                    (
                        "/mcr_navigation/direct_base_controller/coordinator/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=10,
            ),
            transitions={
                "success": "MOVE_ARM",
                "timeout": "STOP_DBC",
                "failure": "STOP_DBC",
            },
        )

        smach.StateMachine.add(
            "SET_APPROPRIATE_ARM_POSE",
            SetupMoveArm(),
            transitions={
                "pose_set": "MOVE_ARM",
                "tried_all": "STOP_OBJECT_LIST_MERGER",
            },
        )

        # move arm to appropriate position
        smach.StateMachine.add(
            "MOVE_ARM",
            gms.move_arm_and_gripper("open"),
            transitions={
                "succeeded": "WAIT_FOR_ARM_TO_STABILIZE",
                "failed": "MOVE_ARM",
            },
        )

        # move arm to appropriate position
        smach.StateMachine.add(
            "WAIT_FOR_ARM_TO_STABILIZE",
            mir_gbs.wait_for(0.5),
            transitions={
                "succeeded": "START_OBJECT_RECOGNITION",
            },
        )

        # New perception pipeline state machine
        smach.StateMachine.add(
            "START_OBJECT_RECOGNITION",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    (
                        "/mir_perception/multimodal_object_recognition/event_in",
                        "e_start",
                    )
                ],
                event_out_list=[
                    (
                        "/mir_perception/multimodal_object_recognition/event_out",
                        "e_done",
                        True,
                    )
                ],
                timeout_duration=10,
            ),
            transitions={
                "success": "STOP_RECOGNITION",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "STOP_RECOGNITION",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    (
                        "/mir_perception/multimodal_object_recognition/event_in",
                        "e_stop",
                    )
                ],
                event_out_list=[
                    (
                        "/mir_perception/multimodal_object_recognition/event_out",
                        "e_stopped",
                        True,
                    )
                ],
                timeout_duration=5,
            ),
            transitions={
                "success": "GET_MOTION_TYPE",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "STOP_OBJECT_LIST_MERGER",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_perception/object_list_merger/event_in", "e_stop")
                ],
                event_out_list=[
                    ("/mcr_perception/object_list_merger/event_out", "e_stopped", True,)
                ],
                timeout_duration=5,
            ),
            transitions={
                "success": "PUBLISH_MERGED_OBJECT_LIST",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "PUBLISH_MERGED_OBJECT_LIST",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_perception/object_list_merger/event_in", "e_trigger",)
                ],
                event_out_list=[
                    ("/mcr_perception/object_list_merger/event_out", "e_done", True,)
                ],
                timeout_duration=5,
            ),
            transitions={
                "success": "CHECK_IF_OBJECTS_FOUND",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "CHECK_IF_OBJECTS_FOUND",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    (
                        "/mcr_perception/object_list_merger/object_found_event_in",
                        "e_trigger",
                    )
                ],
                event_out_list=[
                    (
                        "/mcr_perception/object_list_merger/object_found_event_out",
                        "e_objects_found",
                        True,
                    )
                ],
                timeout_duration=5.0,
            ),
            transitions={
                "success": "POPULATE_RESULT_WITH_OBJECTS",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        # populate action server result with perceived objects
        smach.StateMachine.add(
            "POPULATE_RESULT_WITH_OBJECTS",
            PopulateResultWithObjects(),
            transitions={"succeeded": "OVERALL_SUCCESS"},
        )

        smach.StateMachine.add(
            "STOP_DBC",
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
                timeout_duration=10,
            ),
            transitions={
                "success": "OVERALL_FAILED",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

    # smach viewer
    if rospy.get_param("~viewer_enabled", False):
        sis = IntrospectionServer(
            "perceive_location_smach_viewer", sm, "/PERCEIVE_LOCATION_SMACH_VIEWER",
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="perceive_location_server",
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
