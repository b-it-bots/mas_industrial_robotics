#!/usr/bin/python

from __future__ import print_function
import mcr_states.common.basic_states as gbs
import mir_states.common.basic_states as mir_gbs
import mir_states.common.manipulation_states as gms
import mir_states.common.navigation_states as gns
import mir_states.common.action_states as gas
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


class CheckIfBaseCentered(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["yes", "no", "unavailable"],
            input_keys=["goal"]
        )
        self.tf_listener = tf.TransformListener()
        self.distance_threshold = 0.1

    def get_robot_pose(self):
        for i in range(10):
            try:
                trans, rot = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
                _, _, yaw = tf.transformations.euler_from_quaternion(rot)
                return (trans[0], trans[1], yaw)
            except Exception as e:
                rospy.logerr(str(e))
        return None

    def execute(self, userdata):
        target_location = Utils.get_value_of(userdata.goal.parameters, 'location')
        if target_location is not None:
            target_pose = Utils.get_pose_from_param_server(target_location)
            robot_pose = self.get_robot_pose()
            xdiff = target_pose.pose.position.x - robot_pose[0]
            ydiff = target_pose.pose.position.y - robot_pose[1]
            if (xdiff > self.distance_threshold or ydiff > self.distance_threshold):
                return 'no'
            else:
                return 'yes'

        else:
            return 'unavailable'

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
        self.perceived_obj_db_id = []

    def objects_callback(self, msg):
        print(f'inside callback with {len(msg.objects)} objects')
        self.perceived_obj_names = [str(obj.name) for obj in msg.objects]
        self.perceived_obj_db_id = [str(obj.database_id) for obj in msg.objects]

    def execute(self, userdata):
        print('[perc_obj_ser] inside executor')
        result = GenericExecuteResult()
        for i, obj in enumerate(self.perceived_obj_names):
            result.results.append(KeyValue(key="obj_" + str(i + 1), value=obj))
            result.results.append(KeyValue(key="obj_" + str(i + 1) + "_id",
                                           value=self.perceived_obj_db_id[i]))
        userdata.result = result

        userdata.feedback = GenericExecuteFeedback()  # place holder

        self.perceived_obj_names = []  # clear perceived objects for next call
        self.perceived_obj_db_id = []  # clear perceived objects for next call
        return "succeeded"


# ===============================================================================


class GetMotionType(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["base_motion", "arm_motion"])

    def execute(self, userdata):
        base_motion_enabled = rospy.get_param("~base_motion_enabled", False)
        return "base_motion" if base_motion_enabled else "arm_motion"


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
        # "look_at_workspace_from_near_left",
        # "look_at_workspace_from_near_right",
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
            "SETUP", Setup(), transitions={"succeeded": "CHECK_IF_BASE_IS_AT_LOCATION"},
        )

        smach.StateMachine.add(
            "CHECK_IF_BASE_IS_AT_LOCATION",
            CheckIfBaseCentered(),
            transitions={"yes": "PUBLISH_REFERENCE_FRAME",
                         "unavailable": "PUBLISH_REFERENCE_FRAME",
                         "no" : "MOVE_BASE_TO_LOCATION"},
        )

        smach.StateMachine.add(
            "MOVE_BASE_TO_LOCATION",
            gas.move_base(None),
            transitions={"success": "PUBLISH_REFERENCE_FRAME",
                         "failed" : "PUBLISH_REFERENCE_FRAME"},
        )

        # publish a static frame which will be used as reference for perceived objs
        smach.StateMachine.add(
            "PUBLISH_REFERENCE_FRAME",
            gbs.send_event([("/static_transform_publisher_node/event_in", "e_start")]),
            transitions={"success": "SET_DIRECT_BASE_CONTROLLER_PARAMETERS"},
        )

        # publish a static frame which will be used as reference for perceived objs
        smach.StateMachine.add(
            "SET_DIRECT_BASE_CONTROLLER_PARAMETERS",
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
                "base_motion": "SET_NEXT_BASE_POSE",
                "arm_motion": "SET_NEXT_ARM_POSE",
            },
        )

        # smach.StateMachine.add(
        #     "SET_NEXT_BASE_POSE",
        #     SetupMoveBaseWithDBC(),
        #     transitions={
        #         "pose_set": "MOVE_BASE_WITH_DIRECT_BASE_CONTROLLER",
        #         "tried_all": "POPULATE_RESULT_WITH_OBJECTS",
        #     },
        # )

        smach.StateMachine.add(
            "SET_NEXT_BASE_POSE",
            SetupMoveBaseWithDBC(),
            transitions={
                "pose_set": "MOVE_BASE_WITH_DIRECT_BASE_CONTROLLER",
                "tried_all": "STOP_OBJECT_LIST_MERGER",
            },
        )

        smach.StateMachine.add(
            "MOVE_BASE_WITH_DIRECT_BASE_CONTROLLER",
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
                "success": "MOVE_ARM_TO_PERCEIVE_POSE",
                "timeout": "STOP_DIRECT_BASE_CONTROLLER",
                "failure": "STOP_DIRECT_BASE_CONTROLLER",
            },
        )

        smach.StateMachine.add(
            "SET_NEXT_ARM_POSE",
            SetupMoveArm(),
            transitions={
                "pose_set": "MOVE_ARM_TO_PERCEIVE_POSE",
                "tried_all": "STOP_OBJECT_LIST_MERGER",
            },
        )

        # move arm to appropriate position
        smach.StateMachine.add(
            "MOVE_ARM_TO_PERCEIVE_POSE",
            gms.move_arm_and_gripper("open"),
            transitions={
                "succeeded": "WAIT_FOR_ARM_TO_STABILIZE",
                "failed": "MOVE_ARM_TO_PERCEIVE_POSE",
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
                timeout_duration=60,
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
            "STOP_DIRECT_BASE_CONTROLLER",
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

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

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
