#!/usr/bin/python
import mcr_perception_states.common.perception_states as gps
import mcr_states.common.basic_states as gbs
import mir_states.common.action_states as gas
import mir_states.common.manipulation_states as gms
import moveit_commander
import numpy as np
import rospy
import smach
import tf.transformations
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from mcr_perception_msgs.msg import Cavity
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
)
from moveit_msgs.msg import MoveItErrorCodes
from smach_ros import ActionServerWrapper, IntrospectionServer
from std_msgs.msg import Float64, String

# ===============================================================================

# For wiggle arm
arm_command = moveit_commander.MoveGroupCommander("arm_1")
arm_command.set_goal_position_tolerance(0.01)
arm_command.set_goal_orientation_tolerance(0.01)
arm_command.set_goal_joint_tolerance(0.005)

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

        obj = Utils.get_value_of(userdata.goal.parameters, "peg")
        self.publisher.publish(String(data=obj))
        rospy.sleep(0.2)  # let the topic survive for some time
        return "succeeded"


# ===============================================================================


class ppt_wiggle_arm(smach.State):
    """
    Wiggle arm after placement on the table
    """

    def __init__(self, wiggle_offset=0.0, joint=0):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.wiggle_offset = wiggle_offset
        self.joint_number = joint
        self.blocking = True

        # create publisher for sending gripper position
        self.gripper_topic = "/gripper_controller/command"
        self.cavity_pose_topic = "/mcr_perception/object_selector/output/object_pose"

        self.gripper = rospy.Publisher(self.gripper_topic, Float64, queue_size=10)

        # Determining which object to wiggle
        # rospy.Subscriber("~object_name", std_msgs.msg.String, self.object_name_cb)
        # Determineing object orientation
        rospy.Subscriber(self.cavity_pose_topic, PoseStamped, self.cavity_pose_cb)

        self.yaw = 0
        self.joint_values_static = []
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def object_name_cb(self, msg):
        """
        Obtains an event for the component.

        """
        rospy.loginfo("Object received: {0}".format(msg.data))
        self.object_name = msg

    def cavity_pose_cb(self, msg):
        self.object_pose = msg
        o = msg.pose.orientation
        l = [o.x, o.y, o.z, o.w]
        roll, pitch, self.yaw = tf.transformations.euler_from_quaternion(l)

    def execute_arm(self, joint_number, wiggle_offset):
        joint_values = self.joint_values_static[:]
        joint_values[joint_number] = (
            self.joint_values_static[joint_number] + wiggle_offset
        )
        try:
            arm_command.set_joint_value_target(joint_values)
        except Exception as e:
            rospy.logerr("unable to set target position: %s" % (str(e)))
            return "failed"
        error_code = arm_command.go(wait=self.blocking)

        if error_code == MoveItErrorCodes.SUCCESS:
            return "succeeded"
        else:
            rospy.logerr("Arm movement failed with error code: %d", error_code)
            return "failed"

    def execute(self, userdata):
        # open the arm slightly
        message = Float64()
        message.data = -0.1
        self.gripper.publish(message)
        rospy.sleep(0.1)
        joint_number_to_change = None
        wiggle_offset = None
        self.joint_values_static = arm_command.get_current_joint_values()
        #################
        if np.allclose(self.yaw, 0):
            # wiggle right left first
            joint_number_to_change = 0
            wiggle_offset = -0.12
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # go left
            joint_number_to_change = 0
            wiggle_offset = 0.12
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # go center again
            joint_number_to_change = 0
            wiggle_offset = 0.0
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # wiggle forward
            joint_number_to_change = 3
            wiggle_offset = -0.12
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # go left
            joint_number_to_change = 3
            wiggle_offset = 0.12
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # go center again
            joint_number_to_change = 3
            wiggle_offset = 0.0
            self.execute_arm(joint_number_to_change, wiggle_offset)
        else:
            # wiggle forward
            joint_number_to_change = 3
            wiggle_offset = -0.12
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # go left
            joint_number_to_change = 3
            wiggle_offset = 0.12
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # go center again
            joint_number_to_change = 3
            wiggle_offset = 0.0
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # wiggle right left first
            joint_number_to_change = 0
            wiggle_offset = -0.12
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # go left
            joint_number_to_change = 0
            wiggle_offset = 0.12
            self.execute_arm(joint_number_to_change, wiggle_offset)
            # go center again
            joint_number_to_change = 0
            wiggle_offset = 0.0
            self.execute_arm(joint_number_to_change, wiggle_offset)

        return "succeeded"
        #################


# ===============================================================================


def main():
    rospy.init_node("insert_cavity_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )
    with sm:
        # publish object as string to mcr_perception_selectors -> cavity,
        # this component then publishes pose in base_link reference frame
        smach.StateMachine.add(
            "SELECT_OBJECT",
            SelectObject("/mcr_perception/cavity_pose_selector/object_name"),
            transitions={"succeeded": "CHECK_IF_OBJECT_IS_AVAILABLE"},
        )

        smach.StateMachine.add(
            "CHECK_IF_OBJECT_IS_AVAILABLE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_perception/cavity_pose_selector/event_in", "e_trigger",)
                ],
                event_out_list=[
                    (
                        "/mcr_perception/cavity_pose_selector/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=10,
            ),
            transitions={
                "success": "SET_DBC_PARAMS",
                "timeout": "UNSTAGE_OBJECT",
                "failure": "UNSTAGE_OBJECT",
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
                "success": "STOP_POSE_SELECTOR",
                "timeout": "STOP_POSE_SELECTOR",
                "failure": "STOP_POSE_SELECTOR",
            },
        )

        smach.StateMachine.add(
            "STOP_POSE_SELECTOR",
            gbs.send_event(
                [("/mcr_perception/cavity_pose_selector/event_in", "e_stop")]
            ),
            transitions={"success": "PERCEIVE_CAVITY"},
        )

        # perceive cavity again after moving in front of the desired cavity
        smach.StateMachine.add(
            "PERCEIVE_CAVITY",
            gas.perceive_cavity(),
            transitions={"success": "SELECT_OBJECT_AGAIN", "failed": "OVERALL_FAILED",},
        )

        smach.StateMachine.add(
            "SELECT_OBJECT_AGAIN",
            SelectObject("/mcr_perception/cavity_pose_selector/object_name"),
            transitions={"succeeded": "CHECK_IF_OBJECT_IS_AVAILABLE_AGAIN"},
        )

        smach.StateMachine.add(
            "CHECK_IF_OBJECT_IS_AVAILABLE_AGAIN",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_perception/cavity_pose_selector/event_in", "e_trigger",)
                ],
                event_out_list=[
                    (
                        "/mcr_perception/cavity_pose_selector/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=10,
            ),
            transitions={
                "success": "UNSTAGE_OBJECT",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "UNSTAGE_OBJECT",
            gas.unstage_object(),
            transitions={"success": "TRY_INSERTING", "failed": "OVERALL_FAILED",},
        )

        # execute robot motion
        smach.StateMachine.add(
            "TRY_INSERTING",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_start_arm_only")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=20,
            ),
            transitions={
                "success": "OPEN_GRIPPER",
                "timeout": "STAGE_OBJECT",
                "failure": "STAGE_OBJECT",
            },
        )

        smach.StateMachine.add(
            "STAGE_OBJECT",
            gas.stage_object(),
            transitions={"success": "OVERALL_FAILED", "failed": "OVERALL_FAILED",},
        )

        # close gripper
        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper("open"),
            transitions={"succeeded": "WIGGLE_ARM"},
        )

        # wiggling the arm for precision placement
        smach.StateMachine.add(
            "WIGGLE_ARM",
            ppt_wiggle_arm(wiggle_offset=-0.12, joint=0),
            transitions={
                "succeeded": "MOVE_ARM_TO_HOLD",
                "failed": "MOVE_ARM_TO_HOLD",
            },
        )

        # move arm to HOLD position
        smach.StateMachine.add(
            "MOVE_ARM_TO_HOLD",
            gms.move_arm("look_at_turntable"),
            transitions={
                "succeeded": "STOP_POSE_SELECTOR_FINAL",
                "failed": "MOVE_ARM_TO_HOLD",
            },
        )

        # sending e_stop to pose selector
        smach.StateMachine.add(
            "STOP_POSE_SELECTOR_FINAL",
            gbs.send_event(
                [("/mcr_perception/cavity_pose_selector/event_in", "e_stop")]
            ),
            transitions={"success": "OVERALL_SUCCESS"},
        )

    # smach viewer
    if rospy.get_param("~viewer_enabled", False):
        sis = IntrospectionServer(
            "insert_cavity_smach_viewer", sm, "/INSERT_CAVITY_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="insert_cavity_server",
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
