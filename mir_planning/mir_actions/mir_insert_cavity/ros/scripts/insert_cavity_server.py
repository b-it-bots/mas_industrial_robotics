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
from mas_perception_msgs.msg import Cavity
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
    GenericExecuteGoal,
)
from moveit_msgs.msg import MoveItErrorCodes
from smach_ros import ActionServerWrapper, IntrospectionServer
from std_msgs.msg import Float64, String
from diagnostic_msgs.msg import KeyValue

# ===============================================================================

# For wiggle arm
arm_command = moveit_commander.MoveGroupCommander("arm_1")
arm_command.set_goal_position_tolerance(0.01)
arm_command.set_goal_orientation_tolerance(0.01)
arm_command.set_goal_joint_tolerance(0.005)

# ===============================================================================


class SelectCavity(smach.State):
    def __init__(self, topic_name, vertical=False):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )
        self.vertical = vertical
        self.publisher = rospy.Publisher(topic_name, String, queue_size=10)
        rospy.sleep(0.1)  # time for the publisher to register in ros network

    def obj_to_cavity(self, obj_name: str, vertical: bool = False):
        matchings = {
            "M20": ["M20_H", "M20_V"],
            "M30": ["M30_H", "M30_V"],
            "M20_100": ["M20_100_H", "M20_100_V"],
            "F20_20": ["F20_20_H", "F20_20_V"],
            "S40_40": ["S40_40_H", "S40_40_V"],
        }

        for key in matchings.keys():
            if obj_name.startswith(key):
                return matchings[key][vertical]
        return None


    def execute(self, userdata):
        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="SelectObject", text="selecting object"
        )

        obj = Utils.get_value_of(userdata.goal.parameters, "peg")

        if obj is None:
            rospy.logerr("No object name provided")
            return "failed"

        # get the cavity name
        cavity_name = self.obj_to_cavity(obj, self.vertical)

        if cavity_name is None:
            rospy.logerr("No cavity name found for object: " + obj)
            return "failed"

        self.publisher.publish(String(data=cavity_name))
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
        # self.cavity_pose_topic = "/mcr_perception/object_selector/output/object_pose"
        self.cavity_pose_topic = "/mir_perception/multimodel_object_recognition/output/rgb_object_pose_array"

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

class Unstage_to_place(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=["success","failed"],input_keys=["goal","heavy_objects", "platform","object"])
        self.platform = "PLATFORM_MIDDLE"
        self.obj = "M20"

    def execute(self,userdata):

        self.platform = Utils.get_value_of(userdata.goal.parameters, "platform")
        self.obj = Utils.get_value_of(userdata.goal.parameters, "peg")

        if self.obj is None:
            rospy.logwarn('Missing parameter "object". Using default.')
            self.obj = "light"
        else:
            self.obj =  "light"


        self.unstage_client = SimpleActionClient('unstage_object_server', GenericExecuteAction)
        self.unstage_client.wait_for_server()

	    # Assigning the goal    

        goal = GenericExecuteGoal()
        goal.parameters.append(KeyValue(key="platform", value=self.platform))
        goal.parameters.append(KeyValue(key="object", value=self.obj))

        self.unstage_client.send_goal(goal)
        self.unstage_client.wait_for_result(rospy.Duration.from_sec(25.0))

        return "success"

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
    rospy.init_node("insert_cavity_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )
    with sm:
        # set cavity model
        smach.StateMachine.add(
            "SET_PERCEPTION_PARAMS",
            gbs.set_named_config("multimodal_object_recognition_cavity"),
            transitions={
                "success": "PERCEIVE_LOCATION",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        # start perception
        smach.StateMachine.add(
            "PERCEIVE_LOCATION",
            gas.perceive_location(),
            transitions={
                "success": "SELECT_CAVITY",
                "failed": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SELECT_CAVITY",
            SelectCavity("/mcr_perception/object_selector/input/object_name", vertical=False),
            transitions={
                "succeeded": "GENERATE_OBJECT_POSE",
                "failed": "GET_VERTICAL_CAVITY"
            },
        )

        smach.StateMachine.add(
            "GET_VERTICAL_CAVITY",
            SelectCavity("/mcr_perception/object_selector/input/object_name", vertical=True),
            transitions={
                "succeeded": "GENERATE_OBJECT_POSE",
                "failed": "OVERALL_FAILED"
            },
        )

        # generates a pose of object
        smach.StateMachine.add(
            "GENERATE_OBJECT_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/mcr_perception/object_selector/event_in", "e_trigger")],
                event_out_list=[("/mcr_perception/object_selector/event_out", "e_selected", True)],
                timeout_duration=10,
            ),
            transitions={
                "success": "UNSTAGE_FOR_PLACING",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        # unstage object to place
        smach.StateMachine.add(
            "UNSTAGE_FOR_PLACING",
            Unstage_to_place(),
            transitions={
                "success":"SET_DBC_PARAMS", 
                "failed":"OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SET_DBC_PARAMS",
            gbs.set_named_config("dbc_pick_object"),
            transitions={
                "success": "MOVE_ROBOT_AND_TRY_INSERTING",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "MOVE_ROBOT_AND_TRY_INSERTING",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_start")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=50,
            ),
            transitions={
                "success": "OPEN_GRIPPER",
                "timeout": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
                "failure": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
            },
        )

        smach.StateMachine.add(
            "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
            gbs.send_event(
                [
                    ("/waypoint_trajectory_generation/event_in", "e_stop"),
                    ("/wbc/event_in", "e_stop"),
                ]
            ),
            transitions={"success": "OVERALL_FAILED"},
        )

        # open gripper
        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper("open"),
            transitions={
                "succeeded": "WIGGLE_ARM",
                "timeout": "WIGGLE_ARM"
            },
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
            gms.move_arm("pre_place"),
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "failed": "MOVE_ARM_TO_HOLD",
            },
        )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

    # smach viewer
    if rospy.get_param("~viewer_enabled", True):
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
