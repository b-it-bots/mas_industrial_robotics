#!/usr/bin/env python


# threshold checking for TF calculation WIP


"""
This file is for eye to hand configuration of the robot
unstage will happen outside the place object server

"""


from selectors import PollSelector
import mcr_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms  # move the arm, and gripper
import mir_states.common.action_states as gas
import rospy
import smach
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
    GenericExecuteGoal
)
from smach_ros import ActionServerWrapper, IntrospectionServer
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import KeyValue
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from brics_actuator.msg import JointPositions, JointValue
from sensor_msgs.msg import JointState
import numpy as np
import tf

import tf2_ros
import tf2_geometry_msgs

import random

# Global variable to store the latest pose from the topic
latest_empty_space_pose = None

class MoveArmUp(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failed"],
        )
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_cb)
        self.pub_arm_position = rospy.Publisher("/arm_1/arm_controller/position_command", JointPositions, queue_size=1)
        self.current_joint_positions = None
        self.is_arm_moving = False
        self.zero_vel_counter = 0
        self.joint_1_position = 1.3787 #1.8787

    def joint_states_cb(self, msg):
        if "arm_joint_1" in msg.name: # get the joint values of the arm only
            self.current_joint_positions = msg.position

        self.joint_state = msg
        # monitor the velocities
        self.joint_velocities = msg.velocity
        # if all velocities are 0.0, the arm is not moving
        if "arm_joint_1" in msg.name and all([v == 0.0 for v in self.joint_velocities]):
            self.zero_vel_counter += 1

    def execute(self, userdata):
        self.current_joint_positions = None
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.current_joint_positions is not None:
                break
        joint_values = self.current_joint_positions[:]
        joint_values = list(joint_values)
        joint_values[1] -= 0.3 # self.joint_1_position
        
        names = self.joint_state.name

        joint_positions = JointPositions()
        joint_positions.positions = [
            JointValue(
                rospy.Time.now(),
                joint_name,
                "rad",
                joint_value
            )
            for joint_name, joint_value in zip(names, joint_values)
        ]
        self.pub_arm_position.publish(joint_positions)
        rospy.sleep(1)
        return "success"


# ===============================================================================
# by Annudeep

class MoveDBCPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])
        self._dbc_pose_pub = rospy.Publisher(
            "/mcr_navigation/direct_base_controller/input_pose",
            PoseStamped,
            queue_size=1,
        )
        self.last_pose = None
        self.sub = rospy.Subscriber("/wbc/base_motion_pose", PoseStamped, self.pose_cb)
        self.listener = tf.TransformListener()  # INITIALIZE LISTENER

    def pose_cb(self, msg):
        self.last_pose = msg

    def execute(self, userdata):
        try:
            # Get pose from WBC
            if self.last_pose is None:
                rospy.logwarn("No cached WBC pose, waiting...")
                incoming_pose = rospy.wait_for_message("/wbc/base_motion_pose", PoseStamped, timeout=rospy.Duration(1.0))
            else:
                incoming_pose = self.last_pose

            # Get current base orientation from TF
            tf_msg = self.listener.lookupTransform("/base_link_static", "/base_link", rospy.Time(0))

            # Build modified pose
            modified_pose = PoseStamped()
            modified_pose.header.frame_id = "base_link_static"  # CORRECT HEADER
            modified_pose.header.stamp = rospy.Time.now()
            modified_pose.pose.position.x =tf_msg[0][0] + (-incoming_pose.pose.position.x)
            modified_pose.pose.position.y =tf_msg[0][1] + (-incoming_pose.pose.position.y)
            modified_pose.pose.position.z = tf_msg[0][2]
            modified_pose.pose.orientation.x = tf_msg[1][0]
            modified_pose.pose.orientation.y = tf_msg[1][1]
            modified_pose.pose.orientation.z = tf_msg[1][2]
            modified_pose.pose.orientation.w = tf_msg[1][3]

            self._dbc_pose_pub.publish(modified_pose)
            return "succeeded"

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF error in MoveDBC: %s", str(e))
            return "succeeded"
        except rospy.ROSException as e:
            rospy.logerr("ROS error in MoveDBC: %s", str(e))
            return "succeeded"


class TriggerEmptySpaceDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.trigger_pub = rospy.Publisher('/empty_space_detector/event_in', String, queue_size=10)
        self.event_sub = rospy.Subscriber('/empty_space_detector/event_out', String, self.event_callback)
        self.event_received = None  # Track received event type

    def event_callback(self, msg):
        """
        Callback to handle the event_out messages from the empty space detector.
        """
        if msg.data == "e_empty_space_detected":
            self.event_received = "success"
        elif msg.data == "e_no_empty_space_detected":
            self.event_received = "failure"

    def execute(self, userdata):
        """
        Executes the state: 
        - Triggers the empty space detection.
        - Waits for a response (success or failure).
        - Returns 'succeeded' if empty space is found.
        - Returns 'failed' if no empty space is detected or timeout occurs.
        """
        
        
        self.event_received = None  # Reset event flag
        self.trigger_pub.publish(String("e_empty"))

        timeout = rospy.Duration(3.0)  # Change this value as needed
        start_time = rospy.Time.now()

        while self.event_received is None and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)  # Small delay to prevent CPU overload

        if self.event_received == "success":
            return 'succeeded'
        else:  # If "failure" or timeout occurs
            return 'failed'

class TriggerPointCloudProcessing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.trigger_pub = rospy.Publisher('/empty_space_detector/event_in', String, queue_size=10)
        self.event_sub = rospy.Subscriber('/empty_space_detector/event_out', String, self.event_callback)
        self.event_received = False

    def event_callback(self, msg):
        if msg.data == "e_pointcloud_processed":
            self.event_received = True

    def execute(self, userdata):
        self.event_received = False
        self.trigger_pub.publish(String("e_cloud"))
        timeout = rospy.Duration(6.0)
        start_time = rospy.Time.now()
        while not self.event_received and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)
        return 'succeeded' if self.event_received else 'failed'

class SendStopEvent(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.stop_pub = rospy.Publisher('/empty_space_detector/event_in', String, queue_size=10)

    def execute(self, userdata):
        self.stop_pub.publish(String("e_stop"))
        rospy.sleep(0.1)
        return 'succeeded'

class PublishEmptyspacePosetoBaseStatic(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["nor_success", "wbc_success", "failed"],
                                    input_keys=["goal"],
                                    output_keys=["move_arm_to"])

        self.empty_space_pose_pub = rospy.Publisher(
            "mcr_perception/object_selector/output/object_pose",
            PoseStamped,
            queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def pose_callback(self, msg):
        self.received_pose = msg

    def execute(self, userdata):
        self.received_pose = None
        sub = rospy.Subscriber("/empty_space_pose", PoseStamped, self.pose_callback)
        
        timeout = rospy.Duration(5.0)  # 5 seconds timeout
        start_time = rospy.Time.now()
        
        while self.received_pose is None and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)
        
        sub.unregister()  # Unsubscribe from the topic after receiving the pose
        
        if self.received_pose is None:
            rospy.logerr("No pose received from /empty_space_pose topic")
            return "failed"
        
        try:
            # Lookup transform from the pose's frame to base_static_link
            transform = self.tf_buffer.lookup_transform(
                "base_link_static",  # Target frame
                self.received_pose.header.frame_id,  # Source frame
                rospy.Time(0),  # Get the latest available transform
                rospy.Duration(1.0)  # Timeout for transform availability
            )
            
            # Transform pose to base_static_link frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose(self.received_pose, transform)

            # Set the orientation to a "straight" predefined quaternion
            transformed_pose.pose.orientation.x = 0.0
            transformed_pose.pose.orientation.y = 0.0
            transformed_pose.pose.orientation.z = 0.0
            transformed_pose.pose.orientation.w = 1.0  # Identity quaternion

            rospy.loginfo("Transformed pose: %s" % transformed_pose)
            
            # Publish the transformed pose
            self.empty_space_pose_pub.publish(transformed_pose)
            
            # Check the y value of the transformed pose
            y_value = transformed_pose.pose.position.y
            
            if -0.05 <= y_value <= 0.1:
                return "nor_success"
            else:
                return "wbc_success"
        
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            rospy.logerr("Failed to transform pose: %s" % str(e))
            return "failed"


# ===============================================================================


class DefineShelfPlacePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'failed'],
                             input_keys=["goal"],
                             output_keys=['move_arm_to'])
        self.pose_list_sh01 = ["shelf_place_sh1_1", "shelf_place_sh1_2"]
        self.pose_list_sh02 = ["shelf_place_sh2_1", "shelf_place_sh2_2"]
        # self.pose_list_sh02 = ["shelf_place_3", "shelf_place_4"]

    def execute(self, userdata):
        location = Utils.get_value_of(userdata.goal.parameters, "location")

        try:
            if location == "SH01":
                if len(self.pose_list_sh01) > 0:
                    rospy.logwarn("Getting shelf place pose from list")
                    userdata.move_arm_to = self.pose_list_sh01.pop()
                else:
                    rospy.logfatal("No more shelf place pose in list, so using default pose")
                    userdata.move_arm_to = "shelf_place_final"
            elif location == "SH02":
                if len(self.pose_list_sh02) > 0:
                    rospy.logwarn("Getting shelf place pose from list")
                    userdata.move_arm_to = self.pose_list_sh02.pop()
                else:
                    rospy.logfatal("No more shelf place pose in list, so using default pose")
                    userdata.move_arm_to = "shelf_place_final"
            # if len(self.pose_list) > 0:
            #     rospy.logwarn("Getting shelf place pose from list")
            #     userdata.move_arm_to = self.pose_list.pop()
            # else:
            #     rospy.logfatal("No more shelf place pose in list, so using default pose")
            #     userdata.move_arm_to = "shelf_place_final"
            return 'succeeded'
        except:
            return 'failed'

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


class CheckModePlacing(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            input_keys=["empty_place"],
            outcomes=["pose_selector", "empty_pose"],
        )
    def execute(self, userdata):

        empty_place = userdata.empty_place
        empty_place = True # remove later
        print("******************")
        print(empty_place)
        if empty_place:
            print("++++++++++++++= EMPTY_PLACE")
            return "empty_pose"
        else: 
            print("++++++++++++++= POSE SELECTOR")
            return "pose_selector"


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


class DefalutSafePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"],
                                    input_keys=["goal","move_arm_to"],
                                    output_keys=["move_arm_to"])

    def execute(self, userdata):

        rospy.logwarn("Checking pre-defined safe pose")
        location = Utils.get_value_of(userdata.goal.parameters, "location")
        current_platform_height = rospy.get_param("/"+location)
        
        # Randomly select a pose from pose1 to pose4
        random_pose_index = random.randint(1, 4)  # Generates a number between 1 and 4
        selected_pose = f"pose{random_pose_index}"
        
        # userdata.move_arm_to = str(str(current_platform_height)+'pose4cm/')
        # print("from place server ========")
        
        # Construct the target pose string
        userdata.move_arm_to = f"{current_platform_height}cm/{selected_pose}"

        rospy.loginfo(f"Selected pose: {userdata.move_arm_to}")
        
        
        # print(userdata.move_arm_to)
        rospy.sleep(0.1)
        return "succeeded"


class CheckRetries(smach.State):
    def __init__(self, state=False):
        smach.State.__init__(
            self,
            outcomes=["retry", "no_retry"],
            input_keys=["current_try", "max_allowed_tries"],
            output_keys=["current_try"],
        )

    def execute(self, userdata):

        print("No of rety ===>", userdata.current_try)

        if userdata.current_try < userdata.max_allowed_tries:
            userdata.current_try += 1
            return "retry"
        else:
            userdata.current_try = 0
            return "no_retry"

# ==============================================================================



class MoveDBC(smach.State):
    def __init__(self, forward=True):
        smach.State.__init__(self, outcomes=["succeeded"])
        self._dbc_pose_pub = rospy.Publisher(
            "/mcr_navigation/direct_base_controller/input_pose",
            PoseStamped,
            queue_size=1,
        )
        self.forward = forward
        self.listener = tf.TransformListener()

    def execute(self, userdata):
        # get tf of base_link 
        tf_msg = self.listener.lookupTransform("/base_link_static", "/base_link", rospy.Time(0))
        # get the pose of the object in base_link frame
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "base_link_static"
        pose_msg.header.stamp = rospy.Time.now()
        # amount to move backward after pick object
        if self.forward:
            pose_msg.pose.position.x = tf_msg[0][0] + 0.075
        else:
            pose_msg.pose.position.x = tf_msg[0][0] - 0.075
            
        pose_msg.pose.position.y = tf_msg[0][1]
        pose_msg.pose.position.z = tf_msg[0][2]
        pose_msg.pose.orientation.x = tf_msg[1][0]
        pose_msg.pose.orientation.y = tf_msg[1][1]
        pose_msg.pose.orientation.z = tf_msg[1][2]
        pose_msg.pose.orientation.w = tf_msg[1][3]
        dbc_pose = pose_msg
        self._dbc_pose_pub.publish(dbc_pose)
        return "succeeded"


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


class SetWorkstationParam(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["goal"])

    def execute(self, userdata):
        location = Utils.get_value_of(userdata.goal.parameters, "location")
        
        # ensure it was uppercase
        location = location.upper()
        
        if location:
            rospy.set_param("/place_object_server/worskstation", location)
            rospy.loginfo(f"Set param /place_object_server/worskstation = {location}")
        return "succeeded"



def main():
    rospy.init_node("place_object_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal", "feedback", "result"],
        output_keys=["feedback", "result"],)

    sm.userdata.counter_reset_flag = False
    sm.userdata.threshold_counter = 0
    sm.userdata.empty_locations = None
    sm.userdata.max_allowed_tries = rospy.get_param("~max_allowed_IK_tries", 3)
    sm.userdata.empty_place = rospy.get_param("~is_empty_pose_placing", False) 
    sm.userdata.current_try = 0
    sm.userdata.move_arm_to = None

    # ===============================================================================
    # Added by Anudeep

    # Initialize feedback and result in userdata
    sm.userdata.feedback = GenericExecuteFeedback()
    sm.userdata.result = GenericExecuteResult()
    

    # ===============================================================================

    with sm:
        smach.StateMachine.add(
            "SET_WORKSTATION_PARAM",
            SetWorkstationParam(),
            transitions={"succeeded": "MOVE_ROBOT_TO_CENTER"},
        )
        smach.StateMachine.add(
            "MOVE_ROBOT_TO_CENTER",
            gas.move_base(None),
            transitions={"success": "MOVE_ARM_TO_PRE_PLACE",
                            "failed" : "OVERALL_FAILED"},
        )

        smach.StateMachine.add(
                "MOVE_ARM_TO_PRE_PLACE",
                gms.move_arm("pre_place", use_moveit=False),
                transitions={
                    "succeeded": "CHECK_IF_SHELF_INITIAL",
                    "failed": "MOVE_ARM_TO_PRE_PLACE",
            },
        )

        # add states to the container
        smach.StateMachine.add(
            "CHECK_IF_SHELF_INITIAL",
            CheckIfLocationIsShelf(),
            transitions={
                "shelf": "MOVE_ARM_TO_SHELF_INTERMEDIATE", 
                "not_shelf" :"TRIGGER_EMPTY_SPACE_DETECTION",
            },
        )

        # ====== below are the states for shelf placing  =================

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE",
            gms.move_arm("shelf_intermediate"),
            transitions={
                "succeeded": "PUBLISH_REFERENCE_FRAME",
                "failed": "OVERALL_FAILED",
            },
        )

        # publish a static frame which will be used as reference for perceived objs
        smach.StateMachine.add(
            "PUBLISH_REFERENCE_FRAME",
            gbs.send_event([("/static_transform_publisher_node/event_in", "e_start")]),
            transitions={"success": "SET_DBC_PARAMS"},
        )

        smach.StateMachine.add(
            "SET_DBC_PARAMS",
            gbs.set_named_config("dbc_pick_object"),
            transitions={
                "success": "MOVE_FORWARD",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "MOVE_FORWARD",
            MoveDBC(forward=True),
            transitions={"succeeded": "MOVE_BASE_USING_DBC"},
        )

        # Move base using direct base controller
        smach.StateMachine.add(
            "MOVE_BASE_USING_DBC",
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
                "success": "SET_SHELF_PLACE_POSE",
                "timeout": "SET_SHELF_PLACE_POSE",
                "failure": "SET_SHELF_PLACE_POSE",
            },
        )

        smach.StateMachine.add(
            "SET_SHELF_PLACE_POSE",
            DefineShelfPlacePose(),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_PLACE_FINAL",
                "failed": "SET_SHELF_PLACE_POSE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_PLACE_FINAL",
            gms.move_arm(),
            transitions={
                "succeeded": "OPEN_GRIPPER_SHELF",
                "failed": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_SHELF",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ARM_SAFE",
                         "timeout": "MOVE_ARM_SAFE"}
        )

        # smach.StateMachine.add(
        #     "MOVE_ARM_SAFE",
        #     MoveArmUp(),
        #     transitions={"succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
        #                  "timeout": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT"}
        # )

        smach.StateMachine.add(
            "MOVE_ARM_SAFE",
            MoveArmUp(),
            transitions={"success": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
                        "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT"}
        )



        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            gms.move_arm("shelf_intermediate"),
            transitions={
                    "succeeded": "MOVE_BACKWARD",
                    "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_BACKWARD",
            MoveDBC(forward=False),
            transitions={"succeeded": "MOVE_BASE_USING_DBC_BACK"},
        )

        # Move base using direct base controller
        smach.StateMachine.add(
            "MOVE_BASE_USING_DBC_BACK",
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
                "success": "MOVE_ARM_TO_NEUTRAL",
                "timeout": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
                "failure": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            },
        )
    # ===============================================================================

# below states are for empty space placing, Anudeep

        smach.StateMachine.add(
            "TRIGGER_EMPTY_SPACE_DETECTION",
            TriggerEmptySpaceDetection(),
            transitions={"succeeded": "TRIGGER_POINT_CLOUD_PROCESSING", "failed": "START_PLACE_POSE_SELECTOR"},
        )

        smach.StateMachine.add(
            "TRIGGER_POINT_CLOUD_PROCESSING",
            TriggerPointCloudProcessing(),
            transitions={"succeeded": "PUBLISH_REFERENCE_FRAME_EMP", "failed": "START_PLACE_POSE_SELECTOR"},
        )

        smach.StateMachine.add(
            "PUBLISH_REFERENCE_FRAME_EMP",
            gbs.send_event([("/static_transform_publisher_node/event_in", "e_start")]),
            transitions={"success": "PUBLISH_OBJECT_POSE_AS_STATIC"},
        )

        
        smach.StateMachine.add(
            "PUBLISH_OBJECT_POSE_AS_STATIC",
            PublishEmptyspacePosetoBaseStatic(),
            transitions={ 
                "wbc_success": "SET_DBC_PARAMS_EMP",
                "nor_success": "CHECK_PICK_POSE_IK",
                "failed": "START_PLACE_POSE_SELECTOR"
            },
        )

        # WBC placing

        smach.StateMachine.add(
            "SET_DBC_PARAMS_EMP",
            gbs.set_named_config("dbc_pick_object"),
            transitions={
                "success": "SEND_STOP_EVENT_WBC",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SEND_STOP_EVENT_WBC",
            SendStopEvent(),
            transitions={"succeeded": "MOVE_ROBOT_AND_TRY_PLACING"},
        )

        smach.StateMachine.add(
            "MOVE_ROBOT_AND_TRY_PLACING",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_start")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=50,
            ),
            transitions={
                "success": "RELEASE_GRIPPER_WBC",
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
                    ("/empty_space_detector/event_in", "e_stop"),
                ]
            ),
            transitions={"success": "CHECK_PICK_POSE_IK"},
        )

        smach.StateMachine.add(
            "RELEASE_GRIPPER_WBC",
            gms.control_gripper('release'),
            transitions={"succeeded": "MOVE_ARM_UP_WBC", "timeout": "MOVE_ARM_UP_WBC"},
        )

        smach.StateMachine.add(
            "MOVE_ARM_UP_WBC",
            MoveArmUp(),
            transitions={"success": "MOVE_ARM_TO_NEUTRAL_WBC", "failed": "MOVE_ARM_TO_NEUTRAL"},
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_NEUTRAL_WBC",
            gms.move_arm("pre_place", use_moveit=False),
            transitions={"succeeded": "OPEN_GRIPPER", "failed": "MOVE_ARM_TO_NEUTRAL"},
        )

        # smach.StateMachine.add(
        #     "MOVE_DBC_IN_Y",
        #     MoveDBCPose(),
        #     transitions={"succeeded": "MOVE_BASE_USING_WBC"},
        # )

        # smach.StateMachine.add(
        #     "MOVE_BASE_USING_WBC",
        #     gbs.send_and_wait_events_combined(
        #         event_in_list=[("/mcr_navigation/direct_base_controller/coordinator/event_in", "e_start")],
        #         event_out_list=[("/mcr_navigation/direct_base_controller/coordinator/event_out", "e_success", True)],
        #         timeout_duration=10,
        #     ),
        #     transitions={"success": "OPEN_GRIPPER", "timeout": "OVERALL_FAILED", "failure": "OVERALL_FAILED"},
        # )

        # IK

        smach.StateMachine.add(
            "CHECK_PICK_POSE_IK",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/pregrasp_planner_node/event_in", "e_start")],
                event_out_list=[("/pregrasp_planner_node/event_out", "e_success", True)],
                timeout_duration=20,
            ),
            transitions={"success": "SEND_STOP_EVENT_NO_WBC", "timeout": "CHECK_PICK_POSE_IK", "failure": "OVERALL_FAILED"},
        )

        smach.StateMachine.add(
            "SEND_STOP_EVENT_NO_WBC",
            SendStopEvent(),
            transitions={"succeeded": "GO_TO_PICK_POSE"},
        )

        smach.StateMachine.add(
            "GO_TO_PICK_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/waypoint_trajectory_generation/event_in", "e_start")],
                event_out_list=[("/waypoint_trajectory_generation/event_out", "e_success", True)],
                timeout_duration=20,
            ),
            transitions={"success": "RELEASE_GRIPPER", "timeout": "OVERALL_FAILED", "failure": "OVERALL_FAILED"},
        )


  
    # ===============================================================================

    # below are state for default placing, Anudeep

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
                10.0,
            ),
            transitions={
                "succeeded": "MOVE_ARM_TO_PLACE_OBJECT",
                "failed": "MOVE_ARM_TO_DEFAULT_PLACE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_DEFAULT_PLACE",
            DefalutSafePose(),
            transitions={
                "succeeded": "MOVE_ARM_TO_PLACE_OBJECT",
                "failed": "MOVE_ARM_TO_DEFAULT_PLACE",
            },
        )
        smach.StateMachine.add(
            "MOVE_ARM_TO_PLACE_OBJECT",
            gms.move_arm(),
            transitions={"succeeded": "STOP_PLACE_POSE_SELECTOR", 
                         "failed": "STOP_PLACE_POSE_SELECTOR",
            },
        )

        smach.StateMachine.add(
            "STOP_PLACE_POSE_SELECTOR",
            gbs.send_event(
                [
                ("/mcr_perception/place_pose_selector/event_in", "e_stop"),
                 ("/empty_space_detector/event_in", "e_stop"),
                 ]
            ),
            # transitions={"success": "OPEN_GRIPPER"},
            transitions={"success": "RELEASE_GRIPPER"},
        )

    # ===============================================================================

        smach.StateMachine.add(
                "RELEASE_GRIPPER",
                gms.control_gripper('release'),
                transitions={
			        "succeeded": "MOVE_ARM_UP",
                         "timeout": "MOVE_ARM_UP"}
        )

        smach.StateMachine.add(
                "MOVE_ARM_UP",
                MoveArmUp(),
                transitions={
			        "success": "MOVE_ARM_TO_NEUTRAL",
                                 "failed": "MOVE_ARM_TO_NEUTRAL"}
        )


        smach.StateMachine.add(
                "MOVE_ARM_TO_NEUTRAL",
                gms.move_arm("pre_place", use_moveit=False),
                transitions={
                    "succeeded": "OPEN_GRIPPER",
                    "failed": "MOVE_ARM_TO_NEUTRAL",
            },
        )

        smach.StateMachine.add(
                "OPEN_GRIPPER",
                gms.control_gripper('open'),
                transitions={
			        "succeeded": "OVERALL_SUCCESS",
                                 "timeout": "OVERALL_SUCCESS"}
        )


        

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)
    sm.userdata.threshold_counter = 0
    sm.userdata.current_try = 0

    # smach viewer
    if rospy.get_param("~viewer_enabled", True):
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
