#!/usr/bin/python
import sys

import mcr_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms
import mir_states.common.basic_states as mir_gbs
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
from geometry_msgs.msg import TwistStamped

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import PoseStamped

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

        obj = Utils.get_value_of(userdata.goal.parameters, "object")
        rospy.logwarn('Using object "%s" from goal parameters.', obj)
        
        # set the ros param for the object to be selected
        rospy.set_param("~selected_object", obj)
        
        self.publisher.publish(String(data=obj))
        rospy.sleep(0.2)  # let the topic to survive for some time
        return "succeeded"

# ===============================================================================

class IsObjectLarge(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["large", "small"],
            input_keys=["goal", "large_objects"],
            output_keys=[],
        )

    def execute(self, userdata):
        obj = Utils.get_value_of(userdata.goal.parameters, "object")
        if obj is None:
            rospy.logwarn('Missing parameter "object". Using default.')
            return "large"
        for large_object in userdata.large_objects:
            if large_object.upper() in obj.upper():
                return "large"
        return "small"

# ===============================================================================

class ShouldDragPick(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["yes", "no"],
            input_keys=["goal", "drag_pick_objects"],
            output_keys=[],
        )

    def execute(self, userdata):
        obj = Utils.get_value_of(userdata.goal.parameters, "object")
        if obj is None:
            rospy.logwarn('Missing parameter "object". Using default.')
            return "no"
        for object_to_drag in userdata.drag_pick_objects:
            if object_to_drag.upper() in obj.upper():
                # return "yes"
                return "no" # until tested
        return "no"

# ===============================================================================

class ShouldReperceive(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["yes", "no"],
            input_keys=["reperceive"],
            output_keys=[],
        )

    def execute(self, userdata):
        if userdata.reperceive:
            return 'yes'
        else:
            return 'no'

# ===============================================================================

class MoveArmUp(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )
        # velocity publisher
        self.arm_velocity_pub = rospy.Publisher("/arm_1/arm_controller/cartesian_velocity_command", TwistStamped, queue_size=1)
    
    def execute(self, userdata):
        
        # send the velocity in +z direction wrt base_link to move the arm up
        vel_msg = TwistStamped()
        vel_msg.header.frame_id = "base_link"
        # set velocity to 5cm/s
        vel_msg.twist.linear.z = 0.04
        self.arm_velocity_pub.publish(vel_msg)
        rospy.sleep(1.5)
        # stop the arm
        vel_msg.twist.linear.z = 0
        self.arm_velocity_pub.publish(vel_msg)
        return "succeeded"

# =================================rc24===========================
class MoveArmDown(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )
        # velocity publisher
        self.arm_velocity_pub = rospy.Publisher("/arm_1/arm_controller/cartesian_velocity_command", TwistStamped, queue_size=1)
    
    def execute(self, userdata):
        
        # send the velocity in +z direction wrt base_link to move the arm up
        vel_msg = TwistStamped()
        vel_msg.header.frame_id = "base_link"
        # set velocity to 5cm/s
        vel_msg.twist.linear.z = -0.04  #to do
        self.arm_velocity_pub.publish(vel_msg)
        rospy.sleep(1.5)
        # stop the arm
        vel_msg.twist.linear.z = 0
        self.arm_velocity_pub.publish(vel_msg)
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

# by Anudep
class SetWBCThresholdParam(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["goal", "ws_virtual"],
        )

    def execute(self, userdata):
        location = Utils.get_value_of(userdata.goal.parameters, "location")

        if location in userdata.ws_virtual:
            rospy.set_param("~reduce_wbc_with_threshold", True)
            rospy.loginfo(f"Location '{location}' is in virtual wall list. Set reduce_wbc_with_threshold to True.")
        else:
            rospy.set_param("~reduce_wbc_with_threshold", False)
            rospy.loginfo(f"Location '{location}' not in virtual wall list. Set reduce_wbc_with_threshold to False.")
        return "succeeded"
    

# ===============================================================================
# State to check if we should move back
# ===============================================================================

class ShouldMoveBack(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["yes", "no"],
            input_keys=["goal", "ws_virtual"],
        )

    def execute(self, userdata):
        # Check if location is in virtual wall list
        location = Utils.get_value_of(userdata.goal.parameters, "location")
        if location is None:
            rospy.logwarn('Missing parameter "location". Not moving back.')
            return "no"
            
        if location.upper() in [ws.upper() for ws in userdata.ws_virtual]:
            # Check if clamped pose is different from initial value
            clamped_pose = rospy.get_param("~clamped_base_pose", {})
            if 'x' in clamped_pose and clamped_pose['x'] != 0.0:
                return "yes"
            if 'y' in clamped_pose and clamped_pose['y'] != 0.0:
                return "yes"
        return "no"

# ==================================================================================
# New states for moving back after pick
# ===============================================================================

class PublishClampedPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.publisher = rospy.Publisher(
            "/mcr_navigation/direct_base_controller/input_pose",
            PoseStamped,
            queue_size=1
        )

    def execute(self, userdata):
        
        
        # Get clamped pose from parameter server
        clamped_pose_dict = rospy.get_param("~clamped_base_pose", {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        })
        
        # Create PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = "base_link_static"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = clamped_pose_dict['x']
        pose.pose.position.y = clamped_pose_dict['y']
        pose.pose.position.z = clamped_pose_dict['z']
        pose.pose.orientation.x = clamped_pose_dict['orientation']['x']
        pose.pose.orientation.y = clamped_pose_dict['orientation']['y']
        pose.pose.orientation.z = clamped_pose_dict['orientation']['z']
        pose.pose.orientation.w = clamped_pose_dict['orientation']['w']
        
        # Publish the pose
        self.publisher.publish(pose)
        rospy.loginfo("Published clamped pose to DBC")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        rospy.sleep(0.5)  # Allow some time for the message to be sent
        return 'succeeded'

# ===============================================================================

# State to reset clamped pose parameter
# ===============================================================================

class ResetClampedPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.initial_pose = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0
            }
        }

    def execute(self, userdata):
        rospy.set_param("~clamped_base_pose", self.initial_pose)
        rospy.loginfo("Reset clamped pose parameter to initial values")
        return 'succeeded'
    

# ===============================================================================

def main():
    # Open the container
    rospy.init_node("pick_object_wbc_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )

    # read large object list
    sm.userdata.large_objects = rospy.get_param("~large_objects", ["ALLEN_KEY", "SCREWDRIVER", "WRENCH", "DRILL"])
    sm.userdata.drag_pick_objects = rospy.get_param("~drag_pick_objects", ["ALLEN_KEY","WRENCH"])
    sm.userdata.reperceive = rospy.get_param("~reperceive", True) 
    # workstations with virtual walls
    # sm.userdata.ws_virtual = ["WS05", "WS06"]
    sm.userdata.ws_virtual = rospy.get_param("~ws_virtual", ["WS05", "WS06"])

    with sm:
        smach.StateMachine.add(
            "SET_PREGRASP_PARAMS",
            gbs.set_named_config("pregrasp_planner_no_sampling"),
            transitions={
                # "success": "SELECT_OBJECT",
                "success": "SET_WBC_THRESHOLD_PARAM",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )
        
        smach.StateMachine.add(
            "SET_WBC_THRESHOLD_PARAM",
            SetWBCThresholdParam(),
            transitions={"succeeded": "SELECT_OBJECT"},
        )

        smach.StateMachine.add(
            "SELECT_OBJECT",
            SelectObject("/mcr_perception/object_selector/input/object_name"),
            transitions={"succeeded": "GENERATE_OBJECT_POSE"},
        )

        # generates a pose of object
        smach.StateMachine.add(
            "GENERATE_OBJECT_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_perception/object_selector/event_in", "e_trigger")
                ],
                event_out_list=[
                    ("/mcr_perception/object_selector/event_out", "e_selected", True,)
                ],
                timeout_duration=10,
            ),
            transitions={
                "success": "SET_DBC_PARAMS",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SET_DBC_PARAMS",
            gbs.set_named_config("dbc_pick_object"),
            transitions={
                "success": "MOVE_ROBOT_AND_TRY_PICKING",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        # whole body control command. It moves direct base controller and
        # checks if an IK soln exists for the arm.
        smach.StateMachine.add(
            "MOVE_ROBOT_AND_TRY_PICKING",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_try")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=50,
            ),
            transitions={
                "success": "CHECK_IF_REPERCEIVE",
                "timeout": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
                "failure": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
            },
        )

        smach.StateMachine.add(
            "CHECK_IF_REPERCEIVE",
            ShouldReperceive(),
            transitions={
                "yes": "OPEN_GRIPPER_FOR_REPERCEIVE",
                "no": "GENERATE_OBJECT_POSE_AGAIN",
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_FOR_REPERCEIVE",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ARM",
                         "timeout": "MOVE_ARM"},
        )

        # move arm to appropriate position
        smach.StateMachine.add(
            "MOVE_ARM",
            gms.move_arm("pre_place", use_moveit=False),
            transitions={
                "succeeded": "START_OBJECT_LIST_MERGER",
                "failed": "MOVE_ARM",
            },
        )

        smach.StateMachine.add(
            "START_OBJECT_LIST_MERGER",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/mcr_perception/object_list_merger/event_in", "e_start")],
                event_out_list=[("/mcr_perception/object_list_merger/event_out", "e_started", True)],
                timeout_duration=5,
            ),
            transitions={
                "success": "START_OBJECT_RECOGNITION",
                "timeout": "TRY_PICKING",
                "failure": "TRY_PICKING",
            },
        )

        # New perception pipeline state machine
        smach.StateMachine.add(
            "START_OBJECT_RECOGNITION",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/mir_perception/multimodal_object_recognition/event_in", "e_start")],
                event_out_list=[("/mir_perception/multimodal_object_recognition/event_out", "e_done", True)],
                timeout_duration=10,
            ),
            transitions={
                "success": "STOP_RECOGNITION",
                "timeout": "TRY_PICKING",
                "failure": "TRY_PICKING",
            },
        )

        smach.StateMachine.add(
            "STOP_RECOGNITION",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/mir_perception/multimodal_object_recognition/event_in", "e_stop")],
                event_out_list=[("/mir_perception/multimodal_object_recognition/event_out", "e_stopped", True)],
                timeout_duration=5,
            ),
            transitions={
                "success": "STOP_OBJECT_LIST_MERGER",
                "timeout": "TRY_PICKING",
                "failure": "TRY_PICKING",
            },
        )

        smach.StateMachine.add(
            "STOP_OBJECT_LIST_MERGER",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/mcr_perception/object_list_merger/event_in", "e_stop")],
                event_out_list=[("/mcr_perception/object_list_merger/event_out", "e_stopped", True)],
                timeout_duration=5,
            ),
            transitions={
                "success": "PUBLISH_MERGED_OBJECT_LIST",
                "timeout": "TRY_PICKING",
                "failure": "TRY_PICKING",
            },
        )

        smach.StateMachine.add(
            "PUBLISH_MERGED_OBJECT_LIST",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/mcr_perception/object_list_merger/event_in", "e_trigger_local")],
                event_out_list=[("/mcr_perception/object_list_merger/event_out", "e_done", True)],
                timeout_duration=5,
            ),
            transitions={
                "success": "SELECT_OBJECT_AGAIN",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SELECT_OBJECT_AGAIN",
            SelectObject("/mcr_perception/local_object_selector/input/object_name"),
            transitions={"succeeded": "GENERATE_UPDATED_OBJECT_POSE"},
        )

        # generates a pose of object
        smach.StateMachine.add(
            "GENERATE_UPDATED_OBJECT_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/mcr_perception/local_object_selector/event_in", "e_trigger")],
                event_out_list=[("/mcr_perception/local_object_selector/event_out", "e_selected", True)],
                timeout_duration=10,
            ),
            transitions={
                "success": "CHECK_IF_OBJECT_LARGE_LOCAL",
                "timeout": "GENERATE_OBJECT_POSE_AGAIN",
                "failure": "GENERATE_OBJECT_POSE_AGAIN",
            },
        )

        # generates a pose of object
        smach.StateMachine.add(
            "GENERATE_OBJECT_POSE_AGAIN",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/mcr_perception/object_selector/event_in", "e_re_trigger")],
                event_out_list=[("/mcr_perception/object_selector/event_out", "e_selected", True)],
                timeout_duration=10,
            ),
            transitions={
                "success": "CHECK_IF_OBJECT_LARGE",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "CHECK_IF_OBJECT_LARGE_LOCAL",
            IsObjectLarge(),
            transitions={
                "large": "OPEN_GRIPPER_WIDE_LOCAL",
                "small": "OPEN_GRIPPER_NARROW_LOCAL",
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_WIDE_LOCAL",
            gms.control_gripper("open"),
            transitions={"succeeded": "TRY_PICKING",
                         "timeout": "TRY_PICKING"},
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_NARROW_LOCAL",
            gms.control_gripper("open_narrow"),
            transitions={"succeeded": "TRY_PICKING",
                         "timeout": "TRY_PICKING"},
        )

        # move only arm for wbc
        smach.StateMachine.add(
            "TRY_PICKING",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_start_arm_only")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=20,
            ),
            transitions={
                "success": "CLOSE_GRIPPER",
                "timeout": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
                "failure": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
            },
        )
        
        
        smach.StateMachine.add(
            "CHECK_IF_OBJECT_LARGE",
            IsObjectLarge(),
            transitions={
                "large": "OPEN_GRIPPER_WIDE",
                "small": "OPEN_GRIPPER_NARROW",
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_WIDE",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ROBOT_AND_PICK",
                         "timeout": "MOVE_ROBOT_AND_PICK"},
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_NARROW",
            gms.control_gripper("open_narrow"),
            transitions={"succeeded": "MOVE_ROBOT_AND_PICK",
                         "timeout": "MOVE_ROBOT_AND_PICK"},
        )

        # whole body control command. It moves direct base controller and
        # calls pre-grasp planner, and (optionally) moves arm to object pose
        smach.StateMachine.add(
            "MOVE_ROBOT_AND_PICK",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_start")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=50,
            ),
            transitions={
                "success": "CLOSE_GRIPPER",
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
        
        #==========================================================
        
        # smach.StateMachine.add(
        #     "CHECK_IF_OBJECT_SHOULD_BE_DRAGGED",
        #     ShouldDragPick(),
        #     transitions={"yes":"DRAG_PICK",
        #                  "no":"MOVE_ARM_TO_PRE_PLACE"}
        # )

        # smach.StateMachine.add(
        #     "DRAG_PICK",
        #     gbs.send_and_wait_events_combined(
        #         event_in_list=[("/wbc/event_in", "e_start_drag")],
        #         event_out_list=[("/wbc/event_out", "e_success", True)],
        #         timeout_duration=50,
        #     ),
        #     transitions={
        #         "success": "MOVE_ARM_TO_PRE_PLACE",
        #         "timeout": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
        #         "failure": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
        #     },
        # )
        
        ###################################################################
        
        smach.StateMachine.add(
            "CLOSE_GRIPPER",
            gms.control_gripper("close"),
            transitions={"succeeded": "MOVE_ARM_UP",
                         "timeout": "MOVE_ARM_UP"},
        )
        # move up 5 cm and then verify 

        smach.StateMachine.add(
            "MOVE_ARM_UP",
            MoveArmUp(),
            transitions={
                "succeeded": "VERIFY_OBJECT_GRASPED"
            },
        )

        smach.StateMachine.add(
            "VERIFY_OBJECT_GRASPED",
            gms.verify_object_grasped(3),
            transitions={
                "succeeded": "MOVE_ARM_TO_PRE_PLACE",
                "timeout": "OPEN_GRASP_FAILURE",
                "failed": "OPEN_GRASP_FAILURE",
            },
        )

        smach.StateMachine.add(
            "OPEN_GRASP_FAILURE",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ARM_DOWN",
                         "timeout": "MOVE_ARM_DOWN"},
        )

        #================================RC24==============

        smach.StateMachine.add(
            "MOVE_ARM_DOWN",
            MoveArmDown(),
            transitions={
                "succeeded": "CLOSE_GRIPPER_REPICK"
            },
        )

        smach.StateMachine.add(
            "CLOSE_GRIPPER_REPICK",
            gms.control_gripper("close"),
            transitions={"succeeded": "MOVE_ARM_UP_AGAIN",
                         "timeout": "MOVE_ARM_UP_AGAIN"},
        )

        smach.StateMachine.add(
            "MOVE_ARM_UP_AGAIN",
            MoveArmUp(),
            transitions={
                "succeeded": "MOVE_ARM_TO_PRE_PLACE"
            },
        )

        #=================================================

        # #not being used
        # smach.StateMachine.add(
        #     "MOVE_TO_PRE_PLACE_AND_FAIL",
        #     gms.move_arm("pre_place", use_moveit=True),
        #     transitions={
        #         "succeeded": "OVERALL_FAILED",
        #         "failed": "MOVE_TO_PRE_PLACE_AND_FAIL",
        #     },
        # )

        #=====================================================

        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_PLACE",
            gms.move_arm("pre_place", use_moveit=True),
            transitions={
                "succeeded": "CHECK_IF_OBJECT_LARGE_FOR_STAGING",
                "failed": "CHECK_IF_OBJECT_LARGE_FOR_STAGING",
            },
        )

        smach.StateMachine.add(
            "CHECK_IF_OBJECT_LARGE_FOR_STAGING",
            IsObjectLarge(),
            transitions={
                "large": "MOVE_ARM_TO_PRE_PLACE_INTER",
                "small": "OVERALL_SUCCESS",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_PLACE_INTER",
            gms.move_arm("platform_stage_inter", use_moveit=True),
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "failed": "MOVE_ARM_TO_PRE_PLACE_INTER",
            },
        )
        
        # # Check if we should move back
        # smach.StateMachine.add(
        #     "SHOULD_MOVE_BACK",
        #     ShouldMoveBack(),
        #     transitions={
        #         "yes": "PUBLISH_CLAMPED_POSE",
        #         "no": "OVERALL_SUCCESS"
        #     },
        # )
        
        # # Publish clamped pose to DBC
        # smach.StateMachine.add(
        #     "PUBLISH_CLAMPED_POSE",
        #     PublishClampedPose(),
        #     transitions={"succeeded": "MOVE_BASE_BACK",
        #                  "preempted": "MOVE_BASE_BACK", 
        #     }
        # )
        
        # # Trigger DBC to move to clamped pose
        # smach.StateMachine.add(
        #     "MOVE_BASE_BACK",
        #     gbs.send_and_wait_events_combined(
        #         event_in_list=[
        #             ("/mcr_navigation/direct_base_controller/coordinator/event_in", "e_start")
        #         ],
        #         event_out_list=[
        #             ("/mcr_navigation/direct_base_controller/coordinator/event_out", "e_success", True)
        #         ],
        #         timeout_duration=10,
        #     ),
        #     transitions={
        #         "success": "RESET_CLAMPED_POSE",
        #         "timeout": "RESET_CLAMPED_POSE",  # Continue even if timeout
        #         "failure": "RESET_CLAMPED_POSE"   # Continue even if failure
        #     }
        # )
        
        #  # Reset clamped pose parameter
        # smach.StateMachine.add(
        #     "RESET_CLAMPED_POSE",
        #     ResetClampedPose(),
        #     transitions={"succeeded": "OVERALL_SUCCESS"}
        # )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

    # smach viewer
    if rospy.get_param("~viewer_enabled", True):
        sis = IntrospectionServer(
            "pick_object_smach_viewer", sm, "/PICK_OBJECT_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="wbc_pick_object_server",
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
