#!/usr/bin/env python

import mir_states.common.manipulation_states as gms  # move the arm, and gripper
import mir_states.common.basic_states as mir_gbs
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
import mcr_states.common.basic_states as gbs
from geometry_msgs.msg import PoseStamped
import tf

import tf2_ros
import tf2_geometry_msgs

# ===============================================================================

class SetupMoveArm(smach.State):
    def __init__(self, arm_target):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["goal"],
            output_keys=["feedback", "result", "move_arm_to"],
        )
        self.arm_target = arm_target

    def execute(self, userdata):
        platform = Utils.get_value_of(userdata.goal.parameters, "platform")
        
        # get selected objet from ros parameter
        selected_object = rospy.get_param("/wbc_pick_object_server/selected_object", None)
        large_objects = rospy.get_param("/wbc_pick_object_server/large_objects",None)
        
        print("Large objects: ", large_objects)
        print(type(large_objects))
        
        if platform is None:
            rospy.logwarn('Missing parameter "platform". Using default.')
            platform = "PLATFORM_LEFT"
        platform = platform.lower()
        
        

        if self.arm_target == "pre":
            if selected_object in large_objects:
                platform += "_rot"
            else:
                platform += "_pre"
    
        elif self.arm_target == "final":
            if selected_object in large_objects:
                platform += "_screw"
            else:
                platform = platform
        
        print("Platform to move arm to: ", platform)

        userdata.move_arm_to = platform

        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="SetupMoveArm", text="Moving arm to " + platform
        )
        return "succeeded"

# ===============================================================================

# State to check if we should move back
# ===============================================================================

class ShouldMoveBack(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["yes", "no"],
        )

    def execute(self, userdata):
        # Get initial pose values (0,0,0 with identity orientation)
        initial_pose = {
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
        
        # Get current clamped pose
        clamped_pose = rospy.get_param("/wbc_pick_object_server/clamped_base_pose", initial_pose)
        
        # Check if any component is different from initial
        if (clamped_pose['x'] != initial_pose['x'] or
            clamped_pose['y'] != initial_pose['y'] ):
        
            rospy.loginfo("Clamped pose is non-zero, should move back")
            return "yes"
        
        rospy.loginfo("Clamped pose is at initial values, no need to move back")
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
        self.listener = tf.TransformListener()  # INITIALIZE LISTENER


    def execute(self, userdata):
        
        
        # Get clamped pose from parameter server
        clamped_pose_dict = rospy.get_param("/wbc_pick_object_server/clamped_base_pose", {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        })
        
        # get tf of base_link 
        tf_msg = self.listener.lookupTransform("/base_link_static", "/base_link", rospy.Time(0))
        
        # Create PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = "base_link_static"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = tf_msg[0][0]
        pose.pose.position.y = tf_msg[0][1] - clamped_pose_dict['y']
        pose.pose.position.z = tf_msg[0][2]
        pose.pose.orientation.x = tf_msg[1][0]
        pose.pose.orientation.y = tf_msg[1][1]
        pose.pose.orientation.z = tf_msg[1][2]
        pose.pose.orientation.w = tf_msg[1][3]
        
        # Publish the pose
        self.publisher.publish(pose)
        rospy.loginfo("Published clamped pose to DBC")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        rospy.sleep(0.5)  # Allow some time for the message to be sent
        return 'succeeded'


# reset /wbc_pick_object_server/selected_object rosparam to Default
class ResetRosparam(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.initial_object = "M20"
        
    def execute(self, userdata):
        try:
            rospy.set_param("/wbc_pick_object_server/selected_object", self.initial_object)
            rospy.loginfo("Reset selected object parameter to initial value: %s", self.initial_object)
            return 'succeeded'
        except Exception as e:
            rospy.logerr("Failed to reset selected object parameter: %s", str(e))
            return 'failed'
    

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
        rospy.set_param("/wbc_pick_object_server/clamped_base_pose", self.initial_pose)
        rospy.loginfo("Reset clamped pose parameter to initial values")
        return 'succeeded'
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
    rospy.init_node("stage_object_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )


    with sm:
        # add states to the container
        # smach.StateMachine.add(
        #     "MOVE_ARM_TO_STAGE_INTERMEDIATE",
        #     gms.move_arm("pre_place", use_moveit=False),
        #     transitions={
        #         "succeeded": "SETUP_MOVE_ARM_PRE_STAGE",
        #         "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE",
        #     },
        # )

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
            gms.move_arm(use_moveit=False),
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
            gms.move_arm(use_moveit=False),
            transitions={
                "succeeded": "RESET_ROS_PARAM",
                "failed": "MOVE_ARM_STAGE"
            },
        )

    
        # Reset the rosparam to default
        smach.StateMachine.add(
            "RESET_ROS_PARAM",
            ResetRosparam(),
            transitions={"succeeded": "OPEN_GRIPPER",
                         "failed": "OPEN_GRIPPER"},
        )
        
        ######################################

        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper("open_narrow"),
            transitions={"succeeded": "SETUP_MOVE_ARM_RETRACT",
                         "timeout": "SETUP_MOVE_ARM_RETRACT"},
        )
        

        smach.StateMachine.add(
            "SETUP_MOVE_ARM_RETRACT",
            SetupMoveArm("pre"),
            transitions={
                "succeeded": "MOVE_ARM_RETRACT",
                "failed": "SETUP_MOVE_ARM_RETRACT",
            },
        )
        smach.StateMachine.add(
            "MOVE_ARM_RETRACT",
            gms.move_arm(use_moveit=False),
            transitions={
                "succeeded": "MOVE_ARM_TO_STAGE_INTERMEDIATE_RETRACT",
                "failed": "MOVE_ARM_RETRACT"
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_STAGE_INTERMEDIATE_RETRACT",
            gms.move_arm("pre_place", use_moveit=False),
            transitions={
                "succeeded": "SHOULD_MOVE_BACK",
                "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE_RETRACT",
            },
        )
        
        # Check if we should move back
        smach.StateMachine.add(
            "SHOULD_MOVE_BACK",
            ShouldMoveBack(),
            transitions={
                "yes": "PUBLISH_REFERENCE_FRAME",
                "no": "RESET_CLAMPED_POSE"
            },
        )
        
        smach.StateMachine.add(
            "PUBLISH_REFERENCE_FRAME",
            gbs.send_event([("/static_transform_publisher_node/event_in", "e_start")]),
            transitions={"success": "SET_DBC_PARAMS"},
        )

        smach.StateMachine.add(
            "SET_DBC_PARAMS",
            gbs.set_named_config("dbc_pick_object"),
            transitions={
                "success": "PUBLISH_CLAMPED_POSE",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )
        
        # Publish clamped pose to DBC
        smach.StateMachine.add(
            "PUBLISH_CLAMPED_POSE",
            PublishClampedPose(),
            transitions={"succeeded": "MOVE_BASE_BACK", "preempted": "MOVE_BASE_BACK" }  
        )
        
        # Trigger DBC to move to clamped pose
        smach.StateMachine.add(
            "MOVE_BASE_BACK",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mcr_navigation/direct_base_controller/coordinator/event_in", "e_start")
                ],
                event_out_list=[
                    ("/mcr_navigation/direct_base_controller/coordinator/event_out", "e_success", True)
                ],
                timeout_duration=10,
            ),
            transitions={
                "success": "RESET_CLAMPED_POSE",
                "timeout": "RESET_CLAMPED_POSE",  # Continue even if timeout
                "failure": "RESET_CLAMPED_POSE"   # Continue even if failure
            }
        )
        
        # Reset clamped pose parameter
        smach.StateMachine.add(
            "RESET_CLAMPED_POSE",
            ResetClampedPose(),
            transitions={"succeeded": "OVERALL_SUCCESS"}
        )
        

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

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