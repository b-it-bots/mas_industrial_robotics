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


class DefalutSafePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"],
                                    input_keys=["goal","move_arm_to", "pose_queue"],
                                    output_keys=["move_arm_to", "pose_queue"])
        
        self.full_pose_list = ["pose1", "pose2", "pose3", "pose4"]

    def execute(self, userdata):

        rospy.logwarn("Checking pre-defined safe pose")
        
        location = Utils.get_value_of(userdata.goal.parameters, "location")
        current_platform_height = rospy.get_param("/"+location)
        
        # refill pose queue if empty
        if not userdata.pose_queue:
            rospy.logwarn("Pose queue is empty, refilling with full pose list")
            userdata.pose_queue = self.full_pose_list.copy()    
            
        # pop a pose from the queue
        selected_pose = userdata.pose_queue.pop()
        
        # # Randomly select a pose from bmt_pose1 to pose4
        # random_pose_index = random.randint(1, 3)  # Generates a number between 1 and 4
        # selected_pose = f"pose{random_pose_index}"
        
        # userdata.move_arm_to = str(str(current_platform_height)+'pose4cm/')
        # print("from place server ========")
        
        # Construct the target pose string
        userdata.move_arm_to = f"{current_platform_height}cm/{selected_pose}"

        rospy.loginfo(f"Selected pose: {userdata.move_arm_to}")
        
        
        # print(userdata.move_arm_to)
        rospy.sleep(0.1)
        return "succeeded"



# ==============================================================================



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
    
    sm.userdata.pose_queue = ["pose1", "pose2", "pose3", "pose4"]

    # ===============================================================================

    with sm:
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
                    # "succeeded": "START_PLACE_POSE_SELECTOR",
                    "succeeded": "MOVE_ARM_TO_DEFAULT_PLACE",
                    "failed": "MOVE_ARM_TO_PRE_PLACE",
            },
        )


    # ===============================================================================

    # below are state for default placing, Anudeep

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
                [("/mcr_perception/place_pose_selector/event_in", "e_stop")]
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
