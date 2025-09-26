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
    # Open the container
    rospy.init_node("tc_pick_object_wbc_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )

    with sm:
        smach.StateMachine.add(
            "SET_PREGRASP_PARAMS",
            gbs.set_named_config("pregrasp_planner_no_sampling"),
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
                "success": "MOVE_ROBOT_AND_PICK",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
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
            transitions={"succeeded": "MOVE_TO_PRE_PLACE_AND_FAIL",
                         "timeout": "MOVE_TO_PRE_PLACE_AND_FAIL"},
        )

        smach.StateMachine.add(
            "MOVE_TO_PRE_PLACE_AND_FAIL",
            gms.move_arm("pre_place", use_moveit=True),
            transitions={
                "succeeded": "OVERALL_FAILED",
                "failed": "MOVE_TO_PRE_PLACE_AND_FAIL",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_PLACE",
            gms.move_arm("pre_place", use_moveit=True),
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "failed": "MOVE_ARM_TO_PRE_PLACE",
            },
        )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

    # smach viewer
    if rospy.get_param("~viewer_enabled", True):
        sis = IntrospectionServer(
            "tc_pick_object_smach_viewer", sm, "/TC_PICK_OBJECT_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="tc_wbc_pick_object_server",
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
