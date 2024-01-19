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
)
from smach_ros import ActionServerWrapper, IntrospectionServer
from std_msgs.msg import String, Float64
from mas_perception_msgs.msg import TimeStampedPose
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

# ===============================================================================

class GetPredictions(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "timeout"],
            input_keys=["goal", "current_rtt_pick_retry"],
            output_keys=["feedback", "result", "pose", "time"],
        )
        self.publisher = rospy.Publisher(topic_name, String, queue_size=10)
    
    def execute(self, userdata):
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="PredictObjectPose", text="predicting object pose"
        )

        obj = Utils.get_value_of(userdata.goal.parameters, "object")
        msg_str = f'e_start_{obj}'
        self.publisher.publish(String(data=msg_str))
        rospy.sleep(0.2)  # let the topic to survive for some time\
        try:
            predicted_msg = rospy.wait_for_message("/mir_perception/rtt/time_stamped_pose", TimeStampedPose, timeout=90)
            userdata.time = predicted_msg.timestamps
            userdata.pose = predicted_msg.pose
            return "succeeded"
        except Exception as e:
            rospy.logwarn("Exception occured while waiting for object TimeStampedPose")
            return "timeout"
        
class WaitForObject(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["time", "time_taken"],
            output_keys=["feedback", "result", "time_taken"],
        )   
        # self.publisher = rospy.Publisher("/mir_perception/rtt_grasp/event_in", String, queue_size=10)
        self.subscriber = rospy.Subscriber("/mir_perception/rtt/time_stamped_pose", TimeStampedPose, self.callback)
        self.time = None

    def callback(self, msg):
        self.time = msg.timestamps
    
    def execute(self, userdata):
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="WaitForObject", text="waiting for object"
        )

        userdata.time_taken = rospy.Time.now().to_sec()
        if rospy.Time.now().to_sec() > userdata.time - 0.75: 
            return "failed"
        while rospy.Time.now().to_sec() < userdata.time - 0.75: # This is the time to close the gripper for picking. The higher the time the earlier the gripper will close
            rospy.sleep(0.01)
        return "succeeded"
        


class SetupObjectPose(smach.State):
    def __init__(self, state):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["pose"],
            output_keys=["feedback", "result", "move_arm_to", "move_arm_to_pick"],
        )

        self.pick_pose = rospy.Publisher(
            "/mcr_perception/object_selector/output/object_pose",
            PoseStamped,
            queue_size=10,
        )

        self.state = state

    def execute(self, userdata):
        # #iCartesian pose (x, y, z, r, p, y, frame_id)
        r, p, y = tr.euler_from_quaternion([userdata.pose.pose.orientation.x, userdata.pose.pose.orientation.y, userdata.pose.pose.orientation.z, userdata.pose.pose.orientation.w])

        # convert rpy into quaternion
        quaternion = tr.quaternion_from_euler(r, p, y)

        move_arm_to = PoseStamped()
        move_arm_to.header.frame_id = userdata.pose.header.frame_id
        move_arm_to.pose.position.x = userdata.pose.pose.position.x
        move_arm_to.pose.position.y = userdata.pose.pose.position.y
        move_arm_to.pose.position.z = userdata.pose.pose.position.z + 0.05

        move_arm_to.pose.orientation.x = quaternion[0]
        move_arm_to.pose.orientation.y = quaternion[1]
        move_arm_to.pose.orientation.z = quaternion[2]
        move_arm_to.pose.orientation.w = quaternion[3]

        move_arm_to_pick = PoseStamped()
        move_arm_to_pick.header.frame_id = userdata.pose.header.frame_id
        move_arm_to_pick.pose.position.x = userdata.pose.pose.position.x
        move_arm_to_pick.pose.position.y = userdata.pose.pose.position.y
        move_arm_to_pick.pose.position.z = userdata.pose.pose.position.z

        move_arm_to_pick.pose.orientation.x = quaternion[0]
        move_arm_to_pick.pose.orientation.y = quaternion[1]
        move_arm_to_pick.pose.orientation.z = quaternion[2]
        move_arm_to_pick.pose.orientation.w = quaternion[3]

        
        if self.state == "pre_pick_pose":
            rospy.logwarn(move_arm_to)
            self.pick_pose.publish(move_arm_to)
        elif self.state == "pick_pose":
            rospy.logwarn(move_arm_to_pick)
            self.pick_pose.publish(move_arm_to_pick)

        
        # TODO: to check if position is not nan
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="SetupObjectPose", text="Setting pose for RTT"
        )
        return "succeeded"
    
# ===============================================================================

class CheckRetries(smach.State):
    def __init__(self, state=False):
        smach.State.__init__(
            self,
            outcomes=["retry", "no_retry", "succeeded"],
            input_keys=["current_rtt_pick_retry", "rtt_pick_retries"],
            output_keys=["current_rtt_pick_retry"],
        )
        self.publisher = rospy.Publisher("/mir_perception/rtt/event_in", String, queue_size=10)
        self.state = state

    def execute(self, userdata):
        if self.state:
            msg_str = f'e_stop'
            self.publisher.publish(String(data=msg_str))
            return "succeeded"
        else:
            if userdata.current_rtt_pick_retry < userdata.rtt_pick_retries:
                userdata.current_rtt_pick_retry += 1
                msg_str = f'e_stop'
                self.publisher.publish(String(data=msg_str))
                return "retry"
            else:
                userdata.current_rtt_pick_retry = 0
                msg_str = f'e_stop'
                self.publisher.publish(String(data=msg_str))
                return "no_retry"

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
    rospy.init_node("rtt_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )

    # rtt retries
    sm.userdata.rtt_pick_retries = 1
    sm.userdata.current_rtt_pick_retry = 0

    with sm:
        smach.StateMachine.add(
            "SET_PREGRASP_PARAMS",
            gbs.set_named_config("pregrasp_planner_no_sampling"),
            transitions={
                "success": "GO_PRE_PERCEIVE",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "GO_PRE_PERCEIVE",
            gms.move_arm("pre_place", use_moveit=False),
            transitions={
                "succeeded": "SELECT_OBJECT",
                "failed": "GO_PRE_PERCEIVE",
            }
        )

        smach.StateMachine.add(
            "SELECT_OBJECT",
            GetPredictions("/mir_perception/rtt/event_in"),
            transitions={"succeeded": "OPEN_GRIPPER",
                        "timeout": "RETRY_PICK"}
        )
        # TODO: add timeout to this state

        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper(-1.4),
            transitions={"succeeded": "SETUP_OBJECT_POSE_PICK",
                         "timeout": "SETUP_OBJECT_POSE_PICK"},
        )

        # Set the pose to pre pose for picking
        smach.StateMachine.add(
            "SETUP_OBJECT_POSE_PICK",
            SetupObjectPose("pick_pose"),
            transitions={"succeeded": "CHECK_PICK_POSE_IK",
                          "failed": "SETUP_OBJECT_POSE_PICK"},
        )

        smach.StateMachine.add(
            "CHECK_PICK_POSE_IK",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/pregrasp_planner_node/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/pregrasp_planner_node/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "GO_TO_PICK_POSE",
                "timeout": "CHECK_PICK_POSE_IK", 
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
        "GO_TO_PICK_POSE",
        gbs.send_and_wait_events_combined(
            event_in_list=[
                ("/waypoint_trajectory_generation/event_in", "e_start")],
            event_out_list=[
                (
                    "/waypoint_trajectory_generation/event_out",
                    "e_success",
                    True,
                )],
            timeout_duration=20,
        ),
        transitions={
            "success": "WAIT_FOR_OBJECT", 
            "timeout": "OVERALL_FAILED",
            "failure": "OVERALL_FAILED",
        },
        )

        smach.StateMachine.add(
            "WAIT_FOR_OBJECT",
            WaitForObject(),
            transitions={
                "succeeded": "CLOSE_GRIPPER",
                "failed": "RETRY_PICK",
            },
        )

        smach.StateMachine.add(
            "CLOSE_GRIPPER",
            gms.control_gripper("close", timeout=0.5),
            transitions={"succeeded": "MOVE_ARM_TO_STAGE_INTERMEDIATE",
                         "timeout": "MOVE_ARM_TO_STAGE_INTERMEDIATE"},
        )

        # move arm to stage_intemediate position
        smach.StateMachine.add(
            "MOVE_ARM_TO_STAGE_INTERMEDIATE",
            gms.move_arm("barrier_tape", use_moveit=False),
            transitions={
                "succeeded": "VERIFY_OBJECT_GRASPED",
                "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE",
            },
        )

        smach.StateMachine.add(
            "VERIFY_OBJECT_GRASPED",
            gms.verify_object_grasped(2),
            transitions={
                "succeeded": "OBJECT_GRASPED",
                "timeout": "OBJECT_GRASPED",
                "failed": "RETRY_PICK",
            },
        )

        # retry if failed
        smach.StateMachine.add(
            "OBJECT_GRASPED",
            CheckRetries(state=True),
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "retry": "OVERALL_SUCCESS",
                "no_retry": "OVERALL_SUCCESS",
            },
        )
        
        # retry if failed
        smach.StateMachine.add(
            "RETRY_PICK",
            CheckRetries(state=False),
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "retry": "SELECT_OBJECT",
                "no_retry": "OVERALL_FAILED",
            },
        )

    sm.userdata.current_rtt_pick_retry = 0
    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)


    # # smach viewer
    # if rospy.get_param("~viewer_enabled", True):
    #     sis = IntrospectionServer(
    #         "rtt_pick_object_smach_viewer", sm, "/RTT_PICK_OBJECT_SMACH_VIEWER"
    #     )
    #     sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="rtt_server",
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