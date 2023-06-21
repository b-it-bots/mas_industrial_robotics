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
from std_msgs.msg import String
from mas_perception_msgs.msg import TimeStampedPose
import tf.transformations as tr

# ===============================================================================

class GetPredictions(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["goal"],
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
        predicted_msg = rospy.wait_for_message("/mir_perception/rtt/time_stamped_pose", TimeStampedPose)
        pred_pose = predicted_msg.pose
        userdata.pose = pred_pose
        pred_times = predicted_msg.timestamps
        # print("*"*50)
        # print('pred_pose', pred_pose)
        # print('pred_times', pred_times)
        # print("*"*50)
        userdata.time = pred_times
        return "succeeded"
        
class WaitForObject(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["time"],
            output_keys=["feedback", "result"],
        )
    
    def execute(self, userdata):
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="WaitForObject", text="waiting for object"
        )
        print("userdata.time", userdata.time)
        print("rospy.Time.now().to_sec()", rospy.Time.now().to_sec())
        print("\n\n"*2)
        while rospy.Time.now().to_sec() < userdata.time - 1.5: # 2.1 is the time it takes to move arm to pick
            rospy.sleep(0.01)
        return "succeeded"

class SetupObjectPose(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["pose"],
            output_keys=["feedback", "result", "move_arm_to"],
        )
    
    def execute(self, userdata):
        #iCartesian pose (x, y, z, r, p, y, frame_id)
        r, p, y = tr.euler_from_quaternion([userdata.pose.pose.orientation.x, userdata.pose.pose.orientation.y, userdata.pose.pose.orientation.z, userdata.pose.pose.orientation.w])
        userdata.move_arm_to = [userdata.pose.pose.position.x, userdata.pose.pose.position.y, userdata.pose.pose.position.z, r, p, y, userdata.pose.header.frame_id]
        # TODO: to check if position is not nan
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="SetupObjectPose", text="Setting pose for RTT"
        )
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
    rospy.init_node("rtt_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )

    with sm:
        smach.StateMachine.add(
            "GET_PREDICTIONS",
            GetPredictions("/mir_perception/rtt/event_in"),
            transitions={"succeeded": "MOVE_ARM_PRE_PICK"},
        )

        # move arm to appropriate position
        # TODO: change rtt_pre_pick to variable position 5 cm above current pose received from perception
        smach.StateMachine.add(
            "MOVE_ARM_PRE_PICK",
            gms.move_arm("rtt_pre_pick", use_moveit=False),
            transitions={
                "succeeded": "SETUP_OBJECT_POSE",
                "failed": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SETUP_OBJECT_POSE",
            SetupObjectPose(),
            transitions={"succeeded": "WAIT_FOR_OBJECT",
                          "failed": "TRY_PICKING"},
        )

        smach.StateMachine.add(
            "WAIT_FOR_OBJECT",
            WaitForObject(),
            transitions={
                "succeeded": "TRY_PICKING",
                "failed": "OVERALL_FAILED",
            },
        )

        # move only arm for wbc
        smach.StateMachine.add(
            "TRY_PICKING",
            gms.move_arm(),
            transitions={
                "succeeded": "CLOSE_GRIPPER",
                "failed": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "CLOSE_GRIPPER",
            gms.control_gripper("close"),
            transitions={"succeeded": "MOVE_ARM_TO_STAGE_INTERMEDIATE",
                         "timeout": "MOVE_ARM_TO_STAGE_INTERMEDIATE"},
        )

        # smach.StateMachine.add(
        #     "MOVE_ROBOT_AND_PICK",
        #     gbs.send_and_wait_events_combined(
        #         event_in_list=[("/wbc/event_in", "e_start")],
        #         event_out_list=[("/wbc/event_out", "e_success", True)],
        #         timeout_duration=50,
        #     ),
        #     transitions={
        #         "success": "CLOSE_GRIPPER",
        #         "timeout": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
        #         "failure": "STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE",
        #     },
        # )

        # move arm to stage_intemediate position
        smach.StateMachine.add(
            "MOVE_ARM_TO_STAGE_INTERMEDIATE",
            gms.move_arm("pre_place"),
            transitions={
                "succeeded": "VERIFY_OBJECT_GRASPED",
                "failed": "MOVE_ARM_TO_STAGE_INTERMEDIATE",
            },
        )

        smach.StateMachine.add(
            "VERIFY_OBJECT_GRASPED",
            gms.verify_object_grasped(5),
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "timeout": "OVERALL_FAILED",
                "failed": "OVERALL_FAILED",
            },
        )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)


    # smach viewer
    if rospy.get_param("~viewer_enabled", True):
        sis = IntrospectionServer(
            "rtt_pick_object_smach_viewer", sm, "/RTT_PICK_OBJECT_SMACH_VIEWER"
        )
        sis.start()

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
