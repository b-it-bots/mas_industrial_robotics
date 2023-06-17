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
from mir_object_recognition.msg import TimeStampedPose

# ===============================================================================

class GetPredictions(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(
            self,
            outcomes=["succeeded"],
            input_keys=["goal"],
            output_keys=["feedback", "result", "pose"],
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
        # TODO: if multiple times are there, figure out what to do
        # if pred_time - current_time < 2.1 return succeeded
        # else return failed
        while not pred_time - rospy.Time.now().to_sec() < 2.1:
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
        userdata.move_arm_to = userdata.pose
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
    rospy.init_node("rtt_pick_object_wbc_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )

    with sm:
        smach.StateMachine.add(
            "GET_PREDICTIONS",
            GetPredictions("mir_perception/rtt/event_in"),
            transitions={"succeeded": "MOVE_ARM_PRE_PICK"},
        )

        # move arm to appropriate position
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
            transitions={"succeeded": "TRY_PICKING"},
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
            transitions={"succeeded": "MOVE_ROBOT_AND_PICK"},
        )

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
                "failed": "OVERALL_FAILED",
            },
        )

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
