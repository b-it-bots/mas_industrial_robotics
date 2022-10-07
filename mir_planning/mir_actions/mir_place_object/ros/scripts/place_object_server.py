#!/usr/bin/env python


# threshold checking for TF calculation WIP



from selectors import PollSelector
import mcr_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms  # move the arm, and gripper
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


# ===============================================================================
class GetPoseToPlaceOject(smach.State):  # inherit from the State base class
    """
    Not being used anymore.
    GetEmptyPositionOnTable is used instead.
    """
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
        self.empty_location = None

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


# ===============================================================================

# new class for threshold calculation to go to default place
class Threshold_calculation(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["reached", "continue"],
            input_keys=["max_allowed_tries", "threshold_counter"],
            output_keys=["feedback", "result", "threshold_counter"],
        )

    def execute(self, userdata):

        max_tries = userdata.max_allowed_tries  

        result = None

        if userdata.threshold_counter >= max_tries:
            result = "reached"
            userdata.threshold_counter = 0
        else:
            userdata.threshold_counter += 1
            result =  "continue"

        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="GO_DEFAULT_THRESHOLD", text="No of time tried the IK reachability: " + str(userdata.threshold_counter),
        )
        return result
        
# ==============================================================================

# new class for empty space detection

class GetEmptyPositionOnTable(smach.State):
    def __init__(self,empty_spaces):
        
        smach.State.__init__(self,outcomes=["success", "failure"],
		input_keys=["empty_locations",],
		output_keys=["feedback","result","empty_locations"]
        )
        
        self.empty_locations = None
        self.timeout = rospy.Duration.from_sec(15.0)
        rospy.Subscriber(empty_spaces, PoseArray, self.empty_space_cb) # subscribing to empty_space locations published by empty_space_detector node
        rospy.sleep(0.1)

    def empty_space_cb(self, msg): # Callback for get empty_space_location from empty_space detector node 
        
        self.empty_locations = msg
        
    def execute(self, userdata): # Getting data from empty space detector and giving all the poses it to next states 
        
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
        current_state="EMPTY_POSE_RECEIVE", text="Receiving empty space location",)
        rospy.loginfo("<<--- getting empty position --->>")
        userdata.empty_locations = self.empty_locations # The empty locations have all the empty poses
        
        return 'success'

# ===============================================================================

# new class for publishing the empty pose

class PublishObjectPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failed"],
                                    input_keys=["empty_locations"])

        self.empty_pose_pub = rospy.Publisher(
            "/mcr_perception/object_selector/output/object_pose",
            PoseStamped,
            queue_size=10)

    def execute(self, userdata):

        empty_locations = userdata.empty_locations # This have all the empty poses
	    # converting the pose into desired format
        # nearest_pose = PoseStamped()
        # nearest_pose.header = empty_locations.header
        # nearest_pose.pose = empty_locations.poses[0]
        # prioritise the empty space location closer to the base link 

        if len(empty_locations.poses) > 0:

            base_link_pose = PoseStamped()
            base_link_pose.header = empty_locations.header
            
            base_link_pose.pose.position.x = 0.0
            base_link_pose.pose.position.y = 0.0
            base_link_pose.pose.position.z = 0.0
            base_link_pose.pose.orientation.x = 0.0
            base_link_pose.pose.orientation.y = 0.0
            base_link_pose.pose.orientation.z = 0.0

            empty_locations_temp = PoseStamped()
            empty_locations_temp.header = empty_locations.header
            

            distance_list = []
            for i in range(len(empty_locations.poses)):
                empty_locations_temp.pose = empty_locations.poses[i]
                distance_list.append(Utils.get_distance_between_poses(base_link_pose, empty_locations_temp))
                print("count", i)
            
            # return the index of min value in the list
            index = distance_list.index(min(distance_list))

            nearest_pose = PoseStamped()
            nearest_pose.header = empty_locations.header
            nearest_pose.pose = empty_locations.poses[index]

        nearest_pose.pose.position.z = 0.1 + 0.05 - 0.0815 # setting the z value to 0.0
        # nearest_pose.pose.position.z += rospy.get_param("object_height_above_workspace", 0.035) # adding the height of the object above the workspace Should be change for vertical object
        # nearest_pose.pose.orientation.x = 0
        # nearest_pose.pose.orientation.y = 0
        # nearest_pose.pose.orientation.z = 0
        
        rospy.loginfo(nearest_pose.pose.position.z)
        rospy.loginfo("Publishing single pose to pregrasp planner")

        rospy.loginfo(type(nearest_pose))
        self.empty_pose_pub.publish(nearest_pose)

        rospy.sleep(0.3)
        nearest_pose = None

        return "success"

class SetupMoveArm(smach.State):
    """

This class is added to set the arm at the location of platform for holding the object.
This behaviour is only needed when handling single object

"""
    def __init__(self, arm_target, is_heavy=False):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["goal"],
            output_keys=["feedback", "result", "move_arm_to"],
        )
        self.arm_target = arm_target
        self.is_heavy = is_heavy

    def execute(self, userdata):
        platform = Utils.get_value_of(userdata.goal.parameters, "platform")
        if platform is None:
            rospy.logwarn('Missing parameter "platform". Using default.')
            platform = "PLATFORM_MIDDLE"
        platform = platform.lower()

        if self.arm_target == "pre":
            platform += "_pre"

        if self.is_heavy:
            platform += "_heavy"
        userdata.move_arm_to = platform

        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="SetupMoveArm", text="Moving arm to " + platform
        )
        return "succeeded"


#=================================================================================

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

    sm.userdata.threshold_counter = 0
    sm.userdata.empty_locations = None
    sm.userdata.heavy_objects = rospy.get_param("~heavy_objects", ["m20_100"])
    sm.userdata.max_allowed_tries = rospy.get_param("~max_allowed_IK_tries", 3)

    with sm:
        # add states to the container
        smach.StateMachine.add(
            "CHECK_IF_SHELF_INITIAL",
            CheckIfLocationIsShelf(),
            transitions={
                "shelf": "MOVE_ARM_TO_SHELF_INTERMEDIATE",
                "not_shelf": "CHECK_MAX_TRY_THRESHOLD",  # change it to MOVE_ARM_TO_PRE_PLACE when we use arm camera for empty space detection
            },
        )

# Comment the below state when we use arm camera for empty space detection
 
#=================================================================================

        # """
        # These states are added to keep the arm at the state of holding the object.
        # This behaviour is only needed when handling single object
        # """
        # smach.StateMachine.add(
        #     "MOVE_ARM_STAGE",
        #     gms.move_arm("platform_middle"),
        #     transitions={
        #         "succeeded": "EMPTY_SPACE_CLOUD_ADD",
        #         "failed": "MOVE_ARM_STAGE"
        #     },
        # )
#=================================================================================

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE",
            gms.move_arm("shelf_intermediate"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_2",
            gms.move_arm("shelf_intermediate_2"),
            transitions={
                "succeeded": "MOVE_ARM_TO_PRE_GRASP_LOWER",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_GRASP_LOWER",
            gms.move_arm("shelf_pre_grasp_lower"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_PLACE_FINAL",
                "failed": "MOVE_ARM_TO_PRE_GRASP_LOWER",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_PLACE_FINAL",
            gms.move_arm("shelf_place_final"),
            transitions={
                "succeeded": "OPEN_GRIPPER_SHELF",
                "failed": "MOVE_ARM_TO_SHELF_PLACE_FINAL",
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_SHELF",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT"},
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT",
            gms.move_arm("shelf_place_final"),
            transitions={
                "succeeded": "MOVE_ARM_TO_PRE_GRASP_LOWER_RETRACT",
                "failed": "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_GRASP_LOWER_RETRACT",
            gms.move_arm("shelf_pre_grasp_lower"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2_RETRACT",
                "failed": "MOVE_ARM_TO_PRE_GRASP_LOWER_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_2_RETRACT",
            gms.move_arm("shelf_intermediate_2"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            gms.move_arm("shelf_intermediate"),
            transitions={
                    "succeeded": "MOVE_ARM_TO_NEUTRAL",
                    "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            },
        )
# till above the state machine is for shelf

## Uncomment the below code when we use arm camera for empty space detection

        # smach.StateMachine.add(
        #     "MOVE_ARM_TO_PRE_PLACE",
        #     gms.move_arm("look_at_workspace"), # New change from turntable to workspace
        #     transitions={
        #         "succeeded": "EMPTY_SPACE_CLOUD_ADD",
        #         "failed": "MOVE_ARM_TO_PRE_PLACE",
        #     },
        # )
        

        smach.StateMachine.add(
            "CHECK_MAX_TRY_THRESHOLD",
            Threshold_calculation(),
            transitions={
                "continue": "EMPTY_SPACE_CLOUD_ADD",
                "reached": "GO_DEFAULT_THRESHOLD",
            },
            # remapping={
            #     "threshold_counter_in": "threshold_counter",
            #     "threshold_counter_out": "threshold_counter",
            # },
        )


        smach.StateMachine.add(
            "GO_DEFAULT_THRESHOLD",
            gms.move_arm("look_at_workspace"), # change it to default place later
            transitions={
                "succeeded": "OVERALL_SUCCESS",
                "failed": "GO_DEFAULT_THRESHOLD",
            }
        )

        smach.StateMachine.add(
            "EMPTY_SPACE_CLOUD_ADD",
            gbs.send_and_wait_events_combined(
                event_in_list = [
                    ("/mir_perception/empty_space_detector/event_in","e_add_cloud"),
                                ],
                event_out_list = [("/mir_perception/empty_space_detector/event_out","e_added_cloud", True)],
                timeout_duration=50,),
            transitions={"success": "EMPTY_SPACE_TRIGGER",
                        "timeout": "CHECK_MAX_TRY_THRESHOLD",
                        "failure": "EMPTY_SPACE_CLOUD_ADD",},
        )


        smach.StateMachine.add(
            "EMPTY_SPACE_TRIGGER",
            gbs.send_and_wait_events_combined(
                event_in_list = [("/mir_perception/empty_space_detector/event_in","e_trigger")],
		        event_out_list = [("/mir_perception/empty_space_detector/event_out","e_success",True)],
		        timeout_duration = 50,),
            transitions={"success": "EMPTY_POSE_RECEIVE",
                        "timeout": "EMPTY_SPACE_CLOUD_ADD",
                        "failure": "CHECK_MAX_TRY_THRESHOLD",},
        )


        smach.StateMachine.add(
		"EMPTY_POSE_RECEIVE",
                GetEmptyPositionOnTable(
			"/mir_perception/empty_space_detector/empty_spaces"),

        transitions={
                "success": "PUBLISH_OBJECT_POSE_FOR_VERIFICATION",
                "failure": "EMPTY_SPACE_CLOUD_ADD",
	   },
	)

        smach.StateMachine.add(
            "PUBLISH_OBJECT_POSE_FOR_VERIFICATION",
            PublishObjectPose(),
            transitions={"success": "CHECK_PRE_GRASP_POSE_IK", 
                         "failed": "PUBLISH_OBJECT_POSE_FOR_VERIFICATION"}

        )
        
        smach.StateMachine.add(
            "CHECK_PRE_GRASP_POSE_IK",
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
                "success": "GO_TO_PRE_GRASP_POSE",
                "timeout": "CHECK_MAX_TRY_THRESHOLD", 
                "failure": "CHECK_MAX_TRY_THRESHOLD",
            },
        )

        smach.StateMachine.add(
            "GO_TO_PRE_GRASP_POSE",
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
                "success": "OPEN_GRIPPER", 
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )


        smach.StateMachine.add(
                "OPEN_GRIPPER",
                gms.control_gripper("open"),
                transitions={
			        "succeeded": "MOVE_ARM_TO_NEUTRAL"},
                )


        smach.StateMachine.add(
                "MOVE_ARM_TO_NEUTRAL",
                gms.move_arm("barrier_tape"),
                transitions={
                    "succeeded": "OVERALL_SUCCESS",
                    "failed": "MOVE_ARM_TO_NEUTRAL",
                    },
                )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)
    sm.userdata.threshold_counter = 0

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
