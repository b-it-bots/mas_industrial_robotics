#!/usr/bin/env python


# threshold checking for TF calculation WIP


"""
This file is for eye to hand configuration of the robot
unstage will happen outside the place object server

"""


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
            input_keys=["max_allowed_tries", "threshold_counter", "counter_reset_flag"],
            output_keys=["feedback", "result", "threshold_counter"],
        )

    def execute(self, userdata):
    

        max_tries = userdata.max_allowed_tries  

        print("userdata.threshold_counter", userdata.threshold_counter)

        result = None
        if userdata.counter_reset_flag:
            userdata.threshold_counter = 0

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

# new class for publishing the empty pose.


class DefalutSafePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"],
                                    input_keys=["goal"],)

        self.empty_pose_pub = rospy.Publisher(
            "/mcr_perception/object_selector/output/object_pose",
            PoseStamped,
            queue_size=10,
        )

    def map_location_to_base_link(self, location):

        if location is not None:
            current_platform_height = rospy.get_param("/"+location)
            current_platform_height = current_platform_height /100
        else:
            current_platform_height = 0.10

        map_location_to_platform_height = {
            0.15: 0.065,
            0.10: 0.025,
            0.05: -0.030,
            0.0: -0.080
        }

        return map_location_to_platform_height[current_platform_height]


    def execute(self, userdata):

        location = Utils.get_value_of(userdata.goal.parameters, "location")
        
        rospy.logerr("-->> No empty space found <<--")
        rospy.logwarn("Checking pre-defined safe pose")

        height_from_base = self.map_location_to_base_link(location)

        safe_pose = PoseStamped()
        safe_pose.header.frame_id = "base_link"
        safe_pose.pose.position.x = 0.610
        safe_pose.pose.position.y = 0.095
        safe_pose.pose.position.z = height_from_base + rospy.get_param("/mir_perception/empty_space_detector/object_height_above_workspace") # adding the height of the object above the workspace Should be change for vertical object
        
        self.empty_pose_pub.publish(safe_pose)
        rospy.sleep(0.1)
        safe_pose = None
        return "succeeded"



class PublishObjectPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failed"],
                                    input_keys=["goal","empty_locations", "counter_reset_flag"],
                                    output_keys=["counter_reset_flag"])

        self.empty_pose_pub = rospy.Publisher(
            "/mcr_perception/object_selector/output/object_pose",
            PoseStamped,
            queue_size=10)
    def map_location_to_base_link(self, location):

        if location is not None:
            current_platform_height = rospy.get_param("/"+location)
            current_platform_height = current_platform_height /100
        else:
            current_platform_height = 0.10

        map_location_to_platform_height = {
            0.15: 0.065,
            0.10: 0.025,
            0.05: -0.028,
            0.0: -0.08
        }

        return map_location_to_platform_height[current_platform_height]

    def execute(self, userdata):

        empty_locations = userdata.empty_locations 
        # reset the max threshold counter
        userdata.counter_reset_flag = True
        
        location = Utils.get_value_of(userdata.goal.parameters, "location")
        

        if len(empty_locations.poses) > 0:

            base_link_pose = PoseStamped()
            base_link_pose.header = empty_locations.header

            empty_locations_temp = PoseStamped()
            empty_locations_temp.header = empty_locations.header
            

            distance_list = []
            for i in range(len(empty_locations.poses)):
                empty_locations_temp.pose = empty_locations.poses[i]
                distance_list.append(Utils.get_distance_between_poses(base_link_pose, empty_locations_temp))
            
            index = distance_list.index(min(distance_list))

            nearest_pose = PoseStamped()
            nearest_pose.header = empty_locations.header
            nearest_pose.pose = empty_locations.poses[index]
        
            print("----------------------------------")
            print("nearest pose", nearest_pose.pose.position.z)

            height_from_base = self.map_location_to_base_link(location)
            if height_from_base > nearest_pose.pose.position.z:
                nearest_pose.pose.position.z = height_from_base
            nearest_pose.pose.position.z += rospy.get_param("/mir_perception/empty_space_detector/object_height_above_workspace") # adding the height of the object above the workspace Should be change for vertical object

            print("nearest pose after adding height UPDATED !!!")
            print(nearest_pose.pose.position.z)
            print("Publishing single pose to pregrasp planner")
            print("----------------------------------")

            self.empty_pose_pub.publish(nearest_pose)

            rospy.sleep(0.1)
            nearest_pose = None
            return "success"
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

    sm.userdata.counter_reset_flag = False
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
                "not_shelf": "CHECK_MAX_TRY_THRESHOLD", 
            },
        )

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
            transitions={"succeeded": "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT",
                         "timeout": "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT"}
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

        smach.StateMachine.add(
            "CHECK_MAX_TRY_THRESHOLD",
            Threshold_calculation(),
            transitions={
                "continue": "EMPTY_SPACE_CLOUD_ADD",
                "reached": "GO_SAFE_POSE",
            },
        )

        smach.StateMachine.add(
            "GO_SAFE_POSE",
            DefalutSafePose(),
            transitions={
                "succeeded": "CHECK_PRE_GRASP_POSE_IK",
                "failed": "GO_DEFAULT_THRESHOLD",
            },
        )


        smach.StateMachine.add(
            "GO_DEFAULT_THRESHOLD",
            gms.move_arm("place_default", use_moveit=False),
            transitions={
                "succeeded": "OPEN_GRIPPER",
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
                timeout_duration=50,
            ),
            transitions={
                "success": "EMPTY_SPACE_TRIGGER",
                "timeout": "CHECK_MAX_TRY_THRESHOLD",
                "failure": "EMPTY_SPACE_CLOUD_ADD",
            },
        )


        smach.StateMachine.add(
            "EMPTY_SPACE_TRIGGER",
            gbs.send_and_wait_events_combined(
                event_in_list = [("/mir_perception/empty_space_detector/event_in","e_trigger")],
		        event_out_list = [("/mir_perception/empty_space_detector/event_out","e_success",True)],
		        timeout_duration = 50,
            ),
            transitions={
                "success": "EMPTY_POSE_RECEIVE",
                "timeout": "EMPTY_SPACE_CLOUD_ADD",
                "failure": "CHECK_MAX_TRY_THRESHOLD",
            },
        )


        smach.StateMachine.add(
		    "EMPTY_POSE_RECEIVE",
            GetEmptyPositionOnTable("/mir_perception/empty_space_detector/empty_spaces"),

        transitions={
                "success": "PUBLISH_OBJECT_POSE_FOR_VERIFICATION",
                "failure": "EMPTY_SPACE_CLOUD_ADD",
            },
	    )

        smach.StateMachine.add(
            "PUBLISH_OBJECT_POSE_FOR_VERIFICATION",
            PublishObjectPose(),
            transitions={
                "success": "CHECK_PRE_GRASP_POSE_IK", 
                "failed": "PUBLISH_OBJECT_POSE_FOR_VERIFICATION"
            }

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
			        "succeeded": "MOVE_ARM_TO_NEUTRAL",
                                 "timeout": "MOVE_ARM_TO_NEUTRAL"}
        )


        smach.StateMachine.add(
                "MOVE_ARM_TO_NEUTRAL",
                gms.move_arm("barrier_tape", use_moveit=False),
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
