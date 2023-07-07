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

class MoveArmUp(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "timeout"],
        )
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_cb)
        self.pub_arm_position = rospy.Publisher("/arm_1/arm_controller/position_command", JointPositions, queue_size=1)
        self.current_joint_positions = None
        self.is_arm_moving = False
        self.zero_vel_counter = 0
        self.joint_1_position = 1.8787

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
        joint_values[1] = self.joint_1_position
        
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
        return "succeeded"





# ===============================================================================

class DefineShelfPlacePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'failed'],
                             output_keys=['move_arm_to'])
        self.pose_list = ["shelf_place_1", "shelf_place_2"]

    def execute(self, userdata):
        try:
            if len(self.pose_list) > 0:
                rospy.logwarn("Getting shelf place pose from list")
                userdata.move_arm_to = self.pose_list.pop()
            else:
                rospy.logfatal("No more shelf place pose in list, so using default pose")
                userdata.move_arm_to = "shelf_place_final"
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

# # new class for threshold calculation to go to default place
# class Threshold_calculation(smach.State):

#     def __init__(self):
#         smach.State.__init__(
#             self,
#             outcomes=["reached", "continue"],
#             input_keys=["max_allowed_tries", "threshold_counter", "counter_reset_flag"],
#             output_keys=["feedback", "result", "threshold_counter"],
#         )

#     def execute(self, userdata):
    
#         max_tries = userdata.max_allowed_tries  

#         print("userdata.threshold_counter", userdata.threshold_counter)

#         result = None
#         if userdata.counter_reset_flag:
#             userdata.threshold_counter = 0

#         if userdata.threshold_counter >= max_tries:
#             result = "reached"
#             userdata.threshold_counter = 0
#         else:
#             userdata.threshold_counter += 1
#             result =  "continue"

#         userdata.result = GenericExecuteResult()
#         userdata.feedback = GenericExecuteFeedback(
#             current_state="GO_DEFAULT_THRESHOLD", text="No of time tried the IK reachability: " + str(userdata.threshold_counter),
#         )
#         return result
        
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

        self.floor_height = rospy.get_param("height_of_floor", -0.083)

    def map_location_to_base_link(self, location):

        if location is not None:
            current_platform_height = rospy.get_param("/"+location)
            current_platform_height = current_platform_height /100
        else:
            current_platform_height = 0.10

        platform_height = self.floor_height + current_platform_height

        return platform_height

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
            
            index = distance_list.index(max(distance_list))

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
                "not_shelf": "CHECK_MODE_OF_PLACING", 
            },
        )

        smach.StateMachine.add(
            "CHECK_MODE_OF_PLACING",
            CheckModePlacing(),
            transitions={
                "pose_selector": "START_PLACE_POSE_SELECTOR",
                "empty_pose": "CHECK_MAX_TRY_THRESHOLD",
            }
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE",
            gms.move_arm("shelf_intermediate"),
            transitions={
                "succeeded": "SET_SHELF_PLACE_POSE",
                "failed": "OVERALL_FAILED",
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
            transitions={"succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
                         "timeout": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT"}
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            gms.move_arm("shelf_intermediate"),
            transitions={
                    "succeeded": "MOVE_ARM_TO_NEUTRAL",
                    "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            },
        )

# below states are for default pose placing

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
                15.0,
            ),
            transitions={
                "succeeded": "MOVE_ARM_TO_PLACE_OBJECT",
                "failed": "MOVE_ARM_TO_DEFAULT_PLACE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_DEFAULT_PLACE",
            gms.move_arm("10cm/pose4"),
            transitions={
                "succeeded": "STOP_PLACE_POSE_SELECTOR",
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
            transitions={"success": "OPEN_GRIPPER"},
        )


# below states for empty space placing--

        # smach.StateMachine.add(
        #     "CHECK_MAX_TRY_THRESHOLD",
        #     Threshold_calculation(),
        #     transitions={
        #         "continue": "EMPTY_SPACE_CLOUD_ADD",
        #         "reached": "GO_SAFE_POSE",
        #     },
        # )

                # retry if failed
        smach.StateMachine.add(
            "CHECK_MAX_TRY_THRESHOLD",
            CheckRetries(),
            transitions={
                "retry": "EMPTY_SPACE_CLOUD_ADD",
                "no_retry": "START_PLACE_POSE_SELECTOR",
            },
        )

        # smach.StateMachine.add(
        #     "GO_SAFE_POSE",
        #     DefalutSafePose(),
        #     transitions={
        #         "succeeded": "CHECK_PRE_GRASP_POSE_IK",
        #         "failed": "GO_DEFAULT_THRESHOLD",
        #     },
        # )


        # smach.StateMachine.add(
        #     "GO_DEFAULT_THRESHOLD",
        #     gms.move_arm("place_default", use_moveit=False),
        #     transitions={
        #         "succeeded": "OPEN_GRIPPER",
        #         "failed": "GO_DEFAULT_THRESHOLD",
        #     }
        # )

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
                gms.control_gripper(0.25),
                transitions={
			        "succeeded": "MOVE_ARM_UP",
                                 "timeout": "MOVE_ARM_UP"}
        )

        smach.StateMachine.add(
                "MOVE_ARM_UP",
                MoveArmUp(),
                transitions={
			        "succeeded": "MOVE_ARM_TO_NEUTRAL",
                                 "timeout": "MOVE_ARM_TO_NEUTRAL"}
        )

        smach.StateMachine.add(
                "MOVE_ARM_TO_NEUTRAL",
                gms.move_arm("pre_place", use_moveit=False),
                transitions={
                    "succeeded": "OVERALL_SUCCESS",
                    "failed": "MOVE_ARM_TO_NEUTRAL",
            },
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
