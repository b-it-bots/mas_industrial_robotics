#!/usr/bin/python
import mcr_perception_states.common.perception_states as gps
import mcr_states.common.basic_states as gbs
import mir_states.common.action_states as gas
import mir_states.common.manipulation_states as gms
import moveit_commander
import numpy as np
import rospy
import smach
import tf.transformations
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, TwistStamped
from mas_perception_msgs.msg import Cavity
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
    GenericExecuteGoal,
)
from moveit_msgs.msg import MoveItErrorCodes
from smach_ros import ActionServerWrapper, IntrospectionServer
from std_msgs.msg import Float64, String
from diagnostic_msgs.msg import KeyValue
from sensor_msgs.msg import JointState


class SelectCavity(smach.State):
    def __init__(self, topic_name, vertical=False):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )
        self.vertical = vertical
        self.publisher = rospy.Publisher(topic_name, String, queue_size=10)
        rospy.sleep(0.1)  # time for the publisher to register in ros network

    def obj_to_cavity(self, obj_name: str, vertical: bool = False):
        matchings = {
            "M20": ["M20_H", "M20_V"],
            "M30": ["M30_H", "M30_V"],
            "M20_100": ["M20_100_H", "M20_100_V"],
            "F20_20_B": ["F20_20_H", "F20_20_V"],
            "F20_20_G": ["F20_20_H", "F20_20_V"],
            "S40_40_B": ["S40_40_H", "S40_40_V"],
            "S40_40_G": ["S40_40_H", "S40_40_V"],
        }
        if obj_name is not None and obj_name != "":
            obj_name = obj_name.split("-")[0]
            for key in matchings.keys():
                if obj_name == key:
                    return matchings[key][vertical]
        return None


    def execute(self, userdata):
        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="SelectObject", text="selecting object"
        )

        obj = Utils.get_value_of(userdata.goal.parameters, "peg")

        if obj is None:
            rospy.logerr("No object name provided")
            return "failed"

        # get the cavity name
        cavity_name = self.obj_to_cavity(obj, self.vertical)

        if cavity_name is None:
            rospy.logerr("No cavity name found for object: " + obj)
            return "failed"

        self.publisher.publish(String(data=cavity_name))
        rospy.sleep(0.2)  # let the topic survive for some time
        return "succeeded"

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
        rospy.sleep(0.55)
        # stop the arm
        vel_msg.twist.linear.z = 0
        self.arm_velocity_pub.publish(vel_msg)
        return "succeeded"

class ppt_wiggle_arm(smach.State):
    """
    Wiggle arm after placement on the table
    """

    def __init__(self, wiggle_yaw=1.57):
        smach.State.__init__(self, 
                             input_keys=["goal"], 
                             outcomes=["succeeded", "failed"])

        self.blocking = True

        # create publisher for sending gripper position
        self.gripper_topic = "/gripper_controller/command"
        # self.cavity_pose_topic = "/mcr_perception/object_selector/output/object_pose"
        self.cavity_pose_topic = "/mir_perception/multimodel_object_recognition/output/rgb_object_pose_array"
        self.arm_velocity_pub = rospy.Publisher("/arm_1/arm_controller/cartesian_velocity_command", TwistStamped, queue_size=10)
    
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_cb)

        self.gripper = rospy.Publisher(self.gripper_topic, Float64, queue_size=10)

        # Determineing object orientation
        rospy.Subscriber(self.cavity_pose_topic, PoseStamped, self.cavity_pose_cb)

        # MoveIt! commander movegroup
        self.arm_command = moveit_commander.MoveGroupCommander("arm_1")
        self.arm_command.set_goal_position_tolerance(0.01)
        self.arm_command.set_goal_orientation_tolerance(0.01)
        self.arm_command.set_goal_joint_tolerance(0.005)

        self.object_name = None
        self.type_of_adjustment = 'rotational'

        self.linear_wiggle_object_list = ["M20", "M30"]
        self.rotational_wiggle_object_list = ["M20_100", "F20_20_B","F20_20_G", "S40_40_B", "S40_40_G"]


        self.joint_offset_tolerance = 0.02 # radians
        self.horizontal_wrist_angle = 2.98 # radians
        self.vertical_wrist_angle = 4.53 # radians

        self.yaw = 0
        self.wiggle_yaw = wiggle_yaw # wiggle angle in radians (default 1.57) on either direction
        self.joint_values_static = []
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)


    def joint_states_cb(self, msg):
        if "arm_joint_1" in msg.name: # get the joint values of the arm only
            self.current_joint_positions = msg.position

    def object_name_cb(self, msg):
        """
        Obtains an event for the component.

        """
        rospy.loginfo("Object received: {0}".format(msg.data))
        self.object_name = msg

    def cavity_pose_cb(self, msg):
        self.object_pose = msg
        o = msg.pose.orientation
        l = [o.x, o.y, o.z, o.w]
        roll, pitch, self.yaw = tf.transformations.euler_from_quaternion(l)

    def execute_arm(self, joint_number, wiggle_offset):
        joint_values = self.joint_values_static[:]
        joint_values[joint_number] = (
            self.joint_values_static[joint_number] + wiggle_offset
        )
        try:
            self.arm_command.set_joint_value_target(joint_values)
        except Exception as e:
            rospy.logerr("unable to set target position: %s" % (str(e)))
            return False
        error_code = self.arm_command.go(wait=True)

        if error_code == MoveItErrorCodes.SUCCESS:
            return True
        else:
            rospy.logerr("Arm movement failed with error code: %d", error_code)
            return False

    def stop_arm(self):
        message = TwistStamped()
        message.header.stamp = rospy.Time.now()
        self.arm_velocity_pub.publish(message)
        rospy.sleep(0.1)
        return True

    def rotational_wiggle(self, direction, message, wiggle_velocity=5.0, timeout=15.0, wiggle_yaw=1.57):

        """
        WORKING DONT TOUCH !!!
        """

        while not rospy.is_shutdown():

            if direction == "CW":
                rospy.loginfo("Wiggling CW")
                rospy.loginfo("Wiggle yaw: %f", wiggle_yaw)
                message.twist.angular.z = wiggle_velocity
            elif direction == "CCW":
                wiggle_yaw = 2 * wiggle_yaw
                rospy.loginfo("Wiggling CCW")
                rospy.loginfo("Wiggle yaw: %f", wiggle_yaw)
                message.twist.angular.z = -wiggle_velocity

            if self.current_joint_positions is None or len(self.current_joint_positions) < 1:
                rospy.logwarn("No joint positions received")
                continue # skip the rest of the loop 

            # Getting Yaw angle from joint_states   
            initial_yaw = self.current_joint_positions[-1] # in radians take the last joint value(EFF)
            initial_time = rospy.Time.now().to_sec()

            while not rospy.is_shutdown():
                current_time = rospy.Time.now().to_sec()
                self.arm_velocity_pub.publish(message)
                rospy.sleep(0.1)
                print("wiggle yaw: ", wiggle_yaw)
                current_yaw = self.current_joint_positions[-1]
                print(np.rad2deg(abs(current_yaw - initial_yaw)))
                # check for timeout also and break after 5 seconds
                # limits: 5.58 and 0.16
                if (current_yaw >= 5.4) or (current_yaw <= 0.28):
                    rospy.logwarn("Reached the joint limit")
                    self.stop_arm()
                    return True
                if abs(current_yaw - initial_yaw) >= wiggle_yaw:
                    rospy.loginfo("Reached the desired yaw angle")
                    self.stop_arm()
                    return True
                if current_time - initial_time >= timeout:
                    rospy.logwarn("Reached the timeout")
                    self.stop_arm()
                    return False 

    def linear_wiggle_cartesian_mode(self, movement_type, travel_direction, message, travel_distance= 0.1, travel_velocity=0.03, timeout=5.0):
        """
        Because of joint limit linear velocity movement is not possible do properlly
        
        # how to use
        self.linear_wiggle_cartesian_mode("horizontal", "backward", message, travel_distance= 0.05, travel_velocity=0.03)
        """

        rospy.logwarn("moving obj %s %s" % (movement_type, travel_direction))

        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "arm_link_5"
        while not rospy.is_shutdown():
            if travel_direction == "forward": # change the direction of travel
                travel_velocity = travel_velocity
            elif travel_direction == "backward":
                travel_velocity = -travel_velocity
            else:
                rospy.logerr("Invalid direction")
                travel_velocity = 0.0

            travel_time = travel_distance / abs(travel_velocity)
            initial_time = rospy.Time.now().to_sec()
            while not rospy.is_shutdown():
                if movement_type == "horizontal":
                    message.twist.linear.x = 0.0
                    message.twist.linear.y = travel_velocity
                elif movement_type == "vertical":
                    message.twist.linear.x = travel_velocity
                    message.twist.linear.y = 0.0
                else:
                    rospy.logerr("Invalid movement type")
                    message.twist.linear.x = 0.0
                    message.twist.linear.y = 0.0

                self.arm_velocity_pub.publish(message)
                current_time = rospy.Time.now().to_sec()
                rospy.sleep(0.05)
                print("travel time: ", travel_time)
                print(abs(current_time - initial_time))
                if abs(current_time - initial_time) >= travel_time: # no feedback from the arm just wait for the travel time
                    rospy.loginfo("Travel time reached")
                    self.stop_arm()
                    positive_flag = True
                    return True

        # get current pose of the arm and add offset and move to that position, 

    def linear_wiggle_joint_mode(self, travel_direction, travel_distance= 0.08): # travel distance in radians
    
        # for vertical motion move arm link 4 and for horizontal motion move arm link 0

        rospy.logwarn("Adjusting the object with linear wiggle joint mode in %s direction", travel_direction)

        joint_number_to_change = None
        wiggle_offset = travel_distance

        if travel_distance > 1.5: # Since it is joint angles should be careful with the travel distance 
            rospy.logerr("Travel distance is too high")
            return "failed"

        self.joint_values_static = self.arm_command.get_current_joint_values()

        if travel_direction == "horizontal":
            # check if the joint values for joint 4 is around 2.98 +- 0.1 if not set it to 2.98
            if self.joint_values_static[4] < self.horizontal_wrist_angle - self.joint_offset_tolerance or self.joint_values_static[4] > self.horizontal_wrist_angle + self.joint_offset_tolerance:
                self.joint_values_static[4] = self.horizontal_wrist_angle # joint value for arm link 4 when wrist is horizontal

        elif travel_direction == "vertical":
            # check if the joint values for joint 4 is around 4.01 +- 0.1 if not set it to 4.01
            if self.joint_values_static[4] < self.vertical_wrist_angle - self.joint_offset_tolerance or self.joint_values_static[4] > self.vertical_wrist_angle + self.joint_offset_tolerance:
                self.joint_values_static[4] = self.vertical_wrist_angle # joint value for arm link 4 when wrist is vertical
        else:
            rospy.logerr("[cavity server] Invalid travel direction")
            return "failed"


        try:
            self.arm_command.set_joint_value_target(self.joint_values_static)
        except Exception as e:
            rospy.logerr("unable to set target position: %s" % (str(e)))
            return "failed"
        error_code = self.arm_command.go(wait=True)
        if not error_code == MoveItErrorCodes.SUCCESS:
            return "failed"

        success = self.execute_arm(joint_number_to_change , wiggle_offset= wiggle_offset)
        success &= self.execute_arm(joint_number_to_change , wiggle_offset= -wiggle_offset)
        success &= self.execute_arm(joint_number_to_change , wiggle_offset= 0.0)

        if success:
            return "succeeded"
        else:
            return "failed"

    def rotate_wrist_joint(self, direction):

        """
        input:
            direction: "horizontal" or "vertical" 
        Sets the wrist joint to horizontal or vertical position
        """
        # for vertical motion move arm link 4 and for horizontal motion move arm link 0

        self.joint_values_static = self.arm_command.get_current_joint_values()

        if direction == "horizontal":
            # check if the joint values for joint 4 is around 2.98 +- 0.1 if not set it to 2.98
            if self.joint_values_static[4] < self.horizontal_wrist_angle - self.joint_offset_tolerance or self.joint_values_static[4] > self.horizontal_wrist_angle + self.joint_offset_tolerance:
                self.joint_values_static[4] = self.horizontal_wrist_angle # joint value for arm link 4 when wrist is horizontal

        elif direction == "vertical":
            # check if the joint values for joint 4 is around 4.01 +- 0.1 if not set it to 4.01
            if self.joint_values_static[4] < self.vertical_wrist_angle - self.joint_offset_tolerance or self.joint_values_static[4] > self.vertical_wrist_angle + self.joint_offset_tolerance:
                self.joint_values_static[4] = self.vertical_wrist_angle # joint value for arm link 4 when wrist is vertical
        else:
            rospy.logerr("[cavity server] Invalid travel direction")
            return "failed"

        try:
            self.arm_command.set_joint_value_target(self.joint_values_static)
        except Exception as e:
            rospy.logerr("unable to set target position: %s" % (str(e)))
            return "failed"
        error_code = self.arm_command.go(wait=True)
        if not error_code == MoveItErrorCodes.SUCCESS:
            return "failed"

    def wiggle_arm_working(self, turn = 1, velocity = 0.08):

        message = TwistStamped()
        message.header.frame_id = "arm_link_5"
        message.twist.linear.x = 0.0
        message.twist.linear.y = 0.0
        message.twist.angular.z = 0.0
        initial_time = rospy.Time.now().to_sec()
        travel_time = 1.0

        while(abs(rospy.Time.now().to_sec() - initial_time) < travel_time):
            message.twist.linear.x = 0.0
            message.twist.linear.y = velocity
            self.arm_velocity_pub.publish(message)
            rospy.sleep(0.05)
        self.stop_arm()

        initial_time = rospy.Time.now().to_sec()
        travel_time = 1.0
        while(abs(rospy.Time.now().to_sec() - initial_time) < travel_time):
            message.twist.linear.x = 0.0
            message.twist.linear.y = -velocity
            self.arm_velocity_pub.publish(message)
            rospy.sleep(0.05)
        self.stop_arm()

        initial_time = rospy.Time.now().to_sec()
        travel_time = 1.0
        while(abs(rospy.Time.now().to_sec() - initial_time) < travel_time):
            message.twist.linear.x = 0.0
            message.twist.linear.y = -velocity
            self.arm_velocity_pub.publish(message)
            rospy.sleep(0.05)
        self.stop_arm()

        initial_time = rospy.Time.now().to_sec()
        travel_time = 1.0
        while(abs(rospy.Time.now().to_sec() - initial_time) < travel_time):
            message.twist.linear.x = 0.0
            message.twist.linear.y = velocity
            self.arm_velocity_pub.publish(message)
            rospy.sleep(0.05)
        self.stop_arm()

        return True

    def execute(self, userdata):
        
        message = TwistStamped()
        message.header.frame_id = "base_link"

        self.object_name = Utils.get_value_of(userdata.goal.parameters, "peg")
        adjustment_list = ["rotational", "linear"]

        if self.object_name in self.linear_wiggle_object_list:
            self.type_of_adjustment = "linear"
        elif self.object_name in self.rotational_wiggle_object_list:
            self.type_of_adjustment = "rotational"
        elif self.object_name is None:
            rospy.logwarn("No object name received")
            self.object_name = "unknown"

        if self.type_of_adjustment == "rotational" or self.object_name == "unknown": 

            rospy.loginfo("Adjusting %s with a %s movement", self.object_name, self.type_of_adjustment)

            self.rotational_wiggle("CW", message, wiggle_velocity=7.0)
            rospy.loginfo("Rotational wiggle CW done")
        
            self.rotational_wiggle("CCW", message, wiggle_velocity=7.0)
            rospy.loginfo("Rotational wiggle CCW done")

            
        if self.type_of_adjustment == "linear" or self.object_name == "unknown":

            
            rospy.loginfo("Adjusting %s with a %s movement", self.object_name, self.type_of_adjustment)

            rospy.loginfo("Starting linear wiggle in cartesian mode")

            # TODO: This is a quick  fix only one direction will work, need a general solution
            self.wiggle_arm_working(velocity = 0.1)

            rospy.loginfo("Horizontal wiggle done") 

        return "succeeded"

# ===============================================================================

class Unstage_to_place(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=["success","failed"],input_keys=["goal","heavy_objects", "platform","object"])
        self.platform = "PLATFORM_MIDDLE"
        self.obj = "M20"

        self.unstage_client = SimpleActionClient('unstage_object_server', GenericExecuteAction)
        self.unstage_client.wait_for_server()

    def execute(self,userdata):

        self.platform = Utils.get_value_of(userdata.goal.parameters, "platform")
        self.obj = Utils.get_value_of(userdata.goal.parameters, "peg")

        if self.obj is None:
            rospy.logwarn('Missing parameter "object". Using default.')
            self.obj = "light"
        else:
            self.obj =  "light"

	    # Assigning the goal    

        goal = GenericExecuteGoal()
        goal.parameters.append(KeyValue(key="platform", value=self.platform))
        goal.parameters.append(KeyValue(key="object", value=self.obj))

        self.unstage_client.send_goal(goal)
        self.unstage_client.wait_for_result(rospy.Duration.from_sec(25.0))

        return "success"

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
    rospy.init_node("insert_cavity_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal"],
        output_keys=["feedback", "result"],
    )
    with sm:
        # set pregrasp planner params
        smach.StateMachine.add(
            "SET_PREGRASP_PARAMS",
            gbs.set_named_config("pregrasp_planner_no_sampling"),
            transitions={
                "success": "PERCEIVE_LOCATION",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        # start perception
        smach.StateMachine.add(
            "PERCEIVE_LOCATION",
            gas.perceive_location(obj_category="multimodal_object_recognition_cavity"),
            transitions={
                "success": "SELECT_CAVITY",
                "failed": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SELECT_CAVITY",
            SelectCavity("/mcr_perception/object_selector/input/object_name", vertical=False),
            transitions={
                "succeeded": "GENERATE_OBJECT_POSE",
                "failed": "GET_VERTICAL_CAVITY"
            },
        )

        smach.StateMachine.add(
            "GET_VERTICAL_CAVITY",
            SelectCavity("/mcr_perception/object_selector/input/object_name", vertical=True),
            transitions={
                "succeeded": "GENERATE_OBJECT_POSE",
                "failed": "OVERALL_FAILED"
            },
        )

        # generates a pose of object
        smach.StateMachine.add(
            "GENERATE_OBJECT_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/mcr_perception/object_selector/event_in", "e_trigger")],
                event_out_list=[("/mcr_perception/object_selector/event_out", "e_selected", True)],
                timeout_duration=10,
            ),
            transitions={
                "success": "UNSTAGE_FOR_PLACING",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        # unstage object to place
        smach.StateMachine.add(
            "UNSTAGE_FOR_PLACING",
            Unstage_to_place(),
            transitions={
                "success":"SET_DBC_PARAMS", 
                "failed":"OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "SET_DBC_PARAMS",
            gbs.set_named_config("dbc_pick_object"),
            transitions={
                "success": "MOVE_ROBOT_AND_TRY_INSERTING",
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )

        smach.StateMachine.add(
            "MOVE_ROBOT_AND_TRY_INSERTING",
            gbs.send_and_wait_events_combined(
                event_in_list=[("/wbc/event_in", "e_start")],
                event_out_list=[("/wbc/event_out", "e_success", True)],
                timeout_duration=50,
            ),
            transitions={
                "success": "OPEN_GRIPPER",
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

        # open gripper
        smach.StateMachine.add(
            "OPEN_GRIPPER",
            gms.control_gripper(0.2),
            transitions={
                "succeeded": "MOVE_ARM_UP",
                "timeout": "MOVE_ARM_UP"
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_UP",
            MoveArmUp(),
            transitions={
                "succeeded": "WIGGLE_ARM",
            },
        )

        # wiggling the arm for precision placement
        smach.StateMachine.add(
            "WIGGLE_ARM",
            ppt_wiggle_arm(wiggle_yaw=1.57),
            transitions={
                "succeeded": "MOVE_ARM_TO_HOLD",
                "failed": "MOVE_ARM_TO_HOLD",
            },
        )

        # move arm to HOLD position
        smach.StateMachine.add(
            "MOVE_ARM_TO_HOLD",
            gms.move_arm("pre_place"),
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
            "insert_cavity_smach_viewer", sm, "/INSERT_CAVITY_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="insert_cavity_server",
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










