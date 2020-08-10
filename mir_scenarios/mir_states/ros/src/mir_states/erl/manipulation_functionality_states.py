#!/usr/bin/python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped


class increment_base_position(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["incremented", "reset"],
            input_keys=["base_increments"],
            output_keys=["base_increments"],
        ),

        self.MAX_BASE_POSITIONS = 2

    def execute(self, userdata):
        if userdata.base_increments >= self.MAX_BASE_POSITIONS:
            return "reset"
        else:
            userdata.base_increments += 1
            return "incremented"


class reset_base(smach.State):
    def __init__(self, step_size=0.25):
        smach.State.__init__(
            self,
            outcomes=["done"],
            input_keys=["base_increments", "move_base_by"],
            output_keys=["base_increments", "move_base_by"],
        ),
        self.step_size = step_size

    def execute(self, userdata):
        userdata.move_base_by = (0.0, (userdata.base_increments) * self.step_size, 0.0)
        print "move base by: ", userdata.move_base_by
        userdata.base_increments = 0
        return "done"


class select_object_to_be_grasped(smach.State):
    OBJECT_POSE_TOPIC = "/mir_states/object_selector/object_pose"

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["obj_selected", "no_obj_selected"],
            input_keys=["recognized_objects"],
        ),
        self.object_pose_pub = rospy.Publisher(self.OBJECT_POSE_TOPIC, PoseStamped)

    def execute(self, userdata):
        if userdata.recognized_objects:
            for o in userdata.recognized_objects:
                if o.name in ["pick_EM-01", "pick_EM-02"]:
                    self.object_pose_pub.publish(o.pose)
                    return "obj_selected"
            self.object_pose_pub.publish(userdata.recognized_objects[0].pose)
            return "obj_selected"
        else:
            return "no_obj_selected"


class save_gripper_pose(smach.State):
    GRIPPER_POSE_TOPIC = "/rockin/gripper_pose"

    def __init__(self, timeout=2.0):
        smach.State.__init__(
            self, outcomes=["success", "failure"], output_keys=["end_effector_pose"]
        ),

        self.gripper_pose = None
        self.object_pose_sub = rospy.Subscriber(
            self.GRIPPER_POSE_TOPIC, PoseStamped, self.gripper_pose_cb
        )
        self.timeout = timeout

    def gripper_pose_cb(self, callback_msg):
        self.gripper_pose = callback_msg

    def execute(self, userdata):
        self.gripper_pose = None

        timeout_dur = rospy.Duration.from_sec(self.timeout)
        start_time = rospy.Time.now()

        while True:
            if self.gripper_pose:
                userdata.end_effector_pose = self.gripper_pose
                return "success"
            elif (rospy.Time.now() - start_time) > timeout_dur:
                rospy.logerr(
                    "Timeout of %f seconds exceeded waiting for gripper pose"
                    % float(timeout.to_sec())
                )
                return "failure"
            rospy.sleep(0.1)

        return "success"


class set_is_object_grasped(smach.State):
    def __init__(self, is_object_grasped):
        smach.State.__init__(
            self, outcomes=["success"], output_keys=["is_object_grasped"]
        ),

        self.is_object_grasped = is_object_grasped

    def execute(self, userdata):
        userdata.is_object_grasped = self.is_object_grasped
        return "success"
