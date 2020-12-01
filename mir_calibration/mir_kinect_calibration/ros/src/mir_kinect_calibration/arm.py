import math

import roslib
import rospy
import tf
from actionlib import SimpleActionClient
from arm_navigation_msgs.msg import (
    MoveArmAction,
    MoveArmGoal,
    OrientationConstraint,
    PositionConstraint,
)
from simple_script_server import simple_script_server

PACKAGE = "kinect_calibration"
NODE = "calibrate_kinect"

roslib.load_manifest(PACKAGE)


class Arm(object):
    """
    Constants:
        CART_SERVER:
    """

    CART_SERVER = "/arm_controller/move_arm_cart"

    def __init__(self, default_pitch=(math.pi / 2.0)):
        rospy.loginfo("Initializing arm control.")
        rospy.loginfo("Waiting for [%s] server..." % (self.CART_SERVER))
        self.move_arm_cart_server = SimpleActionClient(self.CART_SERVER, MoveArmAction)
        self.move_arm_cart_server.wait_for_server()
        self.script_server = simple_script_server()
        self.pitch = default_pitch

    def move_to(self, where):
        """
        Function to move the arm as described by variable where
        """
        if isinstance(where, str):
            self._move_to_pose(where)
        elif len(where) == 5:
            self._move_to_joints(where)
        elif len(where) == 4 or len(where) == 3:
            self._move_to_cartesian(where)

    def _move_to_joints(self, joints):
        self.script_server.move("arm", [joints])

    def _move_to_cartesian(self, coordinates):
        """
        Move the arm to the pose given by (x, y, z, pitch) tuple. The pitch is
        optional and can be omitted.
        """
        move_arm_to_goal = MoveArmGoal()
        position_constraint_msg = PositionConstraint()
        position_constraint_msg.header.frame_id = "/base_link"
        position_constraint_msg.header.stamp = rospy.Time.now()
        position_constraint_msg.position.x = coordinates[0]
        position_constraint_msg.position.y = coordinates[1]
        position_constraint_msg.position.z = coordinates[2]
        move_arm_to_goal.motion_plan_request.goal_constraints.position_constraints.append(
            position_constraint_msg)

        orientation_constraint_msg = OrientationConstraint()
        roll_euler = 0.0
        pitch_euler = coordinates[3] if len(coordinates) == 4 else self.pitch
        yaw_euler = 0.0

        (quaternion_x,
         quaternion_y,
         quaternion_z,
         quaternion_w) = tf.transformations.quaternion_from_euler(
             roll_euler,
             pitch_euler,
             yaw_euler)

        orientation_constraint_msg.header.frame_id = "/base_link"
        orientation_constraint_msg.header.stamp = rospy.Time.now()
        orientation_constraint_msg.orientation.x = quaternion_x
        orientation_constraint_msg.orientation.y = quaternion_y
        orientation_constraint_msg.orientation.z = quaternion_z
        orientation_constraint_msg.orientation.w = quaternion_w
        move_arm_to_goal.motion_plan_request.goal_constraints.orientation_constraints.append(
            orientation_constraint_msg)

        self.move_arm_cart_server.send_goal(move_arm_to_goal)
        rospy.loginfo("Sent move arm goal, waiting for result...")
        self.move_arm_cart_server.wait_for_result()
        result_value = self.move_arm_cart_server.get_result().error_code.val
        print result_value
        if not result_value == 1:
            raise Exception("Failed to move the arm to the given pose.")

    def _move_to_pose(self, pose):
        """
        Function to move arm to a pose
        """
        self.script_server.move("arm", pose)

    def open_gripper(self):
        """
        Function to open the gripper
        """
        self.script_server.move("gripper", "open")

    def close_gripper(self):
        """
        Function to close the gripper
        """
        self.script_server.move("gripper", "close")
