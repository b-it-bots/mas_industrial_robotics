import math

PACKAGE = 'raw_generic_states'

import roslib
roslib.load_manifest(PACKAGE)

import tf
import rospy
from actionlib import SimpleActionClient
from simple_script_server import simple_script_server
from arm_navigation_msgs.msg import MoveArmAction, MoveArmGoal, \
                                    PositionConstraint, OrientationConstraint


class Arm(object):

    CART_SERVER = '/arm_controller/move_arm_cart'

    def __init__(self, default_pitch=(math.pi / 2.0), planning_mode=''):
        self.move_arm_cart_server = SimpleActionClient(self.CART_SERVER,
                                                       MoveArmAction)
        self.script_server = simple_script_server()
        self.pitch = default_pitch
        self.planning_mode = planning_mode
        self.blocking = True

    def move_to(self, where, blocking=True):
        self.blocking = blocking
        if isinstance(where, str):
            self._move_to_pose(where)
        elif len(where) == 5:
            self._move_to_joints(where)
        elif len(where) == 4 or len(where) == 3:
            self._move_to_cartesian(where)

    def _move_to_joints(self, joints):
        self.script_server.move('arm', [joints], mode=self.planning_mode,
                                blocking=self.blocking)

    def _move_to_cartesian(self, coordinates):
        """
        Move the arm to the pose given by (x, y, z, pitch) tuple. The pitch is
        optional and can be omitted.
        """
        rospy.logdebug('Waiting for [%s] server...' % (self.CART_SERVER))
        self.move_arm_cart_server.wait_for_server()
        g = MoveArmGoal()
        pc = PositionConstraint()
        pc.header.frame_id = "/base_link"
        pc.header.stamp = rospy.Time.now()
        pc.position.x = coordinates[0]
        pc.position.y = coordinates[1]
        pc.position.z = coordinates[2]
        g.motion_plan_request.goal_constraints.position_constraints.append(pc)
        oc = OrientationConstraint()
        r = 0.0
        p = coordinates[3] if len(coordinates) == 4 else self.pitch
        y = 0.0
        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(r, p, y)
        oc.header.frame_id = "/base_link"
        oc.header.stamp = rospy.Time.now()
        oc.orientation.x = qx
        oc.orientation.y = qy
        oc.orientation.z = qz
        oc.orientation.w = qw
        g.motion_plan_request.goal_constraints.orientation_constraints.append(oc)
        self.move_arm_cart_server.send_goal(g)
        rospy.loginfo('Sent move arm goal, waiting for result...')
        # TODO: this is always blocking
        self.move_arm_cart_server.wait_for_result()
        rv = self.move_arm_cart_server.get_result().error_code.val
        if not rv == 1:
            raise Exception('Failed to move the arm to the given pose.')

    def _move_to_pose(self, pose):
        self.script_server.move('arm', pose, mode=self.planning_mode,
                                blocking=self.blocking)

    def gripper(self, state):
        self.script_server.move('gripper', state)
