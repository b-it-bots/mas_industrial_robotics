import math
import PyKDL as kdl

from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam
from geometry_msgs.msg import Pose, Quaternion

class Kinematics(object):

    """Kinematics class for youbot arm using kdl library

       Note: Input pose for inverse_kinematics has to be in `root` frame"""

    def __init__(self, root='base_link', tip='gripper_static_grasp_link', tolerance=0.001):
        self.root = root
        self.tip = tip
        self.joint_limits = self._get_joint_limits()
        self.joint_limits[0][1] = 5.839
        self.joint_limits[1][1] = 2.535
        self.joint_limits[2][0] = -4.94
        self.joint_limits[2][1] = -0.09
        self.joint_limits[3][0] = 0.11
        self.joint_limits[3][1] = 3.336
        self.joint_limits[4][0] = 0.19
        self.joint_limits[4][1] = 5.61
        print(self.joint_limits)
        self.chain = self._get_chain()
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.vik = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_NR(self.chain, self.fk_solver, self.vik)
        self.num_of_joints = self.chain.getNrOfJoints()
        self.tolerance = tolerance

    def forward_kinematics(self, joint_angles=[]):
        """
        Returns a Pose where the tip would be if given joint angles are applied.

        :joint_angles: list of float
        :return: geometry_msgs.Pose

        """
        if len(joint_angles) != self.num_of_joints:
            return None
        joint_arr = kdl.JntArray(self.num_of_joints)
        for i, joint_angle in enumerate(joint_angles):
            joint_arr[i] = joint_angle
        return self._forward_kinematics_joint_arr(joint_arr)

    def inverse_kinematics(self, pose, initial_angles=None):
        """
        Returns joint angles required for the tip to be at the given pose

        :pose: geometry_msgs.Pose
        :initial_angles: list
        :return: list of float

        """
        if initial_angles is None:
            initial_angles = [0.0] * self.num_of_joints

        init_joint_arr = kdl.JntArray(self.num_of_joints)
        for i, joint_angle in enumerate(initial_angles):
            init_joint_arr[i] = joint_angle

        target_frame = Kinematics.get_frame_from_pose(pose)
        target_joint_arr = kdl.JntArray(self.num_of_joints)
        self.ik_solver.CartToJnt(init_joint_arr, target_frame, target_joint_arr)
        print(target_joint_arr)
        if self._is_ik_solution_valid(target_joint_arr, pose):
            return [target_joint_arr[i] for i in range(self.num_of_joints)]
        else:
            return None

    def _forward_kinematics_joint_arr(self, joint_arr):
        ee_frame = kdl.Frame()
        self.fk_solver.JntToCart(joint_arr, ee_frame)
        ee_pose = Kinematics.get_pose_from_frame(ee_frame)
        return ee_pose

    def _is_ik_solution_valid(self, target_joint_angles, target_pose):
        possible = True
        two_pi = 2 * math.pi
        for i in range(self.num_of_joints):
            print(i)
            joint = target_joint_angles[i]
            if not (self.joint_limits[i][0] < joint < self.joint_limits[i][1]):
                # scale down joint angle to be between -2pi and +2pi
                # if joint > two_pi or joint < -two_pi:
                #     joint %= two_pi if joint > 0.0 else -two_pi
                # revert joint angle from +ve to -ve or vise versa to fit in joint limit
                if not (self.joint_limits[i][0] < joint < self.joint_limits[i][1]):
                    joint += two_pi if joint < 0.0 else -two_pi
                print(joint)
                if self.joint_limits[i][0] < joint < self.joint_limits[i][1]:
                    target_joint_angles[i] = joint
                else:
                    possible = False
                    # break
        if possible:
            pose = self._forward_kinematics_joint_arr(target_joint_angles)
            if self._is_within_tolerance(pose.position.x, target_pose.position.x) and\
               self._is_within_tolerance(pose.position.y, target_pose.position.y) and\
               self._is_within_tolerance(pose.position.z, target_pose.position.z) and\
               self._is_within_tolerance(pose.orientation.x, target_pose.orientation.x) and\
               self._is_within_tolerance(pose.orientation.y, target_pose.orientation.y) and\
               self._is_within_tolerance(pose.orientation.z, target_pose.orientation.z) and\
               self._is_within_tolerance(pose.orientation.w, target_pose.orientation.w):
                   return True
        return False

    def _get_joint_limits(self):
        robot = URDF.from_parameter_server()
        chain = robot.get_chain(self.root, self.tip, links=False, fixed=False)
        joints = []
        for joint_name in chain:
            for joint in robot.joints:
                if joint.name == joint_name:
                    joints.append(joint)
                    break
        limits = []
        for joint in joints:
            limits.append([joint.limit.lower, joint.limit.upper])
        return limits

    def _get_chain(self):
        success, tree = treeFromParam('/robot_description')
        if not success:
            return
        chain = tree.getChain(self.root, self.tip)
        return chain

    @staticmethod
    def get_pose_from_frame(frame):
        """Convert kdl frame obj to geometry_msgs Pose obj

        :frame: PyKDL.Frame
        :returns: geometry_msgs.Pose

        """
        pose = Pose()
        pose.position.x = frame.p.x()
        pose.position.y = frame.p.y()
        pose.position.z = frame.p.z()
        pose.orientation = Quaternion(*frame.M.GetQuaternion())
        return pose

    @staticmethod
    def get_frame_from_pose(pose):
        """Convert geometry_msgs Pose obj to kdl frame obj

        :pose: geometry_msgs.Pose
        :returns: PyKDL.Frame

        """
        rot = kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w)
        pos = kdl.Vector(pose.position.x, pose.position.y, pose.position.z)
        frame = kdl.Frame(rot, pos)
        return frame

    def _is_within_tolerance(self, value1, value2):
        """Check if the two given values are within tolerance

        :value1: float
        :value2: float
        :returns: bool

        """
        return abs(value1 - value2) < self.tolerance
