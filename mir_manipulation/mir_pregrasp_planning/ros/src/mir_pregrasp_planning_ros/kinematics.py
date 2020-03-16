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
        self.tolerance = tolerance
        lower_limit, upper_limit = self._get_joint_limits()
        self.chain = self._get_chain()
        self.num_of_joints = self.chain.getNrOfJoints()
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.vik = kdl.ChainIkSolverVel_pinv(self.chain)

        # self.ik_solver = kdl.ChainIkSolverPos_NR(self.chain, self.fk_solver, self.vik)
        self.ik_solver = kdl.ChainIkSolverPos_NR_JL(self.chain, lower_limit, upper_limit,
                                                    self.fk_solver, self.vik,
                                                    maxiter=100, eps=self.tolerance)

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
        ee_frame = kdl.Frame()
        self.fk_solver.JntToCart(joint_arr, ee_frame)
        ee_pose = Kinematics.get_pose_from_frame(ee_frame)
        return ee_pose

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
        reached_frame = kdl.Frame()
        self.fk_solver.JntToCart(target_joint_arr, reached_frame)
        if kdl.Equal(reached_frame, target_frame, self.tolerance):
            return [target_joint_arr[i] for i in range(self.num_of_joints)]
        else:
            return None

    def _get_joint_limits(self):
        robot = URDF.from_parameter_server()
        chain = robot.get_chain(self.root, self.tip, links=False, fixed=False)
        joints = []
        for joint_name in chain:
            for joint in robot.joints:
                if joint.name == joint_name:
                    joints.append(joint)
                    break
        num_of_joints = len(joints)
        lower_limit = kdl.JntArray(num_of_joints)
        upper_limit = kdl.JntArray(num_of_joints)
        for i, joint in enumerate(joints):
            lower_limit[i] = joint.limit.lower
            upper_limit[i] = joint.limit.upper
        return lower_limit, upper_limit

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
