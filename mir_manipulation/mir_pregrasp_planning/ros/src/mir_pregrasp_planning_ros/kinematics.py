import math
import PyKDL as kdl

from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam
from geometry_msgs.msg import Pose, Quaternion

class Kinematics(object):

    """Kinematics class for youbot arm using kdl library

       Note: Input pose for inverse_kinematics has to be in `root` frame"""

    def __init__(self, tip='gripper_static_grasp_link'):
        self.root = 'arm_link_0'
        self.tip = tip
        self.lower_limit, self.upper_limit = self._get_joint_limits()
        self.chain = self._get_chain()
        self.ee_to_arm_dist = None

        num_of_segments = self.chain.getNrOfSegments()
        if num_of_segments == 6 and self.chain.getSegment(5).getName() == self.tip:
            self.ee_to_arm_dist = self.chain.getSegment(5).getFrameToTip().p.z()
        else:
            rospy.logerr('Given tip not found in chain. Using default (arm_link_0)')
            self.tip = 'arm_link_5'
            self.chain = self._get_chain()
            self.ee_to_arm_dist = 0.0

        self.num_of_joints = self.chain.getNrOfJoints()
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

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
        NOTE: pose has to be in arm_link_0 frame

        :pose: geometry_msgs.Pose
        :initial_angles: list
        :return: list of float

        """
        if initial_angles is None:
            initial_angles = [0.0] * self.num_of_joints

        solutions = []
        for offset_joint_1 in [True, False]:
            for offset_joint_3 in [True, False]:
                solution = self._analytical_ik(pose, offset_joint_1, offset_joint_3)
                if self._is_solution_within_limits(solution):
                    solutions.append(solution)

        if len(solutions) == 0:
            return None
        # return the solution nearest to intial angles
        differences = [sum([sol - init for sol, init in zip(solution, initial_angles)]) for solution in solutions]
        return solutions[differences.index(min(differences))]
    
    def _analytical_ik(self, pose, offset_joint_1=True, offset_joint_3=True):
        """
        Adapted to python from 
        https://github.com/mas-group/youbot-manipulation/blob/indigo/youbot_arm_kinematics/common/src/inverse_kinematics.cpp#L64
        """
        g0 = Kinematics.get_frame_from_pose(pose)
        # Parameters from youBot URDF file
        l0x = 0.024
        l0z = 0.096
        l1x = 0.033
        l1z = 0.019
        l2 = 0.155
        l3 = 0.135

        # Distance from arm_link_4 to arm_link_5
        d = 0.13 + self.ee_to_arm_dist

        j1 = 0.0
        j2 = 0.0
        j3 = 0.0
        j4 = 0.0
        j5 = 0.0

        # Transform from frame 0 to frame 1
        frame0_to_frame1 = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-l0x, 0, -l0z))
        g1 = frame0_to_frame1 * g0

        # First joint
        j1 = math.atan2(g1.p.y(), g1.p.x())
        if offset_joint_1:
            j1 += math.pi if j1 < 0.0 else -math.pi

        # Fifth joint, determines the roll of the gripper (= wrist angle)
        s1 = math.sin(j1)
        c1 = math.cos(j1)
        r11 = g1.M[0, 0]
        r12 = g1.M[0, 1]
        r21 = g1.M[1, 0]
        r22 = g1.M[1, 1]
        j5 = math.atan2(r21 * c1 - r11 * s1, r22 * c1 - r12 * s1)

        # Transform from frame 1 to frame 2
        frame1_to_frame2 = kdl.Frame(kdl.Rotation.RPY(0, 0, -j1), kdl.Vector(-l1x, 0, -l1z))
        g2 = frame1_to_frame2 * g1

        # Project the frame into the plane of the arm
        g2_proj = self._project_goal_orientation_into_arm_subspace(g2)

        # Set all values in the frame that are close to zero to exactly zero
        for i in range(3):
            for j in range(3):
                if abs(g2_proj.M[i, j]) < 0.000001:
                    g2_proj.M[i, j] = 0

        # The sum of joint angles two to four determines the overall "pitch" of the
        # end effector
        r13 = g2_proj.M[0, 2]
        r33 = g2_proj.M[2, 2]
        j234 = math.atan2(r13, r33)

        p2 = g2_proj.p

        # In the arm's subplane, offset from the end-effector to the fourth joint
        p2.x(p2.x() - d * math.sin(j234))
        p2.z(p2.z() - d * math.cos(j234))

        # Check if the goal position can be reached at all
        if (l2 + l3) < math.sqrt((p2.x() * p2.x()) + (p2.z() * p2.z())):
            return None

        # Third joint
        l_sqr = (p2.x() * p2.x()) + (p2.z() * p2.z())
        l2_sqr = l2 * l2
        l3_sqr = l3 * l3
        j3_cos = (l_sqr - l2_sqr - l3_sqr) / (2.0 * l2 * l3)

        if j3_cos > 0.9999999:
            j3 = 0.0
        elif j3_cos < -0.9999999:
            j3 = math.pi
        else:
            j3 = math.atan2(math.sqrt(1.0 - (j3_cos * j3_cos)), j3_cos)

        if offset_joint_3:
            j3 *= -1

        # Second joint
        t1 = math.atan2(p2.z(), p2.x())
        t2 = math.atan2(l3 * math.sin(j3), l2 + l3 * math.cos(j3))
        j2 = (math.pi/2) - t1 - t2

        # Fourth joint, determines the pitch of the gripper
        j4 = j234 - j2 - j3

        # This IK assumes that the arm points upwards, so we need to consider
        # the offsets to the real home position
        offset1 = math.radians( 169.0)
        offset2 = math.radians(  65.0)
        offset3 = math.radians(-146.0)
        offset4 = math.radians( 102.5)
        offset5 = math.radians( 167.5)

        solution = [0.0]*5
        solution[0] = offset1 - j1
        solution[1] = j2 + offset2
        solution[2] = j3 + offset3
        solution[3] = j4 + offset4
        solution[4] = offset5 - j5

        # print("Configuration without offsets: ")
        # print(j1, j2, j3, j4, j5)
        # print("Configuration with offsets: ")
        # print([round(i, 2) for i in solution])
        # print()
        return solution

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
        lower_limit = [0.0]*num_of_joints
        upper_limit = [0.0]*num_of_joints
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

    def _project_goal_orientation_into_arm_subspace(self, goal):
        y_t_hat = goal.M.UnitY()    # y vector of the rotation matrix
        z_t_hat = goal.M.UnitZ()    # z vector of the rotation matrix

        # m_hat is the normal of the "arm plane"
        m_hat = kdl.Vector(0, -1, 0)

        # k_hat is the vector about which rotation of the goal frame is performed
        k_hat = m_hat * z_t_hat         # cross product

        # z_t_hat_tick is the new pointing direction of the arm
        z_t_hat_tick = k_hat * m_hat    # cross product

        # the amount of rotation
        cos_theta = kdl.dot(z_t_hat, z_t_hat_tick)
        # first cross product then dot product
        sin_theta = kdl.dot(z_t_hat * z_t_hat_tick, k_hat)

        # use Rodrigues' rotation formula to perform the rotation
        y_t_hat_tick = (cos_theta * y_t_hat) \
                        + (sin_theta * (k_hat * y_t_hat)) \
                        + (1 - cos_theta) * (kdl.dot(k_hat, y_t_hat)) * k_hat
                        # k_hat * y_t_hat is cross product
        x_t_hat_tick = y_t_hat_tick * z_t_hat_tick  # cross product

        rot = kdl.Rotation(x_t_hat_tick, y_t_hat_tick, z_t_hat_tick)

        # the frame uses the old position but has the new, projected orientation
        return kdl.Frame(rot, goal.p);

    def _is_solution_within_limits(self, solution):
        if solution is None or len(solution) != 5:
            return False
        for i in range(5):
            if solution[i] < self.lower_limit[i] or solution[i] > self.upper_limit[i]:
                return False
        return True

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
