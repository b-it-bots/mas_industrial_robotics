import tf
import numpy as np
import rospy

from brics_actuator.msg import JointPositions
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
# from mcr_manipulation_pose_selector_ros.reachability_pose_selector import PoseSelector
from mcr_manipulation_measurers_ros.pose_transformer import PoseTransformer
from mir_pregrasp_planning_ros.kinematics import Kinematics

class OrientationIndependentIK(object):

    def __init__(self, ee_to_arm_dist=0.11, debug=False):
        self.debug = debug
        self._base_link_frame = 'base_link'
        self.kinematics = Kinematics(root=self._base_link_frame, tip='gripper_static_grasp_link')
        # self._ee_to_arm_transform = np.identity(4)
        # self._ee_to_arm_transform[2, 3] = -ee_to_arm_dist

        # self._pose_selector = PoseSelector()
        self._pose_transformer = PoseTransformer()
        self._tf_listener = tf.TransformListener()
        self._arm_base_frame = 'arm_link_0'

        self._base_link_to_arm_base_offset = None
        self._initialise_base_to_arm_offset()

        if self.debug:
            self._pose_array_pub = rospy.Publisher('~pose_samples', PoseArray, queue_size=1)
            self._pose_out_pub = rospy.Publisher('~pose_out', PoseStamped, queue_size=1)
            rospy.sleep(0.5)
        rospy.logdebug('[orientation_independent_ik] Initialised')

    def get_joint_msg_from_point(self, x, y, z, frame_id):
        """
        Calculates the joint angles for the arm's end effector to reach a point.
        The orientation is sampled around the given point to find an IK solution.
        The sampling is done in stages to reduce computational time. The stages are
        based on existing knowledge about the placement of the arm on robot base
        and where the arm can and cannot reach.

        :x: float
        :y: float
        :z: float
        :frame_id: str
        :returns: brics_actuator.JointPositions or None

        """
        pitch_ranges = [(0.0, 0.0), (-30.0, 0.0), (-60.0, -30.0), (-90.0, -60.0), (0.0, 10.0)]
        return self._get_joint_msg_from_point_and_pitch_ranges(x, y, z, frame_id, pitch_ranges)

    def get_joint_msg_from_point_and_pitch(self, x, y, z, pitch, pitch_tolerance, frame_id):
        """
        Calculates the joint angles for the arm's end effector to reach a point
        with given pitch angle (plus or minus some given tolerance)
        The orientation is sampled around the given point to find an IK solution.
        The sampling is done in stages to reduce computational time. 

        pitch is zero when the gripper fingers are facing down.
        pitch is -90 degrees when gripper fingers are facing forward

        Note: pitch and pitch_tolerance are considered in degrees

        :x: float
        :y: float
        :z: float
        :pitch: float
        :pitch_tolerance: float
        :frame_id: str
        :returns: brics_actuator.JointPositions or None

        """
        tolerance = abs(pitch_tolerance)
        pitch_ranges = [(pitch, pitch), (pitch-tolerance, pitch), (pitch, pitch+tolerance)]
        return self._get_joint_msg_from_point_and_pitch_ranges(x, y, z, frame_id, pitch_ranges)

    def _get_joint_msg_from_point_and_pitch_ranges(self, x, y, z, frame_id, pitch_ranges):
        self._initialise_base_to_arm_offset()
        if self._base_link_to_arm_base_offset is None:
            rospy.logerr('Could not get translation from ' + str(self._base_link_frame) + ' to ' + str(self._arm_base_frame))
            return None

        input_pose = PoseStamped()
        input_pose.header.frame_id = frame_id
        input_pose.header.stamp = rospy.Time.now()
        input_pose.pose.position.x = x
        input_pose.pose.position.y = y
        input_pose.pose.position.z = z
        input_pose.pose.orientation.w = 1.0
        transformed_input_pose = self._pose_transformer.get_transformed_pose(input_pose,
                                                                             self._base_link_frame)
        if transformed_input_pose is None:
            rospy.logerr("Unable to transform pose to {0}".format(self._base_link_frame))

        input_pose = transformed_input_pose

        delta_x = input_pose.pose.position.x - self._base_link_to_arm_base_offset[0]
        delta_y = input_pose.pose.position.y - self._base_link_to_arm_base_offset[1]
        yaw_rad = np.arctan2(delta_y, delta_x)
        yaw = np.degrees(yaw_rad)

        found_solution = False
        for pitch_range in pitch_ranges:
            pose_samples = self._generate_samples(input_pose,
                                                  pitch_min=pitch_range[0],
                                                  pitch_max=pitch_range[1],
                                                  pitch_offset=180.0,
                                                  pitch_samples=5,
                                                  yaw_min=yaw,
                                                  yaw_max=yaw,
                                                  yaw_offset=0.0,
                                                  yaw_samples=1)

            if self.debug:
                self._pose_array_pub.publish(pose_samples)
                print(pitch_range)
            # reachable_pose, joint_msg, _ = self._pose_selector.get_reachable_pose_and_configuration(
            #         pose_samples, None)
            reachable_pose, joint_angles = self._get_reachable_pose_and_configuration(pose_samples)
            if reachable_pose is not None:
                found_solution = True
                rospy.logdebug('Found solution')
                rospy.logdebug('Pitch range: ' + str(pitch_range))
                if self.debug:
                    self._pose_out_pub.publish(reachable_pose)
                return joint_angles
        return None

    def _get_reachable_pose_and_configuration(self, pose_samples):
        for pose in pose_samples.poses:
            solution = self.kinematics.inverse_kinematics(pose)
            print(solution)
            if solution is not None:
                reachable_pose = PoseStamped(pose=pose)
                reachable_pose.header.frame_id = self._base_link_frame
                reachable_pose.header.stamp = rospy.Time.now()
                return reachable_pose, solution
        return None, None

    def _generate_samples(self, input_pose,
                          roll_min=0.0, roll_max=0.0, roll_samples=1, roll_offset=0.0,
                          pitch_min=0.0, pitch_max=0.0, pitch_samples=1, pitch_offset=0.0,
                          yaw_min=0.0, yaw_max=0.0, yaw_samples=1, yaw_offset=0.0):
        """
        Generate samples around input pose within given limits. The amount of samples
        generated can be calculated by (`roll_samples` * `pitch_samples * `yaw_samples`)

        Note: All min, max and offset values are in degrees

        :input_pose: geometry_msgs/PoseStamped
        :roll_min: float
        :roll_max: float
        :roll_samples: int
        :roll_offset: float
        :pitch_min: float
        :pitch_max: float
        :pitch_samples: int
        :pitch_offset: float
        :yaw_min: float
        :yaw_max: float
        :yaw_samples: int
        :yaw_offset: float
        :returns: geometry_msgs/PoseArray

        """
        roll_samples = list(np.linspace(roll_min, roll_max, roll_samples))
        pitch_samples = list(np.linspace(pitch_min, pitch_max, pitch_samples))
        yaw_samples = list(np.linspace(yaw_min, yaw_max, yaw_samples))

        pose_samples = PoseArray()
        pose_samples.header = input_pose.header
        for roll in roll_samples:
            for pitch in pitch_samples:
                for yaw in yaw_samples:
                    pose = Pose()
                    pose.position = input_pose.pose.position
                    roll_rad = np.radians(roll + roll_offset)
                    pitch_rad = np.radians(pitch + pitch_offset)
                    yaw_rad = np.radians(yaw + yaw_offset)
                    pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad))
                    pose_samples.poses.append(pose)
                    # transformed_pose = self._get_arm_pose_from_ee_pose(pose)
                    # pose_samples.poses.append(transformed_pose)
        return pose_samples

    # def _get_arm_pose_from_ee_pose(self, gripper_pose):
    #     """ (Copied from mcr_moveit_client)
    #     Transform 'gripper_pose' into 'arm_pose' such that if 'gripper_pose' represents where
    #     the gripper must be, then 'arm_pose' represents where the arm_link_5 must be

    #     :gripper_pose: geometry_msgs/Pose
    #     :returns: geometry_msgs/Pose

    #     """
    #     # convert gripper_pose to gripper_pose_matrix
    #     gripper_pose_matrix = tf.transformations.quaternion_matrix([
    #         gripper_pose.orientation.x,
    #         gripper_pose.orientation.y,
    #         gripper_pose.orientation.z,
    #         gripper_pose.orientation.w])
    #     gripper_pose_matrix[0, 3] = gripper_pose.position.x
    #     gripper_pose_matrix[1, 3] = gripper_pose.position.y
    #     gripper_pose_matrix[2, 3] = gripper_pose.position.z

    #     # transform gripper to arm_pose using a transformation matrix
    #     arm_pose_matrix = np.dot(gripper_pose_matrix, self._ee_to_arm_transform)

    #     # convert arm_pose_matrix to arm_pose
    #     arm_pose = Pose()
    #     arm_pose.position.x = arm_pose_matrix[0, 3]
    #     arm_pose.position.y = arm_pose_matrix[1, 3]
    #     arm_pose.position.z = arm_pose_matrix[2, 3]
    #     quaternion = tf.transformations.quaternion_from_matrix(arm_pose_matrix)
    #     arm_pose.orientation = Quaternion(*quaternion)
    #     return arm_pose

    def _initialise_base_to_arm_offset(self):
        num_of_tries = 0
        while self._base_link_to_arm_base_offset is None and num_of_tries < 3:
            try:
                trans, _ = self._tf_listener.lookupTransform(self._base_link_frame,
                                                             self._arm_base_frame,
                                                             rospy.Time(0))
                self._base_link_to_arm_base_offset = (trans[0], trans[1])
            except Exception as e:
                rospy.sleep(1.0)
