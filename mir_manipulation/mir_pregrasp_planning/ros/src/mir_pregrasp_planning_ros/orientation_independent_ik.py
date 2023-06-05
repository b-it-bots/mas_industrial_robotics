from __future__ import print_function
import tf
import numpy as np
import rospy

from brics_actuator.msg import JointPositions, JointValue
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
from mcr_manipulation_measurers_ros.pose_transformer import PoseTransformer
from mir_pregrasp_planning_ros.kinematics import Kinematics

class OrientationIndependentIK(object):

    def __init__(self, debug=False):
        self.debug = debug
        self._base_link_frame = 'base_link'
        self.kinematics = Kinematics(tip='gripper_static_grasp_link')

        self._pose_transformer = PoseTransformer()
        self._tf_listener = tf.TransformListener()
        self._arm_base_frame = 'arm_link_0'

        self._base_link_to_arm_base_offset = None
        self._initialise_base_to_arm_offset()

        # create subscriber to pick from shelf topic to limit orientation independent ik to certain pitch ranges
        self.is_picking_from_shelf = False

        if self.debug:
            self._pose_array_pub = rospy.Publisher('~pose_samples', PoseArray, queue_size=1)
            self._pose_out_pub = rospy.Publisher('~pose_out', PoseStamped, queue_size=1)
            rospy.sleep(0.5)
        rospy.logdebug('[orientation_independent_ik] Initialised')

    def get_joint_msg_from_point(self, x, y, z, frame_id):
        """
        Calculates the joint angles for the arm's end effector to reach a point.
        The orientation is sampled around the given point to find an IK solution.

        :x: float
        :y: float
        :z: float
        :frame_id: str
        :returns: brics_actuator.JointPositions or None

        """
        reachable_pose, joint_msg = self.get_reachable_pose_and_joint_msg_from_point(x, y, z, frame_id)
        return joint_msg

    def get_reachable_pose_and_joint_msg_from_point(self, x, y, z, frame_id):
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
        :returns: (geometry_msgs.msg.PoseStamped, brics_actuator.JointPositions) or None

        """
        # check is_picking_from_shelf from parameter server
        self.is_picking_from_shelf = bool(rospy.get_param("/pick_from_shelf_server/pick_statemachine_says_shelf", False))
        rospy.loginfo(f"[orientation_independent_ik] shelf picking is set to: {self.is_picking_from_shelf}")
        if self.is_picking_from_shelf=='True':
            pitch_ranges = [(-45.0, -15.0)] # limiting the pitch range to avoid collisions with shelf and table
        else:
            pitch_ranges = [(0.0, 0.0), (-30.0, 0.0), (-60.0, -30.0), (-90.0, -60.0), (0.0, 10.0)]
        return self._get_reachable_pose_and_joint_msg_from_point_and_pitch_ranges(
                x, y, z, frame_id, pitch_ranges)

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
        reachable_pose, joint_msg = self._get_reachable_pose_and_joint_msg_from_point_and_pitch_ranges(
                x, y, z, frame_id, pitch_ranges)
        return joint_msg

    def _get_reachable_pose_and_joint_msg_from_point_and_pitch_ranges(
            self, x, y, z, frame_id, pitch_ranges):
        """
        Iteratively generate samples for each pitch_range and first the first
        pose that has an ik solution

        :x: float
        :y: float
        :z: float
        :frame_id: str
        :pitch_ranges: list[tuple(float, float)]
        :returns: brics_actuator.JointPositions or None
        """
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
            return None

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
            reachable_pose, joint_angles = self._get_reachable_pose_and_configuration(pose_samples)
            if reachable_pose is not None:
                found_solution = True
                rospy.loginfo('Found solution')
                rospy.loginfo('Pitch range: ' + str(pitch_range))
                rospy.loginfo('Yaw : ' + str(yaw))
                if self.debug:
                    self._pose_out_pub.publish(reachable_pose)
                return (reachable_pose, OrientationIndependentIK.get_joint_pos_msg_from_joint_angles(joint_angles))
        return None

    def _get_reachable_pose_and_configuration(self, pose_samples):
        """
        Out of all the pose samples, return the first reachable pose and its ik
        solution.

        :pose_samples: geometry_msgs.msg.PoseArray
        :returns: tuple(geometry_msgs.msg.PoseStamped, list(float))

        """
        for pose in pose_samples.poses:
            # transform pose to arm base frame (since ik only works with that)
            pose_stamped = PoseStamped(pose=pose, header=pose_samples.header)
            trans_pose = self._pose_transformer.get_transformed_pose(pose_stamped,
                                                                     self._arm_base_frame)
            if trans_pose is None:
                rospy.logerr("Unable to transform pose to {0}".format(self._arm_base_frame))

            solution = self.kinematics.inverse_kinematics(trans_pose.pose)
            if solution is not None:
                reachable_pose = PoseStamped(pose=pose, header=pose_samples.header)
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
        return pose_samples

    def _initialise_base_to_arm_offset(self):
        num_of_tries = 0
        while self._base_link_to_arm_base_offset is None and num_of_tries < 3:
            num_of_tries += 1
            try:
                trans, _ = self._tf_listener.lookupTransform(self._base_link_frame,
                                                             self._arm_base_frame,
                                                             rospy.Time(0))
                self._base_link_to_arm_base_offset = (trans[0], trans[1])
            except Exception as e:
                rospy.sleep(1.0)

    @staticmethod
    def get_joint_pos_msg_from_joint_angles(joint_angles):
        """
        Converts a list of joint angles to a JointPositions message

        :joint_angles: list(float)
        :returns: brics_actuator.JointPositions

        """
        joint_position_msg = JointPositions()
        for i, joint_angle in enumerate(joint_angles):
            joint_value_msg = JointValue()
            joint_value_msg.joint_uri = 'arm_joint_' + str(i+1)
            joint_value_msg.unit = 'rad'
            joint_value_msg.value = joint_angle
            joint_position_msg.positions.append(joint_value_msg)
        return joint_position_msg
