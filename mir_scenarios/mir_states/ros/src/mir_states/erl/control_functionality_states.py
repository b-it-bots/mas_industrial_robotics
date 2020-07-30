#!/usr/bin/python

import math

import at_work_robot_example_ros.msg
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String


class select_phase_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["success", "failure"], input_keys=["benchmark_state"]
        )

        # calibration phase selected path
        self.calibration_phase_selected_path = rospy.get_param(
            "/fbm3/path_specifications/calibration_phase/selected_path", None
        )
        assert (
            self.calibration_phase_selected_path is not None
        ), "Calibration phase: Selected path(line or sine) must be specified."

        # calibration phase: step size to interpolate path points
        self.calibration_phase_step_size = rospy.get_param(
            "/fbm3/path_specifications/calibration_phase/path/step_size", None
        )
        assert (
            self.calibration_phase_step_size is not None
        ), "Calibration phase: Step size must be specified."

        # calibration phase: line slope
        self.calibration_phase_line_slope = rospy.get_param(
            "/fbm3/path_specifications/calibration_phase/path/line/slope", None
        )
        assert (
            self.calibration_phase_line_slope is not None
        ), "Calibration phase: Path(line) slope must be specified."

        # Execution phase selected path
        self.execution_phase_selected_path = rospy.get_param(
            "/fbm3/path_specifications/execution_phase/selected_path", None
        )
        assert (
            self.execution_phase_selected_path is not None
        ), "Execution phase: Selected path(line or sine) must be specified."

        # Execution phase: step size to interpolate path points
        self.execution_phase_step_size = rospy.get_param(
            "/fbm3/path_specifications/execution_phase/path/step_size", None
        )
        assert (
            self.execution_phase_step_size is not None
        ), "Execution phase: Step size must be specified."

        # Execution phase: line slope
        self.execution_phase_line_slope = rospy.get_param(
            "/fbm3/path_specifications/execution_phase/path/line/slope", None
        )
        assert (
            self.execution_phase_line_slope is not None
        ), "Execution phase: Path(line) slope must be specified."

        # Execution phase: line slope
        self.execution_phase_sine_amplitude = rospy.get_param(
            "/fbm3/path_specifications/execution_phase/path/sine/sine_amplitude", None
        )
        assert (
            self.execution_phase_sine_amplitude is not None
        ), "Execution phase: Sine amplitude must be specified."

        # Execution phase: line slope
        self.execution_phase_sine_angle_conversion_factor = rospy.get_param(
            "/fbm3/path_specifications/execution_phase/path/sine/sine_angle_conversion_factor",
            None,
        )
        assert (
            self.execution_phase_sine_angle_conversion_factor is not None
        ), "Execution phase: Sine angle conversion factor must be specified."

    def execute(self, userdata):

        if (
            userdata.benchmark_state.phase.data
            == at_work_robot_example_ros.msg.BenchmarkState.PREPARATION
        ):
            rospy.set_param(
                "/path_generator/path_selector", self.calibration_phase_selected_path
            )
            rospy.set_param(
                "/path_generator/line_slope", self.calibration_phase_line_slope
            )
            rospy.set_param(
                "/path_generator/step_size", self.calibration_phase_step_size
            )
        elif (
            userdata.benchmark_state.phase.data
            == at_work_robot_example_ros.msg.BenchmarkState.EXECUTION
        ):
            rospy.set_param(
                "/path_generator/path_selector", self.execution_phase_selected_path
            )
            rospy.set_param(
                "/path_generator/line_slope", self.execution_phase_line_slope
            )
            rospy.set_param("/path_generator/step_size", self.execution_phase_step_size)
            rospy.set_param(
                "/path_generator/sine_amplitude", self.execution_phase_sine_amplitude
            )
            rospy.set_param(
                "/path_generator/sine_angle_conversion_factor",
                self.execution_phase_sine_angle_conversion_factor,
            )
        else:
            return "failure"

        return "success"


class initialize_fbm(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success"],
            input_keys=[
                "reference_point",
                "start_point",
                "goal_point",
                "move_arm_to",
                "logging_status",
            ],
            output_keys=[
                "reference_point",
                "start_point",
                "goal_point",
                "move_arm_to",
                "logging_status",
            ],
        )

        # reference frame
        self.selected_coordinate_system_name = rospy.get_param(
            "/fbm3/path_specifications/selected_coordinate_system_name", None
        )
        assert (
            self.selected_coordinate_system_name is not None
        ), "Selected coordinate system name must be specified."

        # calibration distance to be travelled in preparation phase
        self.calibration_distance = rospy.get_param(
            "/fbm3/path_specifications/calibration_phase/calibration_distance", None
        )
        assert (
            self.calibration_distance is not None
        ), "Calibration distance must be specified."

        self.start_of_the_path = rospy.get_param(
            "/fbm3/path_specifications/execution_phase/start_of_the_path", None
        )
        assert (
            self.start_of_the_path is not None
        ), "Start of the path must be specified."

        self.end_of_the_path = rospy.get_param(
            "/fbm3/path_specifications/execution_phase/end_of_the_path", None
        )
        assert self.end_of_the_path is not None, "End of the path must be specified."

        # Execution phase selected path
        self.execution_phase_selected_path = rospy.get_param(
            "/fbm3/path_specifications/execution_phase/selected_path", None
        )
        assert (
            self.execution_phase_selected_path is not None
        ), "Execution phase: Selected path(line or sine) must be specified."

        self.arm_calibration_configuration = rospy.get_param(
            "/fbm3/path_specifications/calibration_phase/arm_calibration_configuration/"
            + str(self.execution_phase_selected_path),
            None,
        )

        assert (
            self.arm_calibration_configuration is not None
        ), "Arm configuration for calibration point must be specified."

    def execute(self, userdata):

        userdata.reference_point = PointStamped()
        userdata.reference_point.header.frame_id = self.selected_coordinate_system_name

        userdata.reference_point.point.x = (
            self.start_of_the_path - self.calibration_distance
        )

        userdata.start_point = PointStamped()
        userdata.start_point.header.frame_id = self.selected_coordinate_system_name

        userdata.start_point.point.x = self.start_of_the_path

        userdata.goal_point = PointStamped()
        userdata.goal_point.header.frame_id = self.selected_coordinate_system_name

        userdata.goal_point.point.x = self.end_of_the_path

        userdata.move_arm_to = self.arm_calibration_configuration

        userdata.logging_status = True

        return "success"


class reset_fbm(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success"],
            input_keys=["logging_status"],
            output_keys=["logging_status"],
        ),

    def execute(self, userdata):

        userdata.logging_status = False

        return "success"


class log_offline_data(smach.State):
    def __init__(self, topic_name=None):
        smach.State.__init__(
            self, outcomes=["success", "failure"], input_keys=["offline_data"]
        ),

        if topic_name is None:
            rospy.logerr("[send_data:] Invalid topic name %s" % (topic_name))
        else:
            self.data_pub = rospy.Publisher(topic_name, PoseStamped)
            self.topic_name = topic_name

        self.offline_data_msgs = ["reference_pose", "starting_pose", "ending_pose"]

    def execute(self, userdata):

        if userdata.offline_data:
            self.data_pub.publish(userdata.offline_data)
        else:
            rospy.logerr(
                "[log_offline_data] Invalid offline data for topic (%s)"
                % (self.topic_name)
            )
            return "failure"

        return "success"


class publish_task_data(smach.State):
    def __init__(self, topic_name=None):
        smach.State.__init__(
            self, outcomes=["success", "failure"], input_keys=["task_data"]
        ),

        if topic_name is None:
            rospy.logerr("[publish_task_data] Invalid topic name %s" % (topic_name))
        else:
            self.task_data_pub = rospy.Publisher(topic_name, PointStamped)
            self.topic_name = topic_name

    def execute(self, userdata):
        if userdata.task_data:
            self.task_data_pub.publish(userdata.task_data)
            return "success"
        else:
            rospy.logwarn(
                "[publish_task_data] Invalid task data for topic (%s)"
                % (self.topic_name)
            )
            return "failure"


class save_gripper_pose(smach.State):
    GRIPPER_POSE_TOPIC = "/rockin/gripper_pose"

    def __init__(self, timeout=2.0):
        smach.State.__init__(
            self, outcomes=["success", "failure"], output_keys=["end_effector_pose"]
        ),

        self.gripper_pose = None
        self.object_pose_sub = rospy.Subscriber(
            self.GRIPPER_POSE_TOPIC, PoseStamped, self.gripper_pose_cb, queue_size=1
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
                    "[save_gripper_pose]Timeout of %f seconds exceeded waiting for gripper pose"
                    % float(timeout_dur.to_sec())
                )
                return "failure"
            rospy.sleep(0.1)

        return "success"
