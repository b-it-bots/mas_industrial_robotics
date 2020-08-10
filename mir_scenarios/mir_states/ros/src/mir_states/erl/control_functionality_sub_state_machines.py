#!/usr/bin/python

# import of generic states
import mir_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms
import mir_states.rockin.control_functionality_states as cfs
import mir_states.rockin.referee_box_states as rockin_refbox
import rospy
import smach
import smach_ros


class initilization_phase(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["success", "failure"],
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
        with self:
            smach.StateMachine.add(
                "STOP_ALL",
                stop_all(),
                transitions={
                    "success": "START_END_EFFECTOR_POSE_LOGGER",
                    "failure": "failure",
                },
            )

            smach.StateMachine.add(
                "START_END_EFFECTOR_POSE_LOGGER",
                gbs.send_and_wait_events_combined(
                    event_in_list=[("/end_effector_pose_logger/event_in", "e_start")]
                ),
                transitions={
                    "success": "INIT_DATA",
                    "timeout": "failure",
                    "failure": "failure",
                },
            )

            smach.StateMachine.add(
                "INIT_DATA", cfs.initialize_fbm(), transitions={"success": "success"}
            )


class calibration_phase(smach.StateMachine):
    def __init__(self, log_benchmark_data=False):
        smach.StateMachine.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=[
                "move_arm_to",
                "reference_point",
                "end_effector_pose",
                "benchmark_state",
                "logging_status",
            ],
        )
        with self:
            smach.StateMachine.add(
                "MOVE_ARM_TO_CALIBRATION_CONFIGURATION",
                gms.move_arm(),
                transitions={"succeeded": "START_LOGGING", "failed": "START_LOGGING"},
            )

            smach.StateMachine.add(
                "START_LOGGING",
                start_logging_offline_data(log_benchmark_data),
                transitions={"success": "STOP_COMPUTE_TASK_FRAME_TRANSFORM"},
            )

            smach.StateMachine.add(
                "STOP_COMPUTE_TASK_FRAME_TRANSFORM",
                gbs.send_and_wait_events_combined(
                    event_in_list=[("/compute_transform/event_in", "e_stop")],
                    event_out_list=[
                        ("/compute_transform/event_out", "e_stopped", True)
                    ],
                    timeout_duration=15,
                ),
                transitions={
                    "success": "PUBLISH_REFERENCE_POINT",
                    "timeout": "PUBLISH_REFERENCE_POINT",
                    "failure": "PUBLISH_REFERENCE_POINT",
                },
            )

            smach.StateMachine.add(
                "PUBLISH_REFERENCE_POINT",
                cfs.publish_task_data(topic_name="/compute_transform/reference_point"),
                transitions={
                    "success": "COMPUTE_TASK_FRAME_TRANSFORM",
                    "failure": "failure",
                },
                remapping={"task_data": "reference_point"},
            )

            smach.StateMachine.add(
                "COMPUTE_TASK_FRAME_TRANSFORM",
                gbs.send_and_wait_events_combined(
                    event_in_list=[("/compute_transform/event_in", "e_start")],
                    event_out_list=[
                        ("/compute_transform/event_out", "e_success", True)
                    ],
                    timeout_duration=15,
                ),
                transitions={
                    "success": "success",
                    "timeout": "failure",
                    "failure": "failure",
                },
            )


class preparation_phase(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=[
                "reference_point",
                "start_point",
                "end_effector_pose",
                "benchmark_state",
            ],
        )
        with self:
            smach.StateMachine.add(
                "GET_GRIPPER_POSE_FOR_REFERENCE",
                cfs.save_gripper_pose(),
                transitions={"success": "LOG_REFERENCE_POINT", "failure": "failure"},
            )

            smach.StateMachine.add(
                "LOG_REFERENCE_POINT",
                cfs.log_offline_data(topic_name="/rockin/reference_pose"),
                transitions={"success": "SELECT_CONFIGURATION", "failure": "failure"},
                remapping={"offline_data": "end_effector_pose"},
            )

            smach.StateMachine.add(
                "SELECT_CONFIGURATION",
                cfs.select_phase_configuration(),
                transitions={"success": "MOVE_ARM_TO_START", "failure": "failure"},
            )

            smach.StateMachine.add(
                "MOVE_ARM_TO_START",
                control_functionality_pipeline(),
                transitions={"success": "GET_GRIPPER_POSE", "failure": "failure"},
                remapping={"start": "reference_point", "goal": "start_point"},
            )

            smach.StateMachine.add(
                "GET_GRIPPER_POSE",
                cfs.save_gripper_pose(),
                transitions={"success": "LOG_START_POINT", "failure": "failure"},
            )

            smach.StateMachine.add(
                "LOG_START_POINT",
                cfs.log_offline_data(topic_name="/rockin/starting_pose"),
                transitions={"success": "success", "failure": "failure"},
                remapping={"offline_data": "end_effector_pose"},
            )


class execution_phase(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=[
                "start_point",
                "goal_point",
                "end_effector_pose",
                "benchmark_state",
            ],
        )
        with self:
            smach.StateMachine.add(
                "SELECT_CONFIGURATION",
                cfs.select_phase_configuration(),
                transitions={"success": "MOVE_ARM_TO_GOAL", "failure": "failure"},
            )

            smach.StateMachine.add(
                "MOVE_ARM_TO_GOAL",
                control_functionality_pipeline(),
                transitions={"success": "GET_GRIPPER_POSE", "failure": "failure"},
                remapping={"start": "start_point", "goal": "goal_point"},
            )

            smach.StateMachine.add(
                "GET_GRIPPER_POSE",
                cfs.save_gripper_pose(),
                transitions={"success": "LOG_END_POINT", "failure": "failure"},
            )

            smach.StateMachine.add(
                "LOG_END_POINT",
                cfs.log_offline_data(topic_name="/rockin/ending_pose"),
                transitions={"success": "success", "failure": "success"},
                remapping={"offline_data": "end_effector_pose"},
            )


class stop_all(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["success", "failure"])
        with self:
            smach.StateMachine.add(
                "STOP_ALL",
                gbs.send_and_wait_events_combined(
                    event_in_list=[
                        ("/compute_transform/event_in", "e_stop"),
                        ("/linear_interpolator_demo/event_in", "e_stop"),
                        (
                            "/linear_interpolator_demo_trajectory_executor/event_in",
                            "e_stop",
                        ),
                        ("/end_effector_pose_logger/event_in", "e_stop"),
                    ]
                ),
                transitions={
                    "success": "success",
                    "timeout": "failure",
                    "failure": "failure",
                },
            )


class finish_phase(smach.StateMachine):
    def __init__(self, log_benchmark_data=False):
        smach.StateMachine.__init__(
            self, outcomes=["success", "failure"], input_keys=["logging_status"]
        )
        with self:
            smach.StateMachine.add(
                "STOP_LOGGING",
                stop_logging_offline_data(log_benchmark_data),
                transitions={"success": "STOP_ALL"},
            )

            smach.StateMachine.add(
                "STOP_ALL",
                gbs.send_and_wait_events_combined(
                    event_in_list=[
                        ("/compute_transform/event_in", "e_stop"),
                        ("/linear_interpolator_demo/event_in", "e_stop"),
                        (
                            "/linear_interpolator_demo_trajectory_executor/event_in",
                            "e_stop",
                        ),
                        ("/end_effector_pose_logger/event_in", "e_stop"),
                    ]
                ),
                transitions={
                    "success": "success",
                    "timeout": "failure",
                    "failure": "failure",
                },
            )


class start_logging_offline_data(smach.StateMachine):
    def __init__(self, log_benchmark_data=False):
        smach.StateMachine.__init__(
            self, outcomes=["success"], input_keys=["logging_status", "enable_logging"]
        )
        with self:
            if log_benchmark_data:
                smach.StateMachine.add(
                    "START_LOGGING_OFFLINE_DATA",
                    gbs.send_and_wait_events_combined(
                        event_in_list=[("/rosbag_recorder/event_in", "e_start")]
                    ),
                    transitions={
                        "success": "SEND_LOGGING_STATUS",
                        "timeout": "SEND_LOGGING_STATUS",
                        "failure": "SEND_LOGGING_STATUS",
                    },
                )

            smach.StateMachine.add(
                "SEND_LOGGING_STATUS",
                rockin_refbox.send_refbox_logging_status(),
                transitions={"done": "success"},
            )


class stop_logging_offline_data(smach.StateMachine):
    def __init__(self, log_benchmark_data=False):
        smach.StateMachine.__init__(
            self, outcomes=["success"], input_keys=["logging_status"]
        )
        with self:
            if log_benchmark_data:
                smach.StateMachine.add(
                    "RESET_FBM",
                    cfs.reset_fbm(),
                    transitions={"success": "STOP_LOGGING_OFFLINE_DATA"},
                )

                smach.StateMachine.add(
                    "STOP_LOGGING_OFFLINE_DATA",
                    gbs.send_and_wait_events_combined(
                        event_in_list=[("/rosbag_recorder/event_in", "e_stop")]
                    ),
                    transitions={
                        "success": "SEND_LOGGING_STATUS",
                        "timeout": "SEND_LOGGING_STATUS",
                        "failure": "SEND_LOGGING_STATUS",
                    },
                )
            else:
                smach.StateMachine.add(
                    "RESET_FBM",
                    cfs.reset_fbm(),
                    transitions={"success": "SEND_LOGGING_STATUS"},
                )

            smach.StateMachine.add(
                "SEND_LOGGING_STATUS",
                rockin_refbox.send_refbox_logging_status(),
                transitions={"done": "success"},
            )


class control_functionality_pipeline(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=["success", "failure"], input_keys=["start", "goal"]
        )
        with self:

            smach.StateMachine.add(
                "PUBLISH_START_POINT",
                cfs.publish_task_data(topic_name="/path_generator/start_point"),
                transitions={"success": "PUBLISH_GOAL_POINT", "failure": "failure"},
                remapping={"task_data": "start"},
            )

            smach.StateMachine.add(
                "PUBLISH_GOAL_POINT",
                cfs.publish_task_data(topic_name="/path_generator/end_point"),
                transitions={"success": "PLAN_PATH", "failure": "failure"},
                remapping={"task_data": "goal"},
            )

            smach.StateMachine.add(
                "PLAN_PATH",
                gbs.send_and_wait_events_combined(
                    event_in_list=[("/linear_interpolator_demo/event_in", "e_start")],
                    event_out_list=[
                        ("/linear_interpolator_demo/event_out", "e_success", True)
                    ],
                    timeout_duration=30,
                ),
                transitions={
                    "success": "EXECUTE_PLAN",
                    "timeout": "failure",
                    "failure": "failure",
                },
            )

            smach.StateMachine.add(
                "EXECUTE_PLAN",
                gbs.send_and_wait_events_combined(
                    event_in_list=[
                        (
                            "/linear_interpolator_demo_trajectory_executor/event_in",
                            "e_start",
                        )
                    ],
                    event_out_list=[
                        (
                            "/linear_interpolator_demo_trajectory_executor/event_out",
                            "e_success",
                            True,
                        )
                    ],
                    timeout_duration=60,
                ),
                transitions={
                    "success": "STOP_ALL",
                    "timeout": "failure",
                    "failure": "failure",
                },
            )

            smach.StateMachine.add(
                "STOP_ALL",
                gbs.send_and_wait_events_combined(
                    event_in_list=[
                        ("/linear_interpolator_demo/event_in", "e_stop"),
                        (
                            "/linear_interpolator_demo_trajectory_executor/event_in",
                            "e_stop",
                        ),
                    ],
                    timeout_duration=15,
                ),
                transitions={
                    "success": "success",
                    "timeout": "failure",
                    "failure": "failure",
                },
            )
