#!/usr/bin/env python

import os
import sys
import threading

# import of generic states
import mcr_states.common.basic_states as gbs
import mir_states.common.action_states as gas
import rospy
import smach
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from mir_planning_msgs.msg import (
    ExecutePlanAction,
    ExecutePlanGoal,
    PlanAction,
    PlanGoal,
)
from mir_planning_msgs.srv import ReAddGoals
from std_msgs.msg import String

# from mir_audio_receiver.msg import AudioMessage
# ===============================================================================


class re_add_goals(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure"])
        rospy.wait_for_service("/task_planning/re_add_goals")
        self.srv = rospy.ServiceProxy("/task_planning/re_add_goals", ReAddGoals)
        rospy.sleep(0.2)

    def execute(self, userdata):
        res = self.srv()
        if res:
            return "success"
        else:
            return "failure"


# ===============================================================================


class plan_task(smach.State):
    def __init__(self, mode=PlanGoal.NORMAL):
        # # publishing a Audio message at startup 
        # audio_message = AudioMessage()
        # audio_pub = rospy.Publisher("/mir_audio_receiver/tts_request", AudioMessage, queue_size=1)
        # audio_message.message = "Hello, I am the mastermind. I am ready to plan your tasks."
        # audio_pub.publish(audio_message)
        # rospy.sleep(1.0)

        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["domain_file", "problem_file", "planner"],
            output_keys=["plan"],
        )
        self.mode = mode
        self.planner_client = SimpleActionClient(
            "/mir_task_planning/task_planner_server/plan_task", PlanAction
        )
        self.planner_client.wait_for_server()
        rospy.sleep(0.2)

    def execute(self, userdata):
        goal = PlanGoal(
            domain_file=userdata.domain_file,
            problem_file=userdata.problem_file,
            planner=userdata.planner,
            mode=self.mode,
        )
        self.planner_client.send_goal(goal)
        self.planner_client.wait_for_result()
        state = self.planner_client.get_state()
        result = self.planner_client.get_result()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Found a plan with %i actions", len(result.plan.plan))
            userdata.plan = result.plan
            return "success"
        elif state == GoalStatus.ABORTED:
            rospy.logerr("Failed to plan")
            rospy.sleep(2.0)
            return "failure"


# ===============================================================================


class execute_plan(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "failure", "preempted"],
            input_keys=["already_once_executed", "plan"],
            output_keys=["already_once_executed"],
        )
        self.planner_executor_client = SimpleActionClient(
            "/task_planning/execute_plan", ExecutePlanAction
        )
        self.planner_executor_client.wait_for_server()
        rospy.sleep(0.2)

    def execute(self, userdata):
        userdata.already_once_executed = True
        success = self.execute_plan(userdata.plan)
        if not success:
            return "failure"
        return "success"

    def execute_plan(self, plan):
        goal = ExecutePlanGoal()
        goal.plan = plan
        self.planner_executor_client.send_goal(goal)
        finished = self.planner_executor_client.wait_for_result()
        if not finished:
            return False
        res = self.planner_executor_client.get_result()
        return res.success


# ===============================================================================


class check_execution_already_started(smach.State):
    """
    This state will check if the execution has started at least once. If it has
    then it will go to exit otherwise it will go to re add goals
    """

    def __init__(self, max_count=3):
        smach.State.__init__(
            self,
            outcomes=["success", "failure"],
            input_keys=["already_once_executed", "check_execution_counter"],
            output_keys=["check_execution_counter"],
        )
        self.max_count = max_count

    def execute(self, userdata):
        rospy.sleep(3.0)  # time for the system to recover for checking goals
        if userdata.already_once_executed:
            if userdata.check_execution_counter >= self.max_count:
                return "success"
            else:
                userdata.check_execution_counter += 1
                return "failure"
        else:
            return "failure"


# ===============================================================================


def main():
    rospy.init_node("planning_coordinator")
    problem_file = rospy.get_param("~problem_file", None)
    domain_file = rospy.get_param("~domain_file", None)
    planner = rospy.get_param("~planner", "mercury")

    # # publishing a Audio message at startup 
    # audio_message = AudioMessage()
    # audio_pub = rospy.Publisher("/mir_audio_receiver/tts_request", AudioMessage, queue_size=1)
    # audio_message.message = "Hello, I am the planning coordinator"
    # audio_pub.publish(audio_message)

    if problem_file is None or domain_file is None:
        rospy.logfatal("Either domain and/or problem file not provided. Exiting.")
        sys.exit(1)
    # Create a SMACH state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_FAILURE", "OVERALL_PREEMPT", "OVERALL_SUCCESS"]
    )
    sm.userdata.already_once_executed = False
    # counter for how many times did it enter check_execution_already_started()
    sm.userdata.check_execution_counter = 0
    sm.userdata.domain_file = domain_file
    sm.userdata.problem_file = problem_file
    sm.userdata.planner = planner
    # Open the container
    with sm:
        # upload intrinsic knowledge
        smach.StateMachine.add(
            "UPLOAD_INTRINSIC_KNOWLEDGE",
            gbs.send_event([("/upload_knowledge/event_in", "e_trigger")]),
            transitions={"success": "LOOK_FOR_UNFINISHED_GOALS"},
        )

        # TODO:wait for upload intrinsic knowledge succes response

        # send even in to look for goals component to process and see if there are unfinished goals on the knowledge base
        smach.StateMachine.add(
            "LOOK_FOR_UNFINISHED_GOALS",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/knowledge_base_analyzer/pending_goals/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/knowledge_base_analyzer/pending_goals/event_out",
                        "e_goals_available",
                        True,
                    )
                ],
                timeout_duration=5,
            ),
            transitions={
                "success": "GENERATE_PDDL_PROBLEM",
                "timeout": "CHECK_EXECUTION_ALREADY_STARTED",
                "failure": "CHECK_EXECUTION_ALREADY_STARTED",
            },
        )

        smach.StateMachine.add(
            "CHECK_EXECUTION_ALREADY_STARTED",
            check_execution_already_started(max_count=3),
            transitions={"success": "MOVE_TO_EXIT", "failure": "RE_ADD_GOALS"},
        )

        smach.StateMachine.add(
            "RE_ADD_GOALS",
            re_add_goals(),
            transitions={
                "success": "LOOK_FOR_UNFINISHED_GOALS",
                "failure": "LOOK_FOR_UNFINISHED_GOALS",
            },
        )

        # generate problem from knowledge base snapshot
        smach.StateMachine.add(
            "GENERATE_PDDL_PROBLEM",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/mir_planning/pddl_problem_generator/event_in", "e_trigger")
                ],
                event_out_list=[
                    (
                        "/mir_planning/pddl_problem_generator/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=5,
            ),
            transitions={
                "success": "PLAN",
                "timeout": "GENERATE_PDDL_PROBLEM",
                "failure": "OVERALL_FAILURE",
            },
        )

        # plan task (mode can be either PlanGoal.NORMAL or PlanGoal.FAST)
        smach.StateMachine.add(
            "PLAN",
            plan_task(mode=PlanGoal.NORMAL),
            transitions={
                "success": "EXECUTE_PLAN",
                "failure": "LOOK_FOR_UNFINISHED_GOALS",
            },
        )

        smach.StateMachine.add(
            "EXECUTE_PLAN",
            execute_plan(),
            transitions={
                "success": "LOOK_FOR_UNFINISHED_GOALS",
                "failure": "LOOK_FOR_UNFINISHED_GOALS",
                "preempted": "OVERALL_PREEMPT",
            },
        )

        smach.StateMachine.add(
            "MOVE_TO_EXIT",
            gas.move_base("EXIT"),
            transitions={"success": "OVERALL_SUCCESS", "failed": "MOVE_TO_EXIT"},
        )

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    while not rospy.is_shutdown() and smach_thread.isAlive():
        rospy.sleep(1.0)
    if rospy.is_shutdown():
        rospy.logwarn("ctrl + c detected!!! preempting smach execution")
        # Request the container to preempt
        sm.request_preempt()
    smach_thread.join()


if __name__ == "__main__":
    print("Going to main function")
    main()
