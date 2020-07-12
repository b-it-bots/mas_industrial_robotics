#! /usr/bin/env python

import sys

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import KeyValue
from mir_planning_msgs.msg import GenericExecuteAction, GenericExecuteGoal

if __name__ == "__main__":
    rospy.init_node("stage_object_client_tester")

    client = SimpleActionClient("stage_object_server", GenericExecuteAction)
    client.wait_for_server()

    if len(sys.argv) > 1:
        platform = str(sys.argv[1]).upper()
    else:
        platform = "PLATFORM_MIDDLE"

    goal = GenericExecuteGoal()
    goal.parameters.append(KeyValue(key="platform", value=platform))
    rospy.loginfo("Sending following goal to stage object server")
    rospy.loginfo(goal)

    client.send_goal(goal)

    timeout = 25.0
    finished_within_time = client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    if not finished_within_time:
        client.cancel_goal()

    state = client.get_state()
    result = client.get_result()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Action SUCCESS")
        rospy.loginfo(client.get_result())
    elif state == GoalStatus.ABORTED:
        rospy.logerr("Action FAILED")
    else:
        rospy.logwarn("State: " + str(state))
        rospy.loginfo(client.get_result())
