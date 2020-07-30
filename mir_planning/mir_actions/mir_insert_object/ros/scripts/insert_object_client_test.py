#! /usr/bin/env python

import sys

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import KeyValue
from mir_planning_msgs.msg import GenericExecuteAction, GenericExecuteGoal

if __name__ == "__main__":
    rospy.init_node("insert_object_client_tester")

    client = SimpleActionClient("insert_object_server", GenericExecuteAction)
    client.wait_for_server()

    if len(sys.argv) == 3:
        goal = GenericExecuteGoal()
        hole = str(sys.argv[1]).upper()
        platform = str(sys.argv[2]).upper()
        goal.parameters.append(KeyValue(key="hole", value=hole))
        goal.parameters.append(KeyValue(key="platform", value=platform))
        rospy.loginfo("Sending following goal to place object server")
        rospy.loginfo(goal)

        client.send_goal(goal)

        timeout = 25.0
        finished_within_time = client.wait_for_result(
            rospy.Duration.from_sec(int(timeout))
        )
        if not finished_within_time:
            client.cancel_goal()

        state = client.get_state()
        result = client.get_result()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Action SUCCESS")
            rospy.loginfo(result)
        elif state == GoalStatus.ABORTED:
            rospy.logerr("Action FAILED")
        else:
            rospy.logwarn("State: " + str(state))
            rospy.loginfo(result)
    else:
        rospy.logerr("Arguments were not received in the proper format !")
        rospy.loginfo("usage : insert OBJECT_NAME PLATFORM_NAME")
