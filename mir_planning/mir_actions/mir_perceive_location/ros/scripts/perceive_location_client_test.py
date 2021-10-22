#! /usr/bin/env python

from __future__ import print_function

import sys

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import KeyValue
from mir_planning_msgs.msg import GenericExecuteAction, GenericExecuteGoal
from mir_actions.utils import Utils

if __name__ == "__main__":
    rospy.init_node("perceive_location_client_tester")

    client = SimpleActionClient("perceive_location_server", GenericExecuteAction)
    client.wait_for_server()

    goal = GenericExecuteGoal()
    if len(sys.argv) > 1:
        location = str(sys.argv[1]).upper()
        goal.parameters.append(KeyValue(key="location", value=location))

    rospy.loginfo("Sending following goal to perceive location server")
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
        print("Perceived objects:")
        for i in range(20):
            obj_key = "obj_" + str(i+1)
            obj_key_id = "obj_" + str(i+1) + "_id"
            obj_name = Utils.get_value_of(result.results, obj_key)
            if obj_name == None:
                break
            
            obj_id = Utils.get_value_of(result.results, obj_key_id)
            print(obj_name.ljust(15), str(obj_id).ljust(3))
    elif state == GoalStatus.ABORTED:
        rospy.logerr("Action FAILED")
    else:
        rospy.logwarn("State: " + str(state))
        rospy.loginfo(result)
