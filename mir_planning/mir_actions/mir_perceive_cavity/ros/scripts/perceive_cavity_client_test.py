#! /usr/bin/env python

import sys
import rospy
from actionlib import SimpleActionClient

from actionlib_msgs.msg import GoalStatus
from mir_planning_msgs.msg import GenericExecuteAction, GenericExecuteGoal
from diagnostic_msgs.msg import KeyValue

if __name__ == '__main__':
    rospy.init_node('perceive_cavity_client_tester')

    client = SimpleActionClient('perceive_cavity_server', GenericExecuteAction)
    client.wait_for_server()

    goal = GenericExecuteGoal()
    if len(sys.argv) > 1:
        location = str(sys.argv[1]).upper()
    else:
        location = "PP01"
    goal.parameters.append(KeyValue(key='location', value=location))

    rospy.loginfo('Sending following goal to perceive cavity server')
    rospy.loginfo(goal)

    client.send_goal(goal)

    timeout = 15.0
    finished_within_time = client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    if not finished_within_time:
        client.cancel_goal()

    state = client.get_state()
    result = client.get_result()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo('Action SUCCESS')
        for kv in result.results:
            rospy.loginfo(kv.value)
    elif state == GoalStatus.ABORTED:
        rospy.logerr('Action FAILED')
    else:
        rospy.logwarn('State: ' + str(state))
        rospy.loginfo(result)
