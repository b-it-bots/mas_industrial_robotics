#! /usr/bin/env python

import sys
import rospy
from actionlib import SimpleActionClient

from actionlib_msgs.msg import GoalStatus
from mir_planning_msgs.msg import GenericExecuteAction, GenericExecuteGoal
from diagnostic_msgs.msg import KeyValue

if __name__ == '__main__':
    rospy.init_node('move_base_safe_client_tester')

    client = SimpleActionClient('move_base_safe_server', GenericExecuteAction)
    client.wait_for_server()

    if len(sys.argv) > 1:
        destination = str(sys.argv[1]).upper()
        next_action = ''
        if len(sys.argv) > 2:
            next_action = str(sys.argv[2]).upper()

        goal = GenericExecuteGoal()
        goal.parameters.append(KeyValue(key='arm_safe_position', value='barrier_tape'))
        goal.parameters.append(KeyValue(key='destination_location', value=destination))
        goal.parameters.append(KeyValue(key='next_action', value=next_action))
        rospy.loginfo('Sending following goal to move base safe server')
        rospy.loginfo(goal)

        client.send_goal(goal)

        timeout = 60.0
        finished_within_time = client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        if not finished_within_time:
            client.cancel_goal()

        state = client.get_state()
        result = client.get_result()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('Action SUCCESS')
            rospy.loginfo(client.get_result())
        elif state == GoalStatus.ABORTED:
            rospy.logerr('Action FAILED')
        else:
            rospy.logwarn('State: ' + str(state))
            rospy.loginfo(client.get_result())
    else:
        rospy.logerr('Arguments were not received in the proper format !')
        rospy.loginfo('usage : move_base DESTINATION')
