#!/usr/bin/env python
"""
Check if the response of the following is empty or not

.. code-block:: bash

    rosservice call /rosplan_knowledge_base/state/goals "predicate_name: ''"

If it is, then there are no unfinished goals
"""

import rospy
from rosplan_knowledge_msgs.srv import GetAttributeService
from std_msgs.msg import String


class KnowledgeBaseAnalizer(object):
    """
    Analyzes if there are new goals in the knowledge base

    """

    def __init__(self):
        # Subscribers
        rospy.Subscriber(
            "~pending_goals/event_in", String, self.pending_goals_event_in_cb
        )
        # Publishers
        self.pending_goals_event_out = rospy.Publisher(
            "~pending_goals/event_out", String
        )

    def pending_goals_event_in_cb(self, msg):
        """
        Obtains an event for the component.

        :param msg: Incoming message
        :type msg: std_msgs.msg.String

        """
        rospy.loginfo("Pending goal request received...")
        rospy.loginfo("Waiting for service: /rosplan_knowledge_base/state/goals")
        rospy.wait_for_service("/rosplan_knowledge_base/state/goals")
        rospy.loginfo(
            "Service /rosplan_knowledge_base/state/goals is available, proceeding"
        )
        event_out = String()
        event_out.data = "e_failure"
        try:
            pending_goals = rospy.ServiceProxy(
                "/rosplan_knowledge_base/state/goals", GetAttributeService
            )
            response = pending_goals("")
            if str(response) == "attributes: []":
                rospy.loginfo("There are no pending goals in the knowledge base")
                event_out.data = "e_no_goals"
            else:
                rospy.loginfo("There are pending goals in the knowledge base")
                event_out.data = "e_goals_available"
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        rospy.loginfo("Publishing pending goals response")
        self.pending_goals_event_out.publish(event_out)

    def start(self):
        """
        Starts the component in a loop till rosnode is alive.

        """
        rospy.loginfo("Pending_goals_analyzer node initilized...")
        rospy.spin()


def main():
    rospy.init_node("pending_goals_analyzer")
    knowledge_base_analyzer = KnowledgeBaseAnalizer()
    knowledge_base_analyzer.start()
