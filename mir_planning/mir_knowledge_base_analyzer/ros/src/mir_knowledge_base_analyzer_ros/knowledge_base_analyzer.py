#!/usr/bin/env python
import rospy
from rosplan_knowledge_msgs.srv import GetAttributeService
from std_msgs.msg import String

"""
rosservice call /kcl_rosplan/get_current_goals "predicate_name: ''"
check if it is empty, this means there are no unfinished goals
"""

class KnowledgeBaseAnalizer(object):
    """
    Analyzes if there are new goals in the knowledge base

    """

    def __init__(self):
        # Subscribers
        rospy.Subscriber("~pending_goals/event_in", String, self.pending_goals_event_in_cb)
        # Publishers
        self.pending_goals_event_out = rospy.Publisher('~pending_goals/event_out', String)


    def pending_goals_event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        rospy.loginfo('Pending goal request received...')
        rospy.loginfo('Waiting for service: /kcl_rosplan/get_current_goals')
        rospy.wait_for_service('/kcl_rosplan/get_current_goals')
        rospy.loginfo('Service /kcl_rosplan/get_current_goals is available, proceeding')
        event_out = String()
        event_out.data = 'e_failure'
        try:
            pending_goals = rospy.ServiceProxy('/kcl_rosplan/get_current_goals', GetAttributeService)
            response = pending_goals('')
            if str(response) == 'attributes: []':
                rospy.loginfo('There are no pending goals in the knowledge base')
                event_out.data = 'e_no_goals'
            else:
                rospy.loginfo('There are pending goals in the knowledge base')
                event_out.data = 'e_goals_available'
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s'%e)
        rospy.loginfo('Publishing pending goals response')
        self.pending_goals_event_out.publish(event_out)

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo('Pending_goals_analyzer node initilized...')
        rospy.spin()


def main():
    rospy.init_node('pending_goals_analyzer')
    knowledge_base_analyzer = KnowledgeBaseAnalizer()
    knowledge_base_analyzer.start()
