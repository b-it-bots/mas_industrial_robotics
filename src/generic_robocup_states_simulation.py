import roslib
roslib.load_manifest('raw_generic_states')

import rospy
import smach

from tasks import *


class get_basic_navigation_task(smach.State):

    """
    Get task description for basic navigation task (no RefereeBox, use
    hard-coded specification).
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['task_received', 'wrong_task_format'],
                             input_keys=['task'], output_keys=['task'])

    def execute(self, userdata):
        nav_task = 'BNT<(D1,W,1),(S1,E,3),(S2,E,3),(D2,S,3),(S3,W,3),(S2,W,3),(D2,W,3),(S1,W,3),(S2,W,3),(S3,W,3),(S2,W,3),(D1,W,3)>'
        try:
            task = BNTTask(nav_task)
        except TaskSpecFormatError:
            return 'wrong_task_format'
        rospy.loginfo('Parsed task:\n%s' % task)
        userdata.task = task
        return 'task_received'
