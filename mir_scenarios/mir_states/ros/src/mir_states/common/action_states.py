#!/usr/bin/python

import rospy
from actionlib import SimpleActionClient
import smach

from actionlib_msgs.msg import GoalStatus
from mir_yb_action_msgs.msg import GenericExecuteAction, GenericExecuteGoal
from diagnostic_msgs.msg import KeyValue

class place_object(smach.State):
    def __init__(self, platform_name):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.place_client = SimpleActionClient('place_object_server', GenericExecuteAction)
        self.place_client.wait_for_server()
        self.goal = GenericExecuteGoal()
        self.goal.parameters.append(KeyValue(key='location', value=str(platform_name).upper()))

    def execute(self, userdata):
        self.place_client.send_goal(self.goal)
        self.place_client.wait_for_result(rospy.Duration.from_sec(15.0))
        state = self.place_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failed'

class pick_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.pick_client = SimpleActionClient('wbc_pick_object_server', GenericExecuteAction) 
        self.pick_client.wait_for_server()
        self.goal = GenericExecuteGoal()
        self.goal.parameters.append(KeyValue(key='object', value='any'))

    def execute(self, userdata):
        self.pick_client.send_goal(self.goal)
        self.pick_client.wait_for_result(rospy.Duration.from_sec(30.0))
        state = self.place_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failed'

class perceive_location(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.perceive_location_client = SimpleActionClient('perceive_location_server', GenericExecuteAction)
        self.perceive_location_client.wait_for_server()
        self.goal = GenericExecuteGoal()

    def execute(self, userdata):
        self.perceive_location_client.send_goal(self.goal)
        self.perceive_location_client.wait_for_result(rospy.Duration.from_sec(30.0))
        state = self.place_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failed'

class move_base(smach.State):
    def __init__(self, destination_location):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.move_base_client = SimpleActionClient('move_base_safe_server', GenericExecuteAction)
        self.move_base_client.wait_for_server()
        self.goal = GenericExecuteGoal()
        self.goal.parameters.append(KeyValue(key='arm_safe_position', value='barrier_tape'))
        self.goal.parameters.append(KeyValue(key='destination_location',
                                             value=str(destination_location).upper()))

    def execute(self, userdata):
        self.move_base_client.send_goal(self.goal)
        self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(15.0)))
        state = self.place_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failed'

class insert_object(smach.State):
    def __init__(self, robot_platform, container):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.insert_object_client = SimpleActionClient('insert_object_server', GenericExecuteAction)
        self.insert_object_client.wait_for_server()
        self.goal = GenericExecuteGoal()
        self.goal.parameters.append(KeyValue(key='robot_platform', value=str(robot_platform).upper()))
        self.goal.parameters.append(KeyValue(key='hole', value=str(container).upper()))

    def execute(self, userdata):
        self.insert_object_client.send_goal(self.goal)
        self.insert_object_client.wait_for_result(rospy.Duration.from_sec(int(30.0)))
        state = self.place_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failed'
