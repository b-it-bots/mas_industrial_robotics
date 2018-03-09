#!/usr/bin/python

import rospy
import roslib
import actionlib
import smach
import smach_ros

import sys
import collections

from mir_yb_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from mir_yb_action_msgs.msg import PerceiveLocationAction, PerceiveLocationGoal
from mir_yb_action_msgs.msg import PickObjectWBCAction, PickObjectWBCGoal
from mir_yb_action_msgs.msg import StageObjectAction, StageObjectGoal
from mir_yb_action_msgs.msg import PlaceObjectAction, PlaceObjectGoal
from mir_yb_action_msgs.msg import UnStageObjectAction, UnStageObjectGoal
from mir_yb_action_msgs.msg import PerceiveLocationAction, PerceiveLocationGoal
from mir_yb_action_msgs.msg import InsertObjectAction, InsertObjectGoal

class place_object(smach.State):
    def __init__(self, platform_name):
        smach.State.__init__(self,
                              outcomes=['success', 'failed'])
        self.place_client = actionlib.SimpleActionClient('place_object_server', PlaceObjectAction)
        self.place_client.wait_for_server()
        self.goal = PlaceObjectGoal()
        self.goal.object = ''
        self.goal.location = platform_name

    def execute(self, userdata):
        self.place_client.send_goal(self.goal)
        self.place_client.wait_for_result(rospy.Duration.from_sec(15.0))
        result = self.place_client.get_result()
        if(result):
            return 'success'
        else:
            return 'failed'

class pick_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                              outcomes=['success', 'failed'])
        self.pick_client = actionlib.SimpleActionClient('wbc_pick_object_server', PickObjectWBCAction) 
        self.pick_client.wait_for_server()
        self.goal = PickObjectWBCGoal()
        self.goal.object = "any"

    def execute(self, userdata):
        self.pick_client.send_goal(self.goal)
        self.pick_client.wait_for_result(rospy.Duration.from_sec(30.0))
        result = self.pick_client.get_result()
        if(result):
            return 'success'
        else:
            return 'failed'

class perceive_location(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                              outcomes=['success', 'failed'])
        self.perceive_location_client = actionlib.SimpleActionClient('perceive_location_server', PerceiveLocationAction)
        self.perceive_location_client.wait_for_server()
        self.goal = PerceiveLocationGoal()
        self.goal.location = ""

    def execute(self, userdata):
        self.perceive_location_client.send_goal(self.goal)
        self.perceive_location_client.wait_for_result(rospy.Duration.from_sec(30.0))
        result = self.perceive_location_client.get_result()
        if(result):
            return 'success'
        else:
            return 'failed'

class move_base(smach.State):
    def __init__(self, destination_location):
        smach.State.__init__(self,
                              outcomes=['success', 'failed'])
        self.move_base_client =  actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
        self.move_base_client.wait_for_server()
        self.goal = MoveBaseSafeGoal()
        self.goal.arm_safe_position = 'barrier_tape'
        self.goal.source_location = ''
        self.goal.destination_location = destination_location

    def execute(self, userdata):
        self.move_base_client.send_goal(self.goal)
        self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(15.0)))
        result = self.move_base_client.get_result()
        if(result):
            return 'success'
        else:
            return 'failed'
