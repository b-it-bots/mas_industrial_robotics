#!/usr/bin/env python
"""
This component moves the base of a robot to a target pose and provides
feedback of its result, i.e. 'e_success' if the pose was reached or 'e_failure'
otherwise. The component also preempts the motion towards the target
pose if an 'e_stop' message is received.

**Assumptions:**
  * The target pose is reachable.

**Input(s):**
  * `pose_in`: The target pose to which the robot's base should be moved.

**Output(s):**
  * `goal`: The navigation goal for the robot's base.

"""
# -*- encoding: utf-8 -*-

import actionlib
import actionlib_msgs.msg
import geometry_msgs.msg
import move_base_msgs.msg
import rospy
import std_msgs.msg
import numpy as np


class MoveBase(object):
    """
    Moves the base of a robot to a target pose

    """

    def __init__(self):
        # Params
        self.event = None
        self.pose_in = None
        self.client_result = None
        self.path_plan = None

        # Action name to move the robot's base
        self.move_base_action_name = rospy.get_param("~move_base_action_name", None)
        assert (
            self.move_base_action_name is not None
        ), "Action name for 'move_base' must be specified."

        # Node cycle rate (in hz).
        self.loop_rate = rospy.Rate(rospy.get_param("~loop_rate", 10))

        # Publishers
        self.event_out = rospy.Publisher(
            "~event_out", std_msgs.msg.String, queue_size=1
        )
        self.remaining_dist_pub = rospy.Publisher(
            "~remaining_distance", std_msgs.msg.Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~pose_in", geometry_msgs.msg.PoseStamped, self.pose_in_cb)
        rospy.Subscriber("~path_plan", geometry_msgs.msg.PoseArray, self.path_plan_cb)

        # Actions
        self.move_base_action = actionlib.SimpleActionClient(
            self.move_base_action_name, move_base_msgs.msg.MoveBaseAction
        )
        rospy.loginfo("Waiting for '{0}' server".format(self.move_base_action_name))
        self.move_base_action.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(self.move_base_action_name))

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def pose_in_cb(self, msg):
        """
        Obtains the target pose.

        """
        self.pose_in = msg

    def client_result_cb(self, state, result):
        """
        Obtains the final status of the client.

        """
        self.client_result = state

    def client_feedback_cb(self, feedback):
        """
        Obtains the feedback from move base and publishes
        remaining distance in the path
        """
        if not self.path_plan:
            return

        def get_dist(p1, p2):
            return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

        path_poses = np.array([[p.position.x, p.position.y] for p in self.path_plan.poses])
        base_pose = np.array([feedback.base_position.pose.position.x, feedback.base_position.pose.position.y])
        dist = np.sum((path_poses - base_pose) ** 2, axis=1)
        # find point in the path to which the base is closest
        closest_pose_idx = np.argmin(dist)
        # distance to closest point (this makes the distance estimation inaccurate because
        # the base may have already crossed the point)
        distance_remaining = dist[closest_pose_idx]
        # distance of remaining points in the path
        for idx in range(closest_pose_idx, len(self.path_plan.poses)-1):
            distance_remaining += get_dist(self.path_plan.poses[idx].position, self.path_plan.poses[idx+1].position)
        self.remaining_dist_pub.publish(distance_remaining)

    def path_plan_cb(self, msg):
        """
        Receive global path plan from move base
        """
        self.path_plan = msg

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = "INIT"

        while not rospy.is_shutdown():

            if state == "INIT":
                state = self.init_state()
            elif state == "IDLE":
                state = self.idle_state()
            elif state == "CONFIGURING":
                state = self.configuring_state()
            elif state == "RUNNING":
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == "e_start":
            return "IDLE"
        else:
            return "INIT"

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == "e_stop":
            self.reset_component_data()
            self.event_out.publish("e_stopped")
            return "INIT"
        elif self.pose_in:
            return "CONFIGURING"
        else:
            return "IDLE"

    def configuring_state(self):
        """
        Executes the CONFIGURING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose = self.pose_in

        self.move_base_action.send_goal(goal, done_cb=self.client_result_cb, feedback_cb=self.client_feedback_cb)

        return "RUNNING"

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == "e_stop":
            self.move_base_action.cancel_goal()
            self.reset_component_data()
            self.event_out.publish("e_stopped")
            return "INIT"
        else:
            if self.client_result == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
                self.event_out.publish("e_success")
                self.reset_component_data()
                return "INIT"
            elif (
                self.client_result == actionlib_msgs.msg.GoalStatus.ABORTED
                or self.client_result == actionlib_msgs.msg.GoalStatus.REJECTED
            ):
                self.event_out.publish("e_failure")
                self.reset_component_data()
                return "INIT"
            else:
                return "RUNNING"

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.pose_in = None
        self.client_result = None
        self.path_plan = None


def main():
    rospy.init_node("move_base", anonymous=True)
    move_base = MoveBase()
    move_base.start()
