#!/usr/bin/env python

"""
Author: Abhishek Padalkar

"""

import math
import time

import numpy as np
import rospy
import tf
from geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseArray,
    PoseStamped,
    TwistStamped,
)
from nav_msgs.msg import Path
from std_msgs.msg import String


class cartesian_velocity_insert:
    def __init__(self):
        print ("Node initiated.........")
        self.tf_listener = tf.TransformListener()

        self.cartesian_velocity_command_pub = (
            "/arm_1/arm_controller/cartesian_velocity_command"
        )
        self.number_of_sampling_points = 30
        self.goal_tolerance = 0.02
        self.vel_publisher = rospy.Publisher(
            self.cartesian_velocity_command_pub, TwistStamped, queue_size=1
        )
        self.object_pose_sub = rospy.Subscriber(
            "/pregrasp_planner_node/pose_samples", PoseArray, self.object_pose_cb
        )
        self.feedforward_gain = 4
        self.feedback_gain = 3
        self.debug_pose_pub = rospy.Publisher(
            "/insert_object_workaround/modified_pose_debug", PoseStamped, queue_size=11
        )
        """
        self.feedforward_gain = rospy.get_param('~feedforward_gain')
        self.feedback_gain = rospy.get_param('~feedback_gain')
        """

        self.tf_listener = tf.TransformListener()
        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param("~loop_rate", 10))
        self.event = None

        rospy.Subscriber("/insert_object_workaroud/event_in", String, self.event_in_cb)
        self.event_out = rospy.Publisher(
            "/insert_object_workaroud/event_out", String, queue_size=1
        )
        self.start_end = None
        self.goal = None
        self.z_offset = 0.03
        self.xy_plane_offset = 0.07

    def object_pose_cb(self, data):
        pose_stamped_ = PoseStamped()
        pose_stamped_.header.frame_id = data.header.frame_id
        pose_stamped_.pose = data.poses[0]
        while not rospy.is_shutdown():
            try:
                pose_stamped_1 = self.tf_listener.transformPose(
                    "/arm_link_0", pose_stamped_
                )
                break
            except:
                continue
        self.goal = [
            pose_stamped_1.pose.position.x,
            pose_stamped_1.pose.position.y,
            pose_stamped_1.pose.position.z,
        ]
        theta = math.atan2(self.goal[1], self.goal[0])
        radius = np.sqrt(self.goal[0] ** 2 + self.goal[1] ** 2)
        radius = radius - self.xy_plane_offset
        pose_stamped_1.pose.position.x = radius * np.cos(theta)
        pose_stamped_1.pose.position.y = radius * np.sin(theta)

        while not rospy.is_shutdown():
            try:
                pose_stamped_2 = self.tf_listener.transformPose(
                    "/base_link", pose_stamped_1
                )
                break
            except:
                continue
        self.debug_pose_pub.publish(pose_stamped_2)
        self.goal = [
            pose_stamped_2.pose.position.x,
            pose_stamped_2.pose.position.y,
            pose_stamped_2.pose.position.z,
        ]
        print "Object pose received ", self.goal

    def event_in_cb(self, msg):
        """
        Starts a planned motion based on the specified arm position.

        """
        print "event_in recieved"
        self.event = msg.data

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start now...")
        state = "INIT"

        while not rospy.is_shutdown():

            if state == "INIT":
                state = self.init_state()
            elif state == "IDLE":
                state = self.idle_state()
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
        elif self.event == "e_stop":
            self.reset_component_data()
            self.event_out.publish("e_stopped")
            return "INIT"
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
        elif self.goal != None:
            return "RUNNING"
        elif self.goal == None and self.event == "e_start":
            rospy.loginfo("Goal not recieved")
            self.event_out.publish("e_failure")
            self.event = None
            return "IDLE"

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == "e_stop":
            self.reset_component_data()
            self.event_out.publish("e_stopped")
            return "INIT"
        else:
            self.execute()
            self.event_out.publish("e_success")
            self.reset_component_data()
        return "INIT"

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.points = None

    def move_arm_to_goal(self):
        print "Moving arm to goal position"
        goal_point = self.goal
        while not rospy.is_shutdown() and self.event != "e_stop":
            try:
                (current_pos, rot) = self.tf_listener.lookupTransform(
                    "/base_link", "/arm_link_5", rospy.Time(0)
                )
                break
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue

        path_x = np.linspace(
            current_pos[0], goal_point[0], self.number_of_sampling_points
        )
        path_y = np.linspace(
            current_pos[1], goal_point[1], self.number_of_sampling_points
        )
        path_z = np.linspace(
            current_pos[2], goal_point[2], self.number_of_sampling_points
        )

        path = path_x[np.newaxis]
        path = np.vstack((path, path_y))
        path = np.vstack((path, path_z))
        self.trajectory_controller(path)

    def trajectory_controller(self, path):
        count = 0
        previous_index = 0
        path_x = path[0, :]
        path_y = path[1, :]
        path_z = path[2, :]
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    "/base_link", "/arm_link_5", rospy.Time(0)
                )
                break
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue
        current_pos = np.array([trans[0], trans[1], trans[2]])

        distance = np.linalg.norm((np.array(path[:, path.shape[1] - 1]) - current_pos))
        print "final pos is ", path[:, path.shape[1] - 1]
        while (
            distance > self.goal_tolerance
            and self.event != "e_stop"
            and not rospy.is_shutdown()
        ):
            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    "/base_link", "/arm_link_5", rospy.Time(0)
                )
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue
            message = TwistStamped()
            current_pos = np.array([trans[0], trans[1], trans[2]])
            distance = np.linalg.norm(
                (np.array(path[:, path.shape[1] - 1]) - current_pos)
            )
            dist = []
            for i in range(path.shape[1]):
                dist.append(np.linalg.norm((path[:, i] - current_pos)))
            index = np.argmin(dist)

            if index < previous_index:
                index = previous_index
            else:
                previous_index = index

            # Delete this block later
            if index > path.shape[1] - 1:
                break

            # Check if path repeated points in it, this will prevent trajectory execution as velocity between these two points will
            # be zero (difference is zero)

            if index == path.shape[1] - 1:
                ind = index
            else:
                ind = index + 1
            # print index, ind, path.shape[1]

            vel_x = self.feedforward_gain * (
                path_x[ind] - path_x[index]
            ) + self.feedback_gain * (path_x[ind] - current_pos[0])
            vel_y = self.feedforward_gain * (
                path_y[ind] - path_y[index]
            ) + self.feedback_gain * (path_y[ind] - current_pos[1])
            vel_z = self.feedforward_gain * (
                path_z[ind] - path_z[index]
            ) + self.feedback_gain * (path_z[ind] - current_pos[2])

            message.header.seq = count
            message.header.frame_id = "/base_link"
            message.twist.linear.x = vel_x
            message.twist.linear.y = vel_y
            message.twist.linear.z = vel_z
            self.vel_publisher.publish(message)
            count += 1

        message = TwistStamped()
        message.header.seq = count
        message.header.frame_id = "/base_link"
        message.twist.linear.x = 0.0
        message.twist.linear.y = 0.0
        message.twist.linear.z = 0.0
        self.vel_publisher.publish(message)

    def execute(self):
        print "executing ............."
        self.move_arm_to_goal()


if __name__ == "__main__":
    rospy.init_node("mir_cartesian_velocity_insert")
    OBJ = cartesian_velocity_insert()
    OBJ.start()
