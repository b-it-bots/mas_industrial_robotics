#!/usr/bin/env python
"""
This module contains a component that publishes an artificial object pose.

"""
# -*- encoding: utf-8 -*-

import math
import threading
import Tkinter
import rospy
import tf
import geometry_msgs.msg
import visualization_msgs.msg

LINEAR_RESOLUTION = 0.005   # in meters
MAX_POSITION_X = 0.9        # in meters
MIN_POSITION_X = -0.9        # in meters
MAX_POSITION_Y = 0.3        # in meters
MIN_POSITION_Y = -0.3       # in meters
MAX_POSITION_Z = -0.3        # in meters
MIN_POSITION_Z = 0.6        # in meters
ANGULAR_RESOLUTION = 1      # in degrees
MIN_ORIENTATION = 0         # in degrees
MAX_ORIENTATION = 359       # in degrees

TRANSPARENCY = 0.5

pose_marker = visualization_msgs.msg.Marker()
pose = geometry_msgs.msg.PoseStamped()

global roll_value
roll_value = 0.0
global pitch_value
pitch_value = 0.0
global yaw_value
yaw_value = 0.0

global lock
lock = threading.Lock()


def create_window():
    """
    Creates a GUI window to publish a pose.

    """
    master = Tkinter.Tk()

    # pose's position
    linear_scale_x = Tkinter.Scale(
        master, command=position_x, from_=MIN_POSITION_X, to=MAX_POSITION_X,
        resolution=LINEAR_RESOLUTION, label="Position X"
    )
    linear_scale_x.grid(row=0, column=0)

    linear_scale_y = Tkinter.Scale(
        master, command=position_y, from_=MIN_POSITION_Y, to=MAX_POSITION_Y,
        resolution=LINEAR_RESOLUTION, label="Position Y"
    )
    linear_scale_y.grid(row=0, column=1)

    linear_scale_z = Tkinter.Scale(
        master, command=position_z, from_=MIN_POSITION_Z, to=MAX_POSITION_Z,
        resolution=LINEAR_RESOLUTION, label="Position Z"
    )
    linear_scale_z.grid(row=0, column=2)

    # pose's orientation
    roll = Tkinter.Scale(
        master, command=orientation_roll, from_=MIN_ORIENTATION, to=MAX_ORIENTATION,
        resolution=ANGULAR_RESOLUTION, label="Roll"
    )
    roll.grid(row=1, column=0)

    pitch = Tkinter.Scale(
        master, command=orientation_pitch, from_=MIN_ORIENTATION, to=MAX_ORIENTATION,
        resolution=ANGULAR_RESOLUTION, label="Pitch"
    )
    pitch.grid(row=1, column=1)

    yaw = Tkinter.Scale(
        master, command=orientation_yaw, from_=MIN_ORIENTATION, to=MAX_ORIENTATION,
        resolution=ANGULAR_RESOLUTION, label="Yaw"
    )
    yaw.grid(row=1, column=2)

    master.title("Pose mock-up")
    master.mainloop()
    rospy.signal_shutdown("GUI closed")


def position_x(slider):
    """
    Sets slider as the target's position in the X axis.

    """
    global lock
    lock.acquire()
    pose.pose.position.x = float(slider)
    pose_marker.pose.position.x = float(slider)
    lock.release()


def position_y(slider):
    """
    Sets slider as the target's position in the Y axis.

    """
    global lock
    lock.acquire()
    pose.pose.position.y = float(slider)
    pose_marker.pose.position.y = float(slider)
    lock.release()


def position_z(slider):
    """
    Sets slider as the target's position in the Z axis.

    """
    global lock
    lock.acquire()
    pose.pose.position.z = float(slider)
    pose_marker.pose.position.z = float(slider)
    lock.release()


def orientation_roll(slider):
    """
    Sets slider as the target's orientation about the X axis.

    """
    global lock
    lock.acquire()
    global roll_value
    roll_value = math.radians(float(slider))
    lock.release()


def orientation_pitch(slider):
    """
    Sets slider as the target's orientation about the Y axis.

    """
    global lock
    lock.acquire()
    global pitch_value
    pitch_value = math.radians(float(slider))
    lock.release()


def orientation_yaw(slider):
    """
    Sets slider as the target's orientation about the Z axis.

    """
    global lock
    lock.acquire()
    global yaw_value
    yaw_value = math.radians(float(slider))
    lock.release()


def publish_pose():
    """
    Publishes the target pose.

    """
    # node cycle rate (in hz)
    loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

    # the transparency of the object to be visualized
    reference_frame = rospy.get_param('~reference_frame', 'base_link')

    # the transparency of the object to be visualized
    transparency = rospy.get_param('~transparency', 0.5)

    # publishers
    pub_pose = rospy.Publisher('~mock_up_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
    pub_pose_marker = rospy.Publisher('~mock_up_pose_marker', visualization_msgs.msg.Marker, queue_size=1)

    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = reference_frame

    pose_marker.header.stamp = rospy.Time.now()
    pose_marker.header.frame_id = reference_frame

    # create a rectangular prism for visualization
    pose_marker.type = 1
    pose_marker.scale.x = 0.05      # in centimeters
    pose_marker.scale.y = 0.02      # in centimeters
    pose_marker.scale.z = 0.01      # in centimeters
    pose_marker.color.r = 0.0
    pose_marker.color.g = 0.0
    pose_marker.color.b = 1.0
    pose_marker.color.a = transparency

    while not rospy.is_shutdown():
        quaternion = tf.transformations.quaternion_from_euler(
            roll_value, pitch_value, yaw_value
        )
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        pose_marker.pose.orientation.x = quaternion[0]
        pose_marker.pose.orientation.y = quaternion[1]
        pose_marker.pose.orientation.z = quaternion[2]
        pose_marker.pose.orientation.w = quaternion[3]

        pub_pose.publish(pose)
        pub_pose_marker.publish(pose_marker)
        loop_rate.sleep()


def main():
    rospy.init_node('target_pose_mock_up')

    import thread
    try:
        thread.start_new_thread(create_window, tuple())

        publish_pose()
    except rospy.ROSInterruptException:
        pass
