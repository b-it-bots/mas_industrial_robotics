#! /usr/bin/env python
"""
This is a test script which publishes one pose stamped message to test moveit_to_pose
node
"""

from __future__ import print_function

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion

DONE = False

def event_out_cb(msg):
    global DONE
    print(msg)
    DONE = True

def main():
    rospy.init_node('moveit_to_pose_test')

    # create a pose object
    candle_pose = PoseStamped()
    candle_pose.header.stamp = rospy.Time.now()

    candle_pose.header.frame_id = '/base_footprint'
    candle_pose.pose.position.x = 0.27
    candle_pose.pose.position.z = 0.7525
    # candle_pose.header.frame_id = '/arm_link_1'
    # candle_pose.pose.position.x = 0.03
    # candle_pose.pose.position.z = 0.34

    candle_pose.pose.position.y = 0.0
    rpy = (0.0, 0.0, 0.0)
    candle_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(*rpy))

    # create neccessary pubs and subs
    pose_pub = rospy.Publisher('/moveit_to_pose/pose_in',
                                PoseStamped,
                                queue_size=1)
    event_in_pub = rospy.Publisher('/moveit_to_pose/event_in',
                                    String,
                                    queue_size=1)

    event_out_sub = rospy.Subscriber('/moveit_to_pose/event_out',
                                     String,
                                     event_out_cb)
    rospy.sleep(1)
    print(candle_pose)

    pose_pub.publish(candle_pose)
    rospy.sleep(1)
    event_in_pub.publish(String(data='e_trigger'))
    while not DONE and not rospy.is_shutdown():
        rospy.sleep(0.5)

if __name__ == "__main__":
    main()
