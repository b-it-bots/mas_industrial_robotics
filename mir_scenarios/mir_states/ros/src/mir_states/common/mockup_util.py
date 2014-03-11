#!/usr/bin/python

import rospy
from mcr_perception_msgs.msg import ObjectList, Object

def set_object_list(object_list, topic="/mcr_perception/object_detector/set_object_to_mockup"):
    pub = rospy.Publisher(topic, ObjectList)
    rospy.sleep(0.25)
    pub.publish(object_list)
    
def remove_object(object, topic="/mcr_perception/object_detector/rm_object_from_mockup"):
    pub = rospy.Publisher(topic, Object)
    rospy.sleep(0.25)
    pub.publish(object)

def add_object(object, topic="/mcr_perception/object_detector/rm_object_from_mockup"):
    pub = rospy.Publisher(topic, Object)
    rospy.sleep(0.25)
    pub.publish(object)