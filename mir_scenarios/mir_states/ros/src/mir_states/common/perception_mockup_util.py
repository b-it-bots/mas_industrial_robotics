#!/usr/bin/python

import rospy
from mcr_perception_msgs.msg import ObjectList, Object

import smach

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
    
    
class add_object_from_task_list_state(smach.State):
     
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['success'],
            input_keys=['task_list'])
        rospy.loginfo("object_detection_task_list_observer_state initialized")
         
         
    def execute(self, userdata):
        rospy.loginfo("object_detection_task_list_observer_state executed")
         
        object_list = []
        
        for task in userdata.task_list:
            if (task.type == "source"):
                for object_names in task.object_names:
                    obj = Object()
                    obj.name = object_names
                    obj.pose.header.frame_id = "/base_link"
                    obj.pose.pose.position.x = 0.61
                    object_list.append(obj)
                 
        set_object_list(object_list)
         
        return 'success'
 
class remove_object_to_grasp_state(smach.State):
     
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['success'],
            input_keys=['object_to_grasp'])
        rospy.loginfo("object_detection_grasp_object_observer_state initialized")
         
         
    def execute(self, userdata):
        rospy.loginfo("object_detection_grasp_object_observer_state executed")
         
        remove_object(userdata.object_to_grasp)
 
        return 'success'   
