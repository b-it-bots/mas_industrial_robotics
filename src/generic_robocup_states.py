#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
pkg_dir = roslib.packages.get_pkg_dir('raw_generic_states')

import sys
sys.path.append(pkg_dir + '/include')

import rospy
import smach
import smach_ros
import referee_box_communication
import re

class Bunch:
    def __init__(self, **kwds):
         self.__dict__.update(kwds)

class get_basic_navigation_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wront_task_format'], input_keys=['task_list'], output_keys=['task_list'])
        
    def execute(self, userdata):
        ip = "10.20.121.62"
        port = "11111"
        team_name = "b-it-bots"

        rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")
        nav_task = referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)  #'BNT<(D1,N,6),(S2,E,3)>'
        rospy.loginfo("Task received: " + nav_task)
        
        # check if Task is a BNT task      
        if(nav_task[0:3] != "BNT"):
           rospy.logerr("Excepted <<BNT>> task, but received <<" + nav_task[0:2] + ">> received")
           return 'wront_task_format' 

        # remove leading start description        
        nav_task = nav_task[3:len(nav_task)]
        
        
        # check if description has beginning '<' and ending '>
        if(nav_task[0] != "<" or nav_task[(len(nav_task)-1)] != ">"):
            rospy.loginfo("task spec not in correct format")
            return 'wront_task_format' 
        
        # remove beginning '<' and ending '>'
        nav_task = nav_task[1:len(nav_task)-1]
        
        #find single tasks
        task_list = re.findall('\((?P<name>.*?)\)', nav_task)
        
        #put them into a struct like structure
        for item in task_list:
            task_items = item.split(',')
            
            if len(task_items) != 3:
                rospy.loginfo("task spec not in correct format")
                return 'wront_task_format' 
            
            task_struct = Bunch(location=task_items[0], orientation=task_items[1], duration=task_items[2])
            userdata.task_list.append(task_struct)
        
        return 'task_received'  
    
class select_pose_to_approach(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['pose_selected', 'location_not_known'], 
                                    input_keys=['task_list', 'current_task_index', 'base_pose_to_approach'], 
                                    output_keys=['base_pose_to_approach'])
        
    def execute(self, userdata):
             
        userdata.base_pose_to_approach = userdata.task_list[userdata.current_task_index].location        
        
        #get location position
        if (not rospy.has_param("script_server/base/" + userdata.base_pose_to_approach)):
            rospy.logerr("location <<" + userdata.base_pose_to_approach + ">> is not on the parameter server")
            return 'location_not_known'
        
        position = rospy.get_param("script_server/base/" + userdata.base_pose_to_approach)
        
        #get orientation angle
        if (not rospy.has_param("script_server/base_orientations/" + userdata.task_list[userdata.current_task_index].orientation)):
            rospy.logerr("orientation <<" + userdata.task_list[userdata.current_task_index].orientation + ">> is not on the parameter server")
            return 'location_not_known'
        
        orientation = rospy.get_param("script_server/base_orientations/" + userdata.task_list[userdata.current_task_index].orientation)
        
        #establish new pose
        new_pose = [0]*3
        new_pose[0] = position[0]
        new_pose[1] = position[1]
        new_pose[2] = orientation
               
        #set parameter
        rospy.set_param("script_server/base/" + userdata.base_pose_to_approach, new_pose)
        
        rospy.loginfo('selected pose: ' + userdata.base_pose_to_approach + " with orientation: " + userdata.task_list[userdata.current_task_index].orientation)
        return 'pose_selected'
    
class wait_for_desired_duration(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], 
                                    input_keys=['task_list', 'current_task_index'])
        
    def execute(self, userdata):
        
        sleep_duration = userdata.task_list[userdata.current_task_index].duration
        rospy.loginfo('wait desired duration of ' + sleep_duration + " seconds")
        rospy.sleep(int(sleep_duration))
        rospy.loginfo('wait done')
        
        return 'succeeded'
    
    
class increment_task_index(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_tasks'], 
                                    input_keys=['task_list', 'current_task_index'],
                                    output_keys=['current_task_index'])
        
    def execute(self, userdata):
        
        # inc index
        userdata.current_task_index = userdata.current_task_index + 1
        
        # check if index is larger than items in task list
        if userdata.current_task_index >= len(userdata.task_list):
            rospy.loginfo("no more tasks in task list")
            return 'no_more_tasks'
        
        return 'succeeded'
    
    
    
