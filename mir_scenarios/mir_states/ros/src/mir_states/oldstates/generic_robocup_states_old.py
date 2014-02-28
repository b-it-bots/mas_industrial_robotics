#!/usr/bin/python
import roslib
roslib.load_manifest('mir_states')
pkg_dir = roslib.packages.get_pkg_dir('mir_states')

import sys
sys.path.append(pkg_dir + '/include')

import rospy
import smach
import smach_ros
import referee_box_communication
import re
import task_converter

try:
    ip = rospy.get_param('refbox_ip')
except KeyError:
    rospy.logerr("Using Hardcoded refbox_ip")
    ip = "192.168.142.1"
port = "11111"
team_name = "b-it-bots"

class Bunch:
    def __init__(self, **kwds):
         self.__dict__.update(kwds)

class get_basic_navigation_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wront_task_format'], input_keys=['task_list'], output_keys=['task_list'])
        
    def execute(self, userdata):

        #rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")

        #nav_task = 'BNT<(T2,E,1),(D2,N,1),(S3,E,3),(T3,S,3),(T7,N,3),(S1,W,3),(T2,S,1),(D1,W,1),(T1,N,1),(S1,E,1)>'
        nav_task = 'BNT<(D1,W,1),(S1,E,3),(S2,E,3),(D2,S,3),(S3,W,3),(S2,W,3),(D2,W,3),(S1,W,3),(S2,W,3),(S3,W,3),(S2,W,3),(D1,W,3)>'
        		        
        #nav_task = referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)  #'BNT<(D1,N,6),(S2,E,3)>'
        rospy.loginfo("Task received: " + nav_task)
        
        # check if Task is a BNT task      
        if(nav_task[0:3] != "BNT"):
           rospy.logerr("Excepted <<BNT>> task, but received <<" + nav_task[0:3] + ">> received")
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


class get_basic_manipulation_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wront_task_format'], input_keys=['task_spec'], output_keys=['task_spec'])
        
    def execute(self, userdata):

        rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")
       
        #man_task = "BMT<D2,D2,D2,line(R20,M20_100,F20_20_B),T4>"
        man_task = referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)  #'BNT<(D1,N,6),

        

        rospy.loginfo("Task received: " + man_task)
        
        # check if Task is a BNT task      
        if(man_task[0:3] != "BMT"):
           rospy.logerr("Excepted <<BMT>> task, but received <<" + man_task[0:3] + ">> received")
           return 'wront_task_format' 

        # remove leading start description        
        man_task = man_task[3:len(man_task)]
        
        # check if description has beginning '<' and ending '>
        if(man_task[0] != "<" or man_task[(len(man_task)-1)] != ">"):
            rospy.loginfo("task spec not in correct format")
            return 'wront_task_format' 
        
        
        # remove beginning '<' and ending '>'
        man_task = man_task[1:len(man_task)-1]
        
        #print man_task
        
        task_list = man_task.split(',')
        
        #print task_list

        init_pose = task_list[0]
        src_pose = task_list[1]
        dest_pose = task_list[2]
        
        subtask_list = task_list[3].split('(')
        obj_cfg = subtask_list[0]
        
        obj_names = []
        obj_names.append(subtask_list[1])
   
        print task_list
        
        for i in range(4, (len(task_list)-1)):
            if i == (len(task_list)-2):
                print task_list[i]
                task_list[i] = task_list[i][0:(len(task_list[i])-1)]
                print task_list[i]
                
                 
            obj_names.append(task_list[i])
               
        
        fnl_pose = task_list[len(task_list)-1]
        
        '''
        print init_pose
        print src_pose
        print dest_pose
        print obj_cfg
        print obj_names
        print fnl_pose
        '''
        for obj in range(len(obj_names)):
            if obj_names[obj] == "V20":
                obj_names[obj] = "R20"


        print obj_names
        
        
        userdata.task_spec = Bunch(inital_pose=init_pose, source_pose=src_pose, destination_pose=dest_pose, object_config=obj_cfg, 
                            object_names=obj_names, final_pose=fnl_pose)
        
        return 'task_received'  

class get_basic_manipulation_task_modified(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wront_task_format'],  input_keys=['task_list'], output_keys=['task_list','final_pose'])
        
    def execute(self, userdata):

        rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")
        
        #referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)  #"BMT<S1,S1,S2,line(nut,screw,bolt),S1>"

        man_task = "BMT<S1,S1,S2,line(nut,screw,bolt),S1>"

        rospy.loginfo("Task received: " + man_task)
        
        # check if Task is a BNT task      
        if(man_task[0:3] != "BMT"):
           rospy.logerr("Excepted <<BMT>> task, but received <<" + man_task[0:3] + ">> received")
           return 'wront_task_format' 

        # remove leading start description        
        man_task = man_task[3:len(man_task)]
        
        # check if description has beginning '<' and ending '>
        if(man_task[0] != "<" or man_task[(len(man_task)-1)] != ">"):
            rospy.loginfo("task spec not in correct format")
            return 'wront_task_format' 
        
        
        # remove beginning '<' and ending '>'
        man_task = man_task[1:len(man_task)-1]
        
        #print man_task
        
        task = man_task.split(',')
        
        subtask_list = task[3].split('(')
        obj_cfg = subtask_list[0]
        
        objs = []
        objs.append(subtask_list[1])
        
        for i in range(4, (len(task)-1)):
            if i == (len(task)-2):
                task[i] = task[i][0:(len(task)-3)]
                 
            objs.append(task[i])
               
        
        fnl_pose = task[len(task)-1]
        

        
        initial_tasklist = Bunch(type='source', location = task[1], object_names=objs) 
        userdata.task_list.append(initial_tasklist)

        goal_tasklist = Bunch(type='destination', location = task[2], object_names = objs,  object_config = obj_cfg)
        userdata.task_list.append(goal_tasklist)

        userdata.final_pose = fnl_pose
        
        return 'task_received'  

class get_basic_transportation_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wront_task_format'], input_keys=['task_list'], output_keys=['task_list'])
        
    def execute(self, userdata):

        rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")

        #transportation_task = 'BTT<initialsituation(<S5,line(R20,F20_20_B,F20_20_G)>);goalsituation(<S6,line(R20,F20_20_B,F20_20_G)>)>'

        #transportation_task = 'BTT<initialsituation(<S1,line(R20)><S5,line(F20_20_B)>);goalsituation(<D1,line(R20,F20_20_B)>)>'
        transportation_task = referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name, True) 

        rospy.loginfo("Task received: " + transportation_task)
        
        # check if Task is a BTT task 
        if(transportation_task[0:3] == "PPT"):
            transportation_task = task_converter.ppt2btt(transportation_task)    
        elif(transportation_task[0:3] != "BTT"):
           rospy.logerr("Excepted <<BTT>> task, but received <<" + transportation_task[0:3] + ">> received")
           return 'wront_task_format' 

        # remove leading start description        
        transportation_task = transportation_task[3:len(transportation_task)]
        
        
        # check if description has beginning '<' and ending '>
        if(transportation_task[0] != "<" or transportation_task[(len(transportation_task)-1)] != ">"):
            rospy.loginfo("task spec not in correct format")
            return 'wront_task_format' 
        
        # remove beginning '<' and ending '>'
        transportation_task = transportation_task[1:len(transportation_task)-1]

        # Task split
        task_situation = transportation_task.split(';')
        rospy.loginfo("split1: %s",task_situation)  
        

        # Initial Situation
        initial_situation = task_situation[0]
        rospy.loginfo("init: %s", initial_situation)        

        if(initial_situation[0:16] != "initialsituation"):
           rospy.logerr("Excepted <<initialsituation>>, but received <<" + initial_situation[0:16] + ">> received")
           return 'wront_task_format' 

        initial_situation = initial_situation[16:len(initial_situation)]
        rospy.loginfo('removed <> and (): %s',initial_situation)

        init_tasks = re.findall('\<(?P<name>.*?)\>', initial_situation)
        rospy.loginfo("split into poses: %s",init_tasks)

        # Update userdata with expcted initial situation information
        for item in init_tasks:            
            desired_loc = item[0:2]

            obj_taskspec = item[3:len(item)]

            objs = re.findall('\((?P<name>.*?)\)', obj_taskspec)
            objs = objs[0].split(',')

            for i in range(len(objs)):
                if objs[i] == "V20":
                    objs[i] = "R20"
                if objs[i] == "M20":
                    objs[i] = "R20"

            obj_conf = obj_taskspec.split('(')
            obj_conf = obj_conf[0]

            rospy.loginfo("    %s %s", desired_loc, objs)
  
            initial_tasklist = Bunch(type='source', location = desired_loc, object_names=objs) 
            userdata.task_list.append(initial_tasklist)

        
        # Goal Situation
        goal_situation = task_situation[1]
        rospy.loginfo('goal %s', goal_situation)    

        if(goal_situation[0:13] != "goalsituation"):
           rospy.logerr("Excepted <<goalsituation>>, but received <<" + goal_situation[0:13] + ">> received")
           return 'wront_task_format' 

        goal_situation = goal_situation[13:len(goal_situation)]
        rospy.loginfo('removed goal string: %s', goal_situation)

        goal_tasks = re.findall('\<(?P<name>.*?)\>', goal_situation)
        rospy.loginfo('split into locations: %s', goal_tasks)

        # Update userdata with expcted goal situation information
        for item in goal_tasks:
            desired_loc = item[0:2]
            
            obj_taskspec = item[3:len(item)]

            objs = re.findall('\((?P<name>.*?)\)', obj_taskspec)
            objs = objs[0].split(',')

            for i in range(len(objs)):
                if objs[i] == "V20":
                    objs[i] = "R20"
                if objs[i] == "M20":
                    objs[i] = "R20"

            obj_conf = obj_taskspec.split('(')
            obj_conf = obj_conf[0]

            rospy.loginfo("    %s %s %s", desired_loc, obj_conf, objs)

            
            goal_tasklist = Bunch(type='destination', location = desired_loc, object_names = objs,  object_config = obj_conf)
            userdata.task_list.append(goal_tasklist)        
        return 'task_received'   
    
class get_basic_competitive_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wront_task_format'], input_keys=['task_list'], output_keys=['task_list'])
        
    def execute(self, userdata):

        rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")
        competitive_task = referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)  #'BTT<(D1,N,6),(S2,E,3)>'
        rospy.loginfo("Task received: " + competitive_task)
        
        # check if Task is a CTT task      
        if(competitive_task[0:3] != "CTT"):
           rospy.logerr("Excepted <<BTT>> task, but received <<" + competitive_task[0:3] + ">> received")
           return 'wront_task_format' 

        # remove leading start description        
        competitive_task = competitive_task[3:len(competitive_task)]
        
        
        # check if description has beginning '<' and ending '>
        if(competitive_task[0] != "<" or competitive_task[(len(competitive_task)-1)] != ">"):
            rospy.loginfo("task spec not in correct format")
            return 'wront_task_format' 
        
        # remove beginning '<' and ending '>'
        competitive_task = competitive_task[1:len(competitive_task)-1]

        # Task split
        task_situation = competitive_task.split(';')
        rospy.loginfo(task_situation)  
        

        # Initial Situation
        initial_situation = task_situation[0]
        rospy.loginfo(initial_situation)        

        if(initial_situation[0:16] != "initialsituation"):
           rospy.logerr("Excepted <<initialsituation>>, but received <<" + initial_situation[0:16] + ">> received")
           return 'wront_task_format' 

        initial_situation = initial_situation[16:len(initial_situation)]
        rospy.loginfo(initial_situation)

        init_tasks = re.findall('\<(?P<name>.*?)\>', initial_situation)
        rospy.loginfo(init_tasks)

        # Update userdata with expcted initial situation information
        for item in init_tasks:            
            desired_loc = item[0:2]

            if item[0] == "D":
                base_orientation = "W"
            elif item[0] == "S":
                base_orientation = "E"

            obj_taskspec = item[3:len(item)]

            objs = re.findall('\((?P<name>.*?)\)', obj_taskspec)
            objs = objs[0].split(',')

            obj_conf = obj_taskspec.split('(')
            obj_conf = obj_conf[0]

  
            initial_tasklist = Bunch(location=desired_loc,orientation=base_orientation,task='fetch object workspace',object_names=objs,object_config=obj_conf) 
            userdata.task_list.append(initial_tasklist)

        
        # Goal Situation
        goal_situation = task_situation[1]
        rospy.loginfo(goal_situation)    

        if(goal_situation[0:13] != "goalsituation"):
           rospy.logerr("Excepted <<goalsituation>>, but received <<" + goal_situation[0:13] + ">> received")
           return 'wront_task_format' 

        rospy.loginfo(goal_situation)
        goal_situation = goal_situation[13:len(goal_situation)]
        rospy.loginfo(goal_situation)

        goal_tasks = re.findall('\<(?P<name>.*?)\>', goal_situation)
        rospy.loginfo(goal_tasks)

        # Update userdata with expcted goal situation information
        for item in goal_tasks:
            desired_loc = item[0:2]
            if item[0] == "D":
                base_orientation = "W"
            elif item[0] == "S":
                base_orientation = "E"
            
            obj_taskspec = item[3:len(item)]

            objs = re.findall('\((?P<name>.*?)\)', obj_taskspec)
            objs = objs[0].split(',')
            rospy.loginfo(objs)

            obj_conf = obj_taskspec.split('(')
            obj_conf = obj_conf[0]
            rospy.loginfo(obj_conf)

            
            goal_tasklist = Bunch(location=desired_loc,orientation=base_orientation,task='place object in workspace',object_names = objs, object_config = obj_conf)
            userdata.task_list.append(goal_tasklist)

        
        return 'task_received'   

        
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
     
