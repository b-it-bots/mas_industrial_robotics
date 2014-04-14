import re
import rospy
import smach
import sys
import zmq

import mir_states_common.robocup.task
import mir_states_common.robocup.referee_box_communication

#from tasks import parse_task, TaskSpecFormatError

try:
    ip = rospy.get_param('refbox_ip')
except KeyError:
    rospy.logerr("Using Hardcoded refbox_ip")
    ip = "192.168.13.109"
    port = "11111"
    team_name = "b-it-bots"


class Bunch:
    def __init__(self, **kwds):
        self.__dict__.update(kwds)


class get_task(smach.State):

    """
    Communicate with the RefereeBox and get a task description.

    Input
    -----
    test: str
        Type of test ('BNT', 'BTT', etc). The task specification will be parsed
        according to this type.
    simulation: bool
        If this is set to True, then no communication with Referee box will
        happen and rather a hard-coded task specification will be used.

    Output
    -----
    task: Task
        An appropriate subclass of Task with fields filled according to the
        received specification.
    """

    HARDCODED_SPECS = {'BNT': 'BNT<(D,W,3),(S1,E,3),(T3,N,3),(S3,S,3),(T1,S,3),(D1,E,3),(S4,N,3),(S5,N,3),(T4,W,3),(T2,S,3),(S2,E,3),(EXIT,E,3)>',
                       'BMT': 'BMT<D1,D1,D1,line(F20_20_B,R20,M20_100),D1>',
                	   'BTT': 'BTT<initialsituation(<S1,(M20_100,S40_40_G)><S2,(F20_20_G,S40_40_B,F20_20_B)>);goalsituation(<S3,line(S40_40_G,F20_20_G)><D1,zigzag(F20_20_B,S40_40_B,M20_100)>)>',
                       'PPT': 'PPT<S6,S5>'}

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['task_received', 'wrong_task_format'], 
            input_keys=['test', 'simulation'],
            io_keys=['task', 'task_spec_copy'])
        #outcomes=['task_received', 'wrong_task_format', 'test_not_set'],

    def execute(self, userdata):
        #if len(userdata.test) <= 0:
        #    rospy.logerr("userdata.test NOT set. Please specifiy in the test state machine")
        #    return 'test_not_set'

        if not userdata.simulation:
            rospy.logdebug('Waiting for task specification (%s:%s)...' % (ip, port))
            task_spec = mir_states_common.robocup.referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)
        else:
            task_spec = self.HARDCODED_SPECS[userdata.test]
        rospy.loginfo("Task specification: %s" % task_spec)
        userdata.task_spec_copy = task_spec
        try:
            userdata.task = mir_states_common.robocup.parse_task(userdata.test, task_spec)
            rospy.loginfo('Parsed task:\n%s' % userdata.task)
            return 'task_received'
        except mir_states_common.robocup.task.TaskSpecFormatError:
            return 'wrong_task_format'

class re_get_task(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['task_received', 'wrong_task_format'], 
            input_keys=['test', 'simulation'],
            io_keys=['task','task_spec_copy'])
        #outcomes=['task_received', 'wrong_task_format', 'test_not_set'],

    def execute(self, userdata):
        #if len(userdata.test) <= 0:
        #    rospy.logerr("userdata.test NOT set. Please specifiy in the test state machine")
        #    return 'test_not_set'
        task_spec = userdata.task_spec_copy        
        try:
            userdata.task = mir_states_common.robocup.parse_task(userdata.test, task_spec)
            rospy.loginfo('Parsed task:\n%s' % userdata.task)
            return 'task_received'
        except mir_states_common.robocup.task.TaskSpecFormatError:
            return 'wrong_task_format'

#FIXME: merge into get_task
class get_basic_manipulation_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wrong_task_format'], input_keys=['task_list'], output_keys=['task_list'])
        
    def execute(self, userdata):

        rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")
       
        #man_task = "BMT<S5,S5,S4,line(S40_40_B,F20_20_B),S4>"
        man_task = mir_states_common.robocup.referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)  #'BNT<(D1,N,6),

        rospy.loginfo("Task received: " + man_task)
        
        # check if Task is a BMT task      
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
        
        for obj in range(len(obj_names)):
            if obj_names[obj] == "V20":
                obj_names[obj] = "R20"


        print obj_names
        
        # which object to get from the source location
        source_tasklist = Bunch(type = 'source', location = src_pose, object_names = list(obj_names)) 
        userdata.task_list.append(source_tasklist)        

        # where to deliver the objects and in which configuration
        destination_tasklist = Bunch(type = 'destination', location = dest_pose, object_names = list(obj_names),  object_config = obj_cfg)
        userdata.task_list.append(destination_tasklist) 

        print "PARSED TASK: "
        print "-----------------------------------------------------"
        for task in userdata.task_list:
          print "type:", task.type
          print "   location:", task.location
          print "   objects:", task.object_names  
          if task.type == "destination":
            print "   objects:", task.object_config       
          print "-----------------------------------------------------"

        return 'task_received' 

#FIXME: merge into get_task
class get_precision_placement_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wrong_task_format'], input_keys=['task_list', 'simulation'], output_keys=['task_list'])
        
    def execute(self, userdata):

        
        if(userdata.simulation):
            rospy.loginfo("Task spec by hard coded string")

            precision_task = 'PTT<S2,(M20,F20_20_B,F20_20_G,R20,V20),S1>'

        else:
            rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")
            precision_task = mir_states_common.robocup.referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name) 

        rospy.loginfo("Task received: " + precision_task)

        ptt_string = precision_task

        objects = re.findall("\(.*?\)", ptt_string)
        objects = objects[0] 
        #print objects

        source = re.findall("<.*?,",ptt_string)[0]
        #print source

        destination = re.findall(".?.?>",ptt_string)[0]
        destination = re.match(".\w",destination).group()
        #print destination
        #destination = destination.group()

        result = "BTT<initialsituation(" + source
        result = result + objects + ">);"
        result = result + "goalsituation(<" + destination + ","
        result = result + "line" + objects + ">)>"

        transportation_task = result
        
        # check if Task is a BTT task      
        if(transportation_task[0:3] != "BTT"):
           rospy.logerr("Excepted <<BTT>> task, but received <<" + transportation_task[0:3] + ">> received")
           return 'wrong_task_format' 

        # remove leading start description        
        transportation_task = transportation_task[3:len(transportation_task)]
        
        
        # check if description has beginning '<' and ending '>
        if(transportation_task[0] != "<" or transportation_task[(len(transportation_task)-1)] != ">"):
            rospy.loginfo("task spec not in correct format")
            return 'wrong_task_format' 
        
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
           return 'wrong_task_format' 

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
           return 'wrong_task_format' 

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

            obj_conf = obj_taskspec.split('(')
            obj_conf = obj_conf[0]

            rospy.loginfo("    %s %s %s", desired_loc, obj_conf, objs)

            
            goal_tasklist = Bunch(type='destination', location = desired_loc, object_names = objs,  object_config = obj_conf)
            userdata.task_list.append(goal_tasklist)    

        print "PARSED TASK: "
        print "-----------------------------------------------------"
        for task in userdata.task_list:
          print "type:", task.type
          print "   location:", task.location
          print "   objects:", task.object_names  
          if task.type == "destination":
            print "   objects:", task.object_config       
          print "-----------------------------------------------------"

    
        return 'task_received'   

class get_basic_transportation_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wrong_task_format'], input_keys=['task_list', 'simulation'], output_keys=['task_list'])
        
    def execute(self, userdata):

        
        if(userdata.simulation):
            rospy.loginfo("Task spec by hard coded string")

            #transportation_task = 'BTT<initialsituation(<S1,(M20_100,S40_40_G)><S2,(F20_20_G,S40_40_B,F20_20_B)>);goalsituation(<S3,line(S40_40_G,F20_20_G)><D1,zigzag(F20_20_B,S40_40_B,M20_100)>)>'
            transportation_task = 'BTT<initialsituation(<S3,(F20_20_B,M20_100)>);goalsituation(<S2,zigzag(M20_100,F20_20_B)>)>'
        else:
            rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")
            transportation_task = mir_states_common.robocup.referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name) 

        rospy.loginfo("Task received: " + transportation_task)
        
        # check if Task is a BTT task      
        if(transportation_task[0:3] != "BTT"):
           rospy.logerr("Excepted <<BTT>> task, but received <<" + transportation_task[0:3] + ">> received")
           return 'wrong_task_format' 

        # remove leading start description        
        transportation_task = transportation_task[3:len(transportation_task)]
        
        
        # check if description has beginning '<' and ending '>
        if(transportation_task[0] != "<" or transportation_task[(len(transportation_task)-1)] != ">"):
            rospy.loginfo("task spec not in correct format")
            return 'wrong_task_format' 
        
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
           return 'wrong_task_format' 

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
           return 'wrong_task_format' 

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

            obj_conf = obj_taskspec.split('(')
            obj_conf = obj_conf[0]

            rospy.loginfo("    %s %s %s", desired_loc, obj_conf, objs)

            
            goal_tasklist = Bunch(type='destination', location = desired_loc, object_names = objs,  object_config = obj_conf)
            userdata.task_list.append(goal_tasklist)        
        return 'task_received'   
    
class get_basic_competitive_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wrong_task_format'], input_keys=['task_list'], output_keys=['task_list'])
        
    def execute(self, userdata):

        rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")
        competitive_task = mir_states_common.robocup.referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)  #'BTT<(D1,N,6),(S2,E,3)>'
        rospy.loginfo("Task received: " + competitive_task)
        
        # check if Task is a CTT task      
        if(competitive_task[0:3] != "CTT"):
           rospy.logerr("Excepted <<BTT>> task, but received <<" + competitive_task[0:3] + ">> received")
           return 'wrong_task_format' 

        # remove leading start description        
        competitive_task = competitive_task[3:len(competitive_task)]
        
        
        # check if description has beginning '<' and ending '>
        if(competitive_task[0] != "<" or competitive_task[(len(competitive_task)-1)] != ">"):
            rospy.loginfo("task spec not in correct format")
            return 'wrong_task_format' 
        
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
           return 'wrong_task_format' 

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
           return 'wrong_task_format' 

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

