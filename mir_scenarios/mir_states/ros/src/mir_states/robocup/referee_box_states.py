import re
import rospy
import smach
import sys
import zmq

import mir_states_common.robocup.referee_box_communication

try:
    ip = rospy.get_param('refbox_ip')
except KeyError:
    rospy.logerr("Using Hardcoded refbox_ip")
    ip = "192.168.25.132"
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

    HARDCODED_SPECS = {'BNT': 'BNT<(C1,W,3),(S1,E,3),(T3,N,3),(S3,S,3),(T1,S,3),(D1,E,3),(S4,N,3),(S5,N,3),(T4,W,3),(T2,S,3),(S2,E,3)>',
                       'BMT': 'BMT<S5,S5,S4,line(S40_40_B,F20_20_B),S4>',
                       'BTT': 'BTT<initialsituation(<S1,(M20_100,S40_40_G)><S2,(F20_20_G,S40_40_B,F20_20_B)>);goalsituation(<S3,line(S40_40_G,F20_20_G)><D1,zigzag(F20_20_B,S40_40_B,M20_100)>)>',
                       'PPT': 'PPT<S6,S5>'}

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['task_received', 'wrong_task', 'wrong_task_format'], 
            input_keys=['test', 'simulation'],
            io_keys=['task_list'])

    def execute(self, userdata):

        if not userdata.simulation:
            rospy.loginfo('Waiting for task specification (%s:%s)...' % (ip, port))
            task_spec = mir_states_common.robocup.referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)
        else:
            rospy.loginfo('Using hardcoded specification')
            task_spec = self.HARDCODED_SPECS[userdata.test]

        rospy.loginfo("Task specification: %s" % task_spec)
    
        test_name = task_spec[0:3]
        if(test_name != userdata.test):
            rospy.logerr("Expected <<" + userdata.test + ">> task, but received <<" + test_name + ">> specification")
            return 'wrong_task'

        # remove leading start description        
        task_spec = task_spec[3:len(task_spec)]

        try:
            if(test_name == "BNT"):
                userdata.task_list = get_basic_navigation_task(task_spec)
            if(test_name == "BMT"):
                userdata.task_list = get_basic_manipulation_task(task_spec)
            elif(test_name == "BTT"):
                userdata.task_list = get_basic_transportation_task(task_spec)
            elif(test_name == "PTT"):
                userdata.task_list = get_precision_placement_task(task_spec)
            elif(test_name == "CBT"):
                userdata.task_list = get_competitive_transportation_task(task_spec)
        except Exception as e:
            rospy.logerr("Exception: %s", e)
            return 'wrong_task_format'

        return 'task_received' 


def get_basic_navigation_task(navigation_task):

    # remove outer task name and '<', '>'. If task name is not correct throws an Exception.
    navigation_task = re.findall(r'<(.*)>', navigation_task)

    if len(navigation_task) != 1:
        raise Exception('Malformed task specification')

    task_list = re.findall(r'\('
                            '(?P<place>.+?),'
                            '(?P<orientation>[NESW]),'
                            '(?P<break>[123])'
                            '\)', navigation_task[0])
    
    return task_list


def get_basic_manipulation_task(man_task):
    task_list = []

    # check if description has beginning '<' and ending '>
    if(man_task[0] != "<" or man_task[(len(man_task)-1)] != ">"):
        raise Exception('Task specification ' + man_task + ' not in correct format')       
    
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
    task_list.append(source_tasklist)        

    # where to deliver the objects and in which configuration
    destination_tasklist = Bunch(type = 'destination', location = dest_pose, object_names = list(obj_names),  object_config = obj_cfg)
    task_list.append(destination_tasklist) 

    print "PARSED TASK: "
    print "-----------------------------------------------------"
    for task in task_list:
      print "type:", task.type
      print "   location:", task.location
      print "   objects:", task.object_names  
      if task.type == "destination":
        print "   objects:", task.object_config       
      print "-----------------------------------------------------"

    return task_list


def get_basic_transportation_task(transportation_task):
    task_list = []

    # check if description has beginning '<' and ending '>
    if(transportation_task[0] != "<" or transportation_task[(len(transportation_task)-1)] != ">"):
        raise Exception("task spec not in correct format")
    
    # remove beginning '<' and ending '>'
    transportation_task = transportation_task[1:len(transportation_task)-1]

    # Task split
    task_situation = transportation_task.split(';')
    rospy.loginfo("split1: %s",task_situation)  
    

    # Initial Situation
    initial_situation = task_situation[0]
    rospy.loginfo("init: %s", initial_situation)        

    if(initial_situation[0:16] != "initialsituation"):
       raise Exception("Excepted <<initialsituation>>, but received <<" + initial_situation[0:16] + ">> received")

    initial_situation = initial_situation[16:len(initial_situation)]
    rospy.loginfo('removed <> and (): %s',initial_situation)

    init_tasks = re.findall('\<(?P<name>.*?)\>', initial_situation)
    rospy.loginfo("split into poses: %s",init_tasks)

    # Update task list with expcted initial situation information
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
        task_list.append(initial_tasklist)

    
    # Goal Situation
    goal_situation = task_situation[1]
    rospy.loginfo('goal %s', goal_situation)    

    if(goal_situation[0:13] != "goalsituation"):
       raise Exception("Excepted <<goalsituation>>, but received <<" + goal_situation[0:13] + ">> received")

    goal_situation = goal_situation[13:len(goal_situation)]
    rospy.loginfo('removed goal string: %s', goal_situation)

    goal_tasks = re.findall('\<(?P<name>.*?)\>', goal_situation)
    rospy.loginfo('split into locations: %s', goal_tasks)

    # Update task list with expcted goal situation information
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
        task_list.append(goal_tasklist)        

    return task_list
 

def get_precision_placement_task(precision_task):

    task_list = []

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
       raise Exception("Excepted <<BTT>> task, but received <<" + transportation_task[0:3] + ">> received")


    # remove leading start description        
    transportation_task = transportation_task[3:len(transportation_task)]
    
    
    # check if description has beginning '<' and ending '>
    if(transportation_task[0] != "<" or transportation_task[(len(transportation_task)-1)] != ">"):
        raise Exception("task spec not in correct format")
    
    # remove beginning '<' and ending '>'
    transportation_task = transportation_task[1:len(transportation_task)-1]

    # Task split
    task_situation = transportation_task.split(';')
    rospy.loginfo("split1: %s",task_situation)  
    

    # Initial Situation
    initial_situation = task_situation[0]
    rospy.loginfo("init: %s", initial_situation)        

    if(initial_situation[0:16] != "initialsituation"):
       raise Exception("Excepted <<initialsituation>>, but received <<" + initial_situation[0:16] + ">> received")

    initial_situation = initial_situation[16:len(initial_situation)]
    rospy.loginfo('removed <> and (): %s',initial_situation)

    init_tasks = re.findall('\<(?P<name>.*?)\>', initial_situation)
    rospy.loginfo("split into poses: %s",init_tasks)

    # Update task list with expcted initial situation information
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
        task_list.append(initial_tasklist)

    
    # Goal Situation
    goal_situation = task_situation[1]
    rospy.loginfo('goal %s', goal_situation)    

    if(goal_situation[0:13] != "goalsituation"):
       raise Exception("Excepted <<goalsituation>>, but received <<" + goal_situation[0:13] + ">> received")

    goal_situation = goal_situation[13:len(goal_situation)]
    rospy.loginfo('removed goal string: %s', goal_situation)

    goal_tasks = re.findall('\<(?P<name>.*?)\>', goal_situation)
    rospy.loginfo('split into locations: %s', goal_tasks)

    # Update task list with expcted goal situation information
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
        task_list.append(goal_tasklist)    

    print "PARSED TASK: "
    print "-----------------------------------------------------"
    for task in task_list:
      print "type:", task.type
      print "   location:", task.location
      print "   objects:", task.object_names  
      if task.type == "destination":
        print "   objects:", task.object_config       
      print "-----------------------------------------------------"

    return task_list


def get_basic_competitive_task(competitive_task):
    task_list = []
   
    # check if description has beginning '<' and ending '>
    if(competitive_task[0] != "<" or competitive_task[(len(competitive_task)-1)] != ">"):
        raise Exception("task spec not in correct format")
    
    # remove beginning '<' and ending '>'
    competitive_task = competitive_task[1:len(competitive_task)-1]

    # Task split
    task_situation = competitive_task.split(';')
    rospy.loginfo(task_situation)  
    

    # Initial Situation
    initial_situation = task_situation[0]
    rospy.loginfo(initial_situation)        

    if(initial_situation[0:16] != "initialsituation"):
       raise Exception("Excepted <<initialsituation>>, but received <<" + initial_situation[0:16] + ">> received")

    initial_situation = initial_situation[16:len(initial_situation)]
    rospy.loginfo(initial_situation)

    init_tasks = re.findall('\<(?P<name>.*?)\>', initial_situation)
    rospy.loginfo(init_tasks)

    # Update task list with expcted initial situation information
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
        task_list.append(initial_tasklist)

    
    # Goal Situation
    goal_situation = task_situation[1]
    rospy.loginfo(goal_situation)    

    if(goal_situation[0:13] != "goalsituation"):
       raise Exception("Excepted <<goalsituation>>, but received <<" + goal_situation[0:13] + ">> received")

    rospy.loginfo(goal_situation)
    goal_situation = goal_situation[13:len(goal_situation)]
    rospy.loginfo(goal_situation)

    goal_tasks = re.findall('\<(?P<name>.*?)\>', goal_situation)
    rospy.loginfo(goal_tasks)

    # Update task list with expcted goal situation information
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
        task_list.append(goal_tasklist)

    
    return task_list

