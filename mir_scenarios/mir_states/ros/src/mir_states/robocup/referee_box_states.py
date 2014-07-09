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
                       'BMT': 'BMT<S3,S3,S2,line(M20_100,F20_20_G,F20_20_B),S2>',
                       'BTT': 'BTT<initialsituation(<S5,(R20,M30,S40_40_B)><S2,(S40_40_G,M20,R20)><S3,(F20_20_B,M20_100,F20_20_G)>);goalsituation(<C1,line(M20_100,M30,M20)><S4,line(F20_20_G,R20,R20)><S1,line(S40_40_B,S40_40_G,F20_20_B)>)>',
                       #'BTT': 'BTT<initialsituation(<S4,(S40_40_G)>);goalsituation(<S5,line(S40_40_G)>)>',
                       'PPT': 'PPT<S1,(M20, S_40_40_G),S2>'}

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
    
        # remove outer task name and '<', '>'. If task name is not correct throws an Exception.
        task_spec = re.findall(r'%s<(.*)>' % userdata.test, task_spec)

        if len(task_spec) != 1:
           raise Exception('Malformed task specification')

        try:
            if(userdata.test == "BNT"):
                userdata.task_list = get_basic_navigation_task(task_spec[0])
            if(userdata.test == "BMT"):
                userdata.task_list = get_basic_manipulation_task(task_spec[0])
            elif(userdata.test == "BTT"):
                userdata.task_list = get_basic_transportation_task(task_spec[0])
            elif(userdata.test == "PTT"):
                userdata.task_list = get_precision_placement_task(task_spec[0])
            elif(userdata.test == "CTT"):
                userdata.task_list = get_competitive_transportation_task(task_spec[0])
        except Exception as e:
            rospy.logerr("Exception: %s", e)
            return 'wrong_task_format'

        return 'task_received' 


def get_basic_navigation_task(navigation_task):  

    task_list = re.findall(r'\('
                            '(?P<place>.+?),'
                            '(?P<orientation>[NESW]),'
                            '(?P<break>[123])'
                            '\)', navigation_task)
    
    return task_list


def get_basic_manipulation_task(man_task):
   
    task_spec = man_task.split(',')
    
    init_pose = task_spec[0]
    src_pose = task_spec[1]
    dest_pose = task_spec[2]
    
    subtask_spec = task_spec[3].split('(')
    obj_cfg = subtask_spec[0]
    
    obj_names = []
    obj_names.append(subtask_spec[1])

    print task_spec
    
    for i in range(4, (len(task_spec)-1)):
        if i == (len(task_spec)-2):
            print task_spec[i]
            task_spec[i] = task_spec[i][0:(len(task_spec[i])-1)]
            print task_spec[i]
            
             
        obj_names.append(task_spec[i])
           
    
    fnl_pose = task_spec[len(task_spec)-1]
    
    for obj in range(len(obj_names)):
        if obj_names[obj] == "V20":
            obj_names[obj] = "R20"


    print obj_names
    
    # which object to get from the source location
    task_list = []
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

    # create BTT task spec
    result = "initialsituation(" + source
    result = result + objects + ">);"
    result = result + "goalsituation(<" + destination + ","
    result = result + "line" + objects + ">)"

    return get_basic_transportation_task(result), destination


def get_competitive_transportation_task(competitive_task):
    task_list = []

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

