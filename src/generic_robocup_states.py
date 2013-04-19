import roslib
roslib.load_manifest('raw_generic_states')

import rospy
import smach
import referee_box_communication
import re

from tasks import parse_task, TaskSpecFormatError

ip = "192.168.51.61"
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

    HARDCODED_SPECS = {'BNT': 'BNT<(D1,W,1),(S1,E,3),(S2,E,3),(D2,S,3),(S3,W,3),(S2,W,3),(D2,W,3),(S1,W,3),(S2,W,3),(S3,W,3),(S2,W,3),(D1,W,3)>',
                       'PPT': 'PPT<S1,S2>'}

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['task_received', 'wrong_task_format'],
                             input_keys=['task', 'test', 'simulation'],
                             output_keys=['task'])

    def execute(self, userdata):
        if not userdata.simulation:
            rospy.logdebug('Waiting for task specification (%s:%s)...' % (ip, port))
            task_spec = referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)
        else:
            task_spec = self.HARDCODED_SPECS[userdata.test]
        rospy.loginfo("Task specification: %s" % task_spec)
        try:
            userdata.task = parse_task(userdata.test, task_spec)
            rospy.loginfo('Parsed task:\n%s' % userdata.task)
            return 'task_received'
        except TaskSpecFormatError:
            return 'wrong_task_format'


class get_basic_transportation_task(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_received', 'wrong_task_format'], input_keys=['task_list'], output_keys=['task_list'])
        
    def execute(self, userdata):

        rospy.loginfo("Wait for task specification from server: " + ip + ":" + port + " (team-name: " + team_name + ")")

		#transportation_task = 'BTT<initialsituation(<S1,(screw,alu_silver_l)><S2,(alu_silver_s,alu_black_l,alu_black_s)>);goalsituation(<S3,line(alu_silver_l,alu_silver_s)><D1,zigzag(alu_black_s,alu_black_l,screw)>)>'
        #transportation_task = 'BTT<initialsituation(<S3,(F20_20_B,M20_100)>);goalsituation(<S2,zigzag(M20_100,F20_20_B)>)>'
        
        transportation_task = referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name) 

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
        competitive_task = referee_box_communication.obtainTaskSpecFromServer(ip, port, team_name)  #'BTT<(D1,N,6),(S2,E,3)>'
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
