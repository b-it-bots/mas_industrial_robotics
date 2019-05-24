import rospy
import smach
import sys
import tf

import pandas as pd
import glob

import std_msgs.msg
import sensor_msgs.msg
#import atwork_ros_msgs.msg
import atwork_ros_msgs.msg
import mas_perception_msgs.msg


def convert_msg_to_log(msg):
    """
    msg type is BenchmarkFeedback
    """

    quaternion = (
        msg.object_pose.orientation.x,
        msg.object_pose.orientation.y,
        msg.object_pose.orientation.z,
        msg.object_pose.orientation.w)
    yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
    x = msg.object_pose.position.x
    y = msg.object_pose.position.y
    return msg.object_class_name.data + " " + msg.object_instance_name.data + " " + str(x) + " " + str(y) + " " + str(yaw)


class get_benchmark_state(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['running', 'running_calibration',
                        'running_preparation', 'running_execution',
                        'paused', 'finished', 'stopped','finished_execution'],
            input_keys=['benchmark_state'],
            output_keys=['benchmark_state'])

        self.state_sub = rospy.Subscriber("/robot_example_ros/benchmark_state",
                            atwork_ros_msgs.msg.BenchmarkState, self.benchmark_state_cb, queue_size=1)
        self.bm_state = None

    def benchmark_state_cb(self, benchmarkstate):
        self.bm_state = benchmarkstate

    def execute(self, userdata):
        self.bm_state = None
        userdata.benchmark_state = None
        rospy.loginfo("Asking for benchmark state")

        timeout = rospy.Duration.from_sec(5.0)
        start_time = rospy.Time.now()

        while(True):
            if not self.bm_state:
                if (rospy.Time.now() - start_time) > timeout:
                    rospy.logerr("Timed out waiting for benchmark state")
                    return 'paused'
                rospy.sleep(0.1)
                continue
            break

        userdata.benchmark_state = self.bm_state
        if (self.bm_state.state.data == atwork_ros_msgs.msg.BenchmarkState.RUNNING):
            if (self.bm_state.phase.data == atwork_ros_msgs.msg.BenchmarkState.CALIBRATION):
                rospy.loginfo("STATE: running_calibration")
                return "running_calibration"
            elif (self.bm_state.phase.data == atwork_ros_msgs.msg.BenchmarkState.PREPARATION):
                rospy.loginfo("STATE: running_preparation")
                return "running_preparation"
            elif (self.bm_state.phase.data == atwork_ros_msgs.msg.BenchmarkState.EXECUTION):
                rospy.loginfo("STATE: running_execution")
                return "running_execution"
            else:
                return 'running'
        elif (self.bm_state.state.data == atwork_ros_msgs.msg.BenchmarkState.PAUSED):
            rospy.loginfo("STATE: paused")
            return 'paused'
        elif (self.bm_state.state.data == atwork_ros_msgs.msg.BenchmarkState.STOPPED):
            rospy.loginfo("STATE: stopped")
            return 'stopped'
        elif (self.bm_state.state.data == atwork_ros_msgs.msg.BenchmarkState.FINISHED):
            if (self.bm_state.phase.data == atwork_ros_msgs.msg.BenchmarkState.EXECUTION):
                rospy.loginfo("STATE: finished_execution")
                return "finished_execution"
            else:
                rospy.loginfo("STATE: finished")
                return 'finished'
        return 'paused'

class send_refbox_logging_status(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['done'],
                                input_keys=['logging_status'])

        self.logging_status_pub = rospy.Publisher("/robot_example_ros/logging_status",
                                            atwork_ros_msgs.msg.LoggingStatus)

    def execute(self, userdata):
        msg = atwork_ros_msgs.msg.LoggingStatus()
        msg.is_logging.data = userdata.logging_status

        self.logging_status_pub.publish(msg)
        rospy.loginfo("Published benchmark logging status")
        return 'done'

# TODO save all object names and classes to a txt file
class send_benchmark_feedback_fbm1(smach.State):
    def __init__(self, phase='exec'):
        smach.State.__init__(self,
            outcomes=['done'],
            input_keys=['recognized_objects',
                        'is_object_grasped',
                        'end_effector_pose',
                        'benchmark_state'])

        self.benchmark_pub = rospy.Publisher("/robot_example_ros/benchmark_feedback",
                                    atwork_ros_msgs.msg.BenchmarkFeedback)
        self.components = ["BEARING_BOX", "BEARING", "MOTOR", "AXIS", "DISTANCE_TUBE"]
        self.profiles = ["F20_20_G", "F20_20_B", "S40_40_G", "S40_40_B", "M20_100"]
        self.plastics = ["R20"]
        self.nuts = ["M20", "M30"]
        self.prefer_rgb = ["F20_20_G", "F20_20_B", "S40_40_G", "S40_40_B", "R20"]
        self.prefer_pcl = ["F20_20_G", "F20_20_B", "S40_40_G", "S40_40_B"]
        self.degrade_conf = 0.7
        self.phase_to_term = phase
        self.obj_sub = rospy.Subscriber('/mcr_perception/object_list_merger/output_object_list', mas_perception_msgs.msg.ObjectList, self.callback, queue_size=1)
        self.grasp_sub = rospy.Subscriber('/gripper_controller/grasp_monitor/event_out', std_msgs.msg.String, self.grasp_callback, queue_size=1)
        self.obj = None
        self.grasp = None
        self.dataset = ParserDataset()

    def callback(self, msg):
        #print "callback ", msg.objects
        self.obj = msg

    def grasp_callback(self, msg):
        self.grasp = msg

    def execute(self, userdata):
        msg = atwork_ros_msgs.msg.BenchmarkFeedback()
        if self.phase_to_term == 'prep':
            msg.phase_to_terminate.data = atwork_ros_msgs.msg.BenchmarkState.PREPARATION
            self.benchmark_pub.publish(msg)
            self.obj = None
            self.grasp = None
            return 'done'
        elif self.phase_to_term == 'exec':
            msg.phase_to_terminate.data = atwork_ros_msgs.msg.BenchmarkState.EXECUTION
            log_msg = std_msgs.msg.String()
            print("phase to terminate: EXECUTION")
            if self.obj != None:
                obj = None
                for tmp_obj in self.obj.objects:
                    # choose the first 2D object
                    if tmp_obj.database_id > 99:
                        if obj is None:
                            obj = tmp_obj
                        elif obj.probability < tmp_obj.probability:
                            obj = tmp_obj
                    if tmp_obj.name in self.prefer_pcl:
                        if tmp_obj.database_id < 99:
                            obj = tmp_obj

                if obj is None:
                    obj = self.obj.objects[0]
                                        
                '''
                if len(self.obj.objects) > 1:
                    probabs = {}
                    # Update probabilities
                    for tmp_obj in self.obj.objects:
                        if tmp_obj.database_id > 99 and tmp_obj.name in self.prefer_rgb:
                            probabs.update({tmp_obj.name:tmp_ob.probability * self.degrade_probab})
                '''
                print("object exists", obj.name)
                msg.object_instance_name.data = obj.name
                obj_dict = self.dataset.load_files()
                msg.object_class_name.data = obj_dict[obj.name]

                '''
                if obj.name in self.components:
                    msg.object_class_name.data = "Component"
                elif obj.name in self.profiles:
                    msg.object_class_name.data = "Profile"
                elif obj.name in self.nuts:
                    msg.object_class_name.data = "Nut"
                elif obj.name in self.plastics:
                    msg.object_class_name.data = "Plastic"
                '''
                    
            else:
                print("object empty")
                msg.object_instance_name.data = ""
                msg.object_class_name.data = ""
                msg.grasp_notification.data = False
            if userdata.end_effector_pose:
                msg.end_effector_pose = userdata.end_effector_pose.pose
            print ("message is ", msg)

        self.benchmark_pub.publish(msg)
        rospy.loginfo("Published benchmark feedback - FBM1")
        return 'done'

class send_benchmark_feedback_fbm2(smach.State):
    def __init__(self, phase='exec'):
        smach.State.__init__(self,
            outcomes=['done'],
            input_keys=['recognized_objects',
                        'is_object_grasped',
                        'end_effector_pose',
                        'benchmark_state'])

        self.benchmark_pub = rospy.Publisher("/robot_example_ros/benchmark_feedback",
                                    atwork_ros_msgs.msg.BenchmarkFeedback)
        self.containers = ["EM-01", "EM-02"]
        self.bearing_boxes = ["BEARING_BOX"]
        self.transmission_parts = ["BEARING", "MOTOR", "AXIS"]
        self.phase_to_term = phase
        self.obj_sub = rospy.Subscriber('/mcr_perception/object_selector/output/object', mas_perception_msgs.msg.Object, self.callback, queue_size=1)
        self.grasp_sub = rospy.Subscriber('/gripper_controller/grasp_monitor/event_out', std_msgs.msg.String, self.grasp_callback, queue_size=1)
        self.obj = None
        self.grasp = None
	# load object dataset from excel
	self.dataset = ParserDataset()

    def callback(self, msg):
        self.obj = msg

    def grasp_callback(self, msg):
        self.grasp = msg

    def execute(self, userdata):
        msg = atwork_ros_msgs.msg.BenchmarkFeedback()
        if self.phase_to_term == 'prep':
            msg.phase_to_terminate.data = atwork_ros_msgs.msg.BenchmarkState.PREPARATION
            self.benchmark_pub.publish(msg)
            self.obj = None
            self.grasp = None
            return 'done'
        elif self.phase_to_term == 'exec':
            msg.phase_to_terminate.data = atwork_ros_msgs.msg.BenchmarkState.EXECUTION
            log_msg = std_msgs.msg.String()
            print("phase to terminate: EXECUTION")


            if self.obj != None:
                obj = self.obj
                print("object exists", obj.name)
                #msg.object_instance_name.data = obj.name
		
		#obj.name = self.dataset.load_files[obj.name]
			
		'''
                if obj.name == 'CONTAINER_BOX_BLUE' or obj.name == 'CONTAINER_BOX_RED':
                    obj.name = 'EM-02'
                if obj.name == 'M20' or obj.name == 'DISTANCE_TUBE':
                    obj.name = 'BEARING'
                if obj.name == 'R20':
                    obj.name = 'MOTOR'
                if obj.name == 'F20_20_G':
                    obj.name = 'AXIS'
                if obj.name == 'S40_40_G':
                    obj.name = 'BEARING_BOX'
		'''
                obj_dict = self.dataset.load_files()
                msg.object_instance_name.data = obj.name
		msg.object_class_name.data = obj_dict[obj.name]

		'''
                if obj.name in self.containers:
                    msg.object_class_name.data = "Containers"
                elif obj.name in self.bearing_boxes:
                    msg.object_class_name.data = "Bearing Boxes"
                elif obj.name in self.transmission_parts:
                    msg.object_class_name.data = "Transmission Parts"
                '''

                if self.grasp:
                    if self.grasp.data == 'e_object_grasped':
                       msg.grasp_notification.data = True
                    else:
                       msg.grasp_notification.data = False
            else:
                print("object empty")
                msg.object_instance_name.data = ""
                msg.object_class_name.data = ""
                msg.grasp_notification.data = False
            if userdata.end_effector_pose:
                msg.end_effector_pose = userdata.end_effector_pose.pose
            print ("message is ", msg)

        self.benchmark_pub.publish(msg)
        rospy.loginfo("Published benchmark feedback - FBM2")
        return 'done'

class send_benchmark_feedback_fbm3(smach.State):
    def __init__(self, phase='prep'):
        smach.State.__init__(self,
            outcomes=['done'],
            input_keys=['recognized_objects',
                        'is_object_grasped',
                        'end_effector_pose',
                        'benchmark_state'])


        self.benchmark_pub = rospy.Publisher("/robot_example_ros/benchmark_feedback",
                                            atwork_ros_msgs.msg.BenchmarkFeedback)
        self.phase_to_term = phase

        self.bm_state = None

    def execute(self, userdata):

        '''
        send feedback msg to terminate phase.
        '''
        msg = atwork_ros_msgs.msg.BenchmarkFeedback()

        if self.phase_to_term == 'prep':
            msg.phase_to_terminate.data = atwork_ros_msgs.msg.BenchmarkState.PREPARATION
            self.benchmark_pub.publish(msg)

        elif self.phase_to_term == 'exec':
            msg.phase_to_terminate.data = atwork_ros_msgs.msg.BenchmarkState.EXECUTION
            msg.grasp_notification.data = True
            self.benchmark_pub.publish(msg)
            print("phase to terminate: EXECUTION")

        return 'done'

class send_benchmark_feedback_plate_drilling(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['done', 'failure'],
            input_keys=['after_receiving',
                        'after_drilling'])

        self.benchmark_pub = rospy.Publisher("/robot_example_ros/benchmark_feedback",
                                atwork_ros_msgs.msg.BenchmarkFeedback)

    def execute(self, userdata):
        if userdata.after_receiving is None and userdata.after_drilling is None:
            rospy.logwarn('[send_benchmark_feedback_platedrilling] Invalid feedback data')
            return 'failure'
        msg = atwork_ros_msgs.msg.BenchmarkFeedback()
        msg.phase_to_terminate.data = msg.EXECUTION
        if userdata.after_receiving:
            msg.plate_state_after_receiving.data = userdata.after_receiving

        if userdata.after_drilling:
            msg.plate_state_after_drilling.data = userdata.after_drilling

        self.benchmark_pub.publish(msg)
        rospy.loginfo("Published plate drilling online benchmark feedback")
        return 'done'

class send_benchmark_feedback_force_fitting(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['done', 'failure'],
            input_keys=['assembly_aid_tray_id',
                        'container_id'])

        self.benchmark_pub = rospy.Publisher("/robot_example_ros/benchmark_feedback",
                                atwork_ros_msgs.msg.BenchmarkFeedback)

    def execute(self, userdata):
        if userdata.assembly_aid_tray_id is None or userdata.container_id is None:
            rospy.logwarn('[send_benchmark_feedback_platedrilling] Invalid feedback data')
            return 'failure'
        msg = atwork_ros_msgs.msg.BenchmarkFeedback()
        msg.phase_to_terminate.data = msg.EXECUTION
        msg.assembly_aid_tray_id = userdata.assembly_aid_tray_id
        msg.contianer_id = userdata.contianer_id
        self.benchmark_pub.publish(msg)
        rospy.loginfo("Published plate drilling online benchmark feedback")
        return 'done'

class conveyor_belt_command(smach.State):
    """
    Only one possible command: 'turn_on'
    Sends the command until the 'cycle' variable on TriggeredConveyorBeltStatus increments
    """
    def __init__(self, command, timeout=10):
        smach.State.__init__(self,
            outcomes=['done', 'timeout'])

        self.conveyor_belt_command_pub = rospy.Publisher("/robot_example_ros/conveyor_belt_command",
                                                atwork_ros_msgs.msg.TriggeredConveyorBeltCommand)
        self.conveyor_belt_status_sub = rospy.Subscriber("/robot_example_ros/conveyor_belt_status",
                                                atwork_ros_msgs.msg.TriggeredConveyorBeltStatus, self.status_cb)
        self.timeout = rospy.Duration.from_sec(timeout)
        self.command = command
        self.status_msg = None
        if self.command != "turn_on":
            rospy.logerr("Conveyor belt only supports 'turn_on' command")

    def status_cb(self, msg):
        self.status_msg = msg

    def execute(self, userdata):
        self.status_msg = None

        loop_rate = rospy.Rate(5)
        start_time = rospy.Time.now()
        # wait till we get the conveyor belt status
        while not self.status_msg and (rospy.Time.now() - start_time) < self.timeout:
            loop_rate.sleep()

        if not self.status_msg:
            return 'timeout'

        # create command with incremented cycle
        msg = atwork_ros_msgs.msg.TriggeredConveyorBeltCommand()
        msg.next_cycle.data = self.status_msg.cycle.data + 1
        msg.command.data = atwork_ros_msgs.msg.TriggeredConveyorBeltCommand.START

        start_time = rospy.Time.now()
        # publish until status message cycle changes (increments by one)
        while self.status_msg.state.data != atwork_ros_msgs.msg.TriggeredConveyorBeltStatus.START or \
                self.status_msg.cycle.data != msg.next_cycle.data:
            self.conveyor_belt_command_pub.publish(msg)
            loop_rate.sleep()
            rospy.logdebug("Waiting for conveyor belt to turn on")
            if (rospy.Time.now() - start_time) > self.timeout:
                return 'timeout'
        rospy.loginfo("Conveyor belt is running")
        return 'done'

class drilling_machine_command(smach.State):
    """
    Possible commands: 'move_up', 'move_down'

    """
    def __init__(self, command, timeout=10):
        smach.State.__init__(self,
            outcomes=['done', 'timeout'])

        self.drilling_machine_command_pub = rospy.Publisher("/robot_example_ros/drilling_machine_command",
                                                atwork_ros_msgs.msg.DrillingMachineCommand)
        self.drilling_machine_status_sub = rospy.Subscriber("/robot_example_ros/drill_machine_status",
                                                atwork_ros_msgs.msg.DrillingMachineStatus, self.status_cb)
        self.timeout = rospy.Duration.from_sec(timeout)
        self.start_time = None
        self.command = command
        self.status_msg = None
        self.loop_rate = rospy.Rate(5)
        if self.command not in ['move_up', 'move_down']:
            rospy.logerr("Possible drilling machine commands are 'move_up' or 'move_down'")

    def status_cb(self, msg):
        self.status_msg = msg

    def re_publish_msg(self, msg):
        self.drilling_machine_command_pub.publish(msg)
        rospy.logdebug("Waiting for drill to reach %s", self.command)
        self.loop_rate.sleep()

    def execute(self, userdata):
        self.status_msg = None

        msg = atwork_ros_msgs.msg.DrillingMachineCommand()
        expected_state = None
        if self.command == "move_up":
            msg.command.data = atwork_ros_msgs.msg.DrillingMachineCommand.MOVE_UP
            expected_state = atwork_ros_msgs.msg.DrillingMachineStatus.AT_TOP
        elif self.command == "move_down":
            msg.command.data = atwork_ros_msgs.msg.DrillingMachineCommand.MOVE_DOWN
            expected_state = atwork_ros_msgs.msg.DrillingMachineStatus.AT_BOTTOM

        self.start_time = rospy.Time.now()
        #check if the data has received
        while(None == self.status_msg):
            self.re_publish_msg(msg)
            if (rospy.Time.now() - self.start_time) > self.timeout:
                return 'timeout'

        # publish until expected state is reached
        while (self.status_msg.state.data != expected_state):
            self.re_publish_msg(msg)
            if (rospy.Time.now() - self.start_time) > self.timeout:
                return 'timeout'

        rospy.loginfo("Drilling machine has reached %s" , self.command)
        return 'done'


class get_task_spec(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['task_received', 'timeout'],
            output_keys=['task_list'])

        self.request_state_pub = rospy.Publisher("/task_specification_transformer_node/event_in",
                                                    std_msgs.msg.String)
        self.state_sub = rospy.Subscriber("/task_specification_transformer_node/task_specification",
                                                    std_msgs.msg.String, self.task_spec_cb)
        self.task_spec_str = None

    def task_spec_cb(self, event):
        self.task_spec_str = event.data

    def execute(self, userdata):
        rospy.loginfo("Asking for task spec")
        self.request_state_pub.publish("e_start")

        timeout = rospy.Duration.from_sec(10.0)
        start_time = rospy.Time.now()

        while(True):
            if not self.task_spec_str:
                if (rospy.Time.now() - start_time) > timeout:
                    rospy.logerr("Timed out waiting for task spec")
                    return 'timeout'
                rospy.sleep(0.1)
                continue
            task_spec = self.task_spec_str
            rospy.loginfo("Task specification: %s" % task_spec)

            # remove outer task name and '<', '>'. If task name is not correct throws an Exception.
            task_spec = re.findall(r'%s<(.*)>' % userdata.test, task_spec)

            if len(task_spec) != 1:
                raise Exception('Malformed task specification')

            userdata.task_list = get_basic_transportation_task(task_spec[0])
            break
        return 'task_received'


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
        desired_loc = item[0:3]

        obj_taskspec = item[4:len(item)]

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
        desired_loc = item[0:3]

        obj_taskspec = item[4:len(item)]

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

