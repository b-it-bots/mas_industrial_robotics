#!/usr/bin/env python
"""

"""
#-*- encoding: utf-8 -*-
__author__ = 'dnair2s'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import atwork_ros_msgs.msg

import roslib
import actionlib
import sys
from mir_yb_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from mir_yb_action_msgs.msg import MoveBaseSafeResult

from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.srv import GetDomainAttributeService
from rosplan_knowledge_msgs.srv import GetDomainTypeService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

class RefboxParser(object):
    """
    World model for storing the objects
    """
    def __init__(self):

        #inventory message
        self._inventory_message = None
        #order messge
        self._tasks = None

        #list for storing the parameters from domain 
        self._predicate_param_type_list = {}
        self._predicate_param_label_list = {}

        # model view
        self._type_list = []

        #list to remember the naming given to models
        self._model_number = {}

        #The dictinary to store name of the dommain as per pddl
        #NOTE container is of type object as per the pddl designed 
        self.domain_name = {'object':'object', 'location':'location', 'container':'object', 'destination':'location'}

        #The enum value for rosplan_knowledge_msgs/KnowledgeUpdateService
        self.knowledge_update_service = {'ADD_KNOWLEDGE':0, 'ADD_GOAL':1, 'REMOVE_KNOWLEDGE':2, 'REMOVE_GOAL':3}

        #Enum of object types from messages rocking
        #https://github.com/rockin-robot-challenge/at_work_central_factory_hub/blob/rockin/rockin/msgs/Inventory.proto
        self.object_type = {1:'F20_20_B', 2:'F20_20_G', 3:'S40_40_B',
                            4:'S40_40_G' , 5:'M20_100', 6:'M20',
                            7:'M30' , 8:'R20', 9:'BEARING_BOX',
                            10:'BEARING' , 11:'AXIS', 12:'DISTANCE_TUBE',
                            13:'MOTOR', 14:'CONTAINER_BOX_BLUE', 15:'CONTAINER_BOX_RED'}

        self.location_type = {1:'SH', 2:'WS', 3:'CB', 4:'WP', 5:'PP', 6:'ROBOT'}

        self.orientation = {1:'NORTH', 2:'EAST', 3:'SOUTH', 4:'WEST'}

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 0.5))

        #Subscriber -> subscribe to refbox Inventory message
        rospy.Subscriber("~refbox", atwork_ros_msgs.msg.Inventory, self.inventory_callback, queue_size=10)

        #Subscriber -> subscribe to refbox for Order message
        rospy.Subscriber("~refbox_task", atwork_ros_msgs.msg.TaskInfo, self.task_callback, queue_size=10)

        #event in subscriber
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.eventInCallBack, queue_size=10)

        #subscriber for robot pose for benchmarking
        rospy.Subscriber("/rockin/robot_pose", geometry_msgs.msg.PoseStamped, self.robotPoseCallBack, queue_size=1)

        #subscriber for IR marker pose for benchmarking
        rospy.Subscriber("/rockin/marker_pose", geometry_msgs.msg.PoseStamped, self.markerPoseCallBack, queue_size=1)

        # For benchmarking
        self.pub_robot_pose = rospy.Publisher("/rockin/robot_pose_waypoint",
                                              geometry_msgs.msg.PoseStamped, queue_size=1)
        self.pub_marker_pose = rospy.Publisher("/rockin/marker_pose_waypoint",
                                               geometry_msgs.msg.PoseStamped, queue_size=1)

        # For logging
        self.pub_logging_status = rospy.Publisher("/robot_example_ros/logging_status",
                                                  atwork_ros_msgs.msg.LoggingStatus, queue_size=1)
        self.pub_logger = rospy.Publisher("/mcr_tools/rosbag_recorder/event_in", std_msgs.msg.String, queue_size=1)

        # even inventory flag
        self.start_inventory_flag = True

        # event of completion of inventory update for starting task update
        self.inventory_update_complete_flag = False

        # even order flag
        self.start_order_flag = True

        #Robocup hack not include object on rotating table
        self.object_to_delete_goals = []

        #publisher
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)

        bnt =  rospy.get_param("~bnt", True)

        if bnt:
            rospy.loginfo('Running BNT from refbox. Not waiting for the database world model')
        else:
            rospy.loginfo('waiting for update world model service')
            rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')

            self.update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base',
                                            KnowledgeUpdateService)

            # populate goal combo boxes
            rospy.wait_for_service('/kcl_rosplan/get_domain_predicates')
            try:
                predicates_client = rospy.ServiceProxy('/kcl_rosplan/get_domain_predicates', GetDomainAttributeService)
                resp = predicates_client()
                for pred in resp.items:
                    param_list = []
                    label_list = []
                    for param in pred.typed_parameters:
                        param_list.append(param.value)
                        label_list.append(param.key)
                    self._predicate_param_type_list[pred.name] = param_list
                    self._predicate_param_label_list[pred.name] = label_list
                print "Predicate param type list"
                print self._predicate_param_type_list
                print "Predicate param label list"
                print self._predicate_param_label_list
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            # populate type combo box
            rospy.wait_for_service('/kcl_rosplan/get_domain_types')
            try:
                type_client = rospy.ServiceProxy('/kcl_rosplan/get_domain_types', GetDomainTypeService)
                resp = type_client()
                for typename in resp.types:
                    self._type_list.append(typename)

                print self._type_list
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        rospy.loginfo("Started node Refbox")
    def eventInCallBack(self, msg):
        if msg.data == 'e_trigger':
            rospy.loginfo('Received trigger updating knowledge base based on order and invetory data')
            self.start_inventory_flag = True
            self.start_order_flag = True

    def start(self):
        while not rospy.is_shutdown():
            if self.start_inventory_flag :
                self.load_inventory_to_knowledge_base()
                #Resetting the namings
                self._model_number = {}
            if self.start_order_flag:
                self.process_task_message()
                #Resetting the namings
                self._model_number = {}
            self.loop_rate.sleep()

    '''
    Callback on inventory message

    Inventory message has a list of "Items".
    Each Item has the following structure.


    object                      ->description of the object
        type
        type_id
        instance_id(optional)   ->If provided then dont check quantity
        description(optional)
    quantity(optional)          ->If instance_id not provided check this
    container(optional)         ->The container in which the item is stored
        type
        type_id
        instance_id(optional)
        description(optional)
    location(optional)          ->Location where the object is stored
        type
        instance_id
        description(optional)
    '''
    def inventory_callback(self, msg):
        self._inventory_message = atwork_ros_msgs.msg.Inventory(msg.items)
	pass

    def load_instance_message_PP01(self, domain):
        #Creating the message
        instance_msg = KnowledgeItem()
        #uploading the instance of object

        instance_msg.knowledge_type = KnowledgeItem.INSTANCE
        instance_msg.instance_type = self.domain_name[domain]
        instance_msg.instance_name = "PP01_CAVITY"
        instance_msg.attribute_name = ''
        instance_msg.function_value = 0.0

        self.write_to_knowledge_base(instance_msg, self.knowledge_update_service['ADD_KNOWLEDGE'])
        return instance_msg.instance_name

    def load_instance_message(self, domain, item, count=1 ):
        #Creating the message
        instance_msg = KnowledgeItem()
        #uploading the instance of object

        instance_msg.knowledge_type = KnowledgeItem.INSTANCE
        instance_msg.instance_type = self.domain_name[domain]
        instance_msg.instance_name = self.msg_to_instance_name(item, domain, count)
        instance_msg.attribute_name = ''
        instance_msg.function_value = 0.0

        self.write_to_knowledge_base(instance_msg, self.knowledge_update_service['ADD_KNOWLEDGE'])
        return instance_msg.instance_name

    def load_fact_message(self, attribute, uploaded_instance, item, domain):
        fact_msg = KnowledgeItem()
        fact_msg.knowledge_type = KnowledgeItem.FACT
        fact_msg.instance_type = ''
        fact_msg.instance_name = ''
        fact_msg.attribute_name = attribute
        fact_msg.function_value = 0.0
        output_value = KeyValue()
        output_value.key = (self._predicate_param_label_list[fact_msg.attribute_name])[0]
        output_value.value = uploaded_instance
        fact_msg.values.append(output_value)

        #write to knowledge base
        ## For insert domain, the attribute container needs to be added as a fact, without second term
        if (('in' == attribute) or ('on' == attribute)):
            output_value = KeyValue()
            output_value.key = (self._predicate_param_label_list[fact_msg.attribute_name])[1]
            output_value.value = self.msg_to_instance_name(item, domain)
            fact_msg.values.append(output_value)

            ##Robocup hack for finals 
            ## Removing instance of objects to be picked form rotation table CB02
            if "CB02" == output_value.value or "CB01" == output_value.value:
                print "Rotating table knowledge base not updateing for object",uploaded_instance
                self.object_to_delete_goals.append(uploaded_instance)
                return
            if "SH01" == output_value.value or "SH02" == output_value.value:
                print "Removing object tot pick from shelf ,", uploaded_instance
                self.object_to_delete_goals.append(uploaded_instance)
                return

        #print (fact_msg)
        self.write_to_knowledge_base(fact_msg, self.knowledge_update_service['ADD_KNOWLEDGE'])

    def load_inventory_to_knowledge_base(self):

        if (None == self._inventory_message):
            return
        
        #Creating the message
        instance_msg = KnowledgeItem()

        for item in self._inventory_message.items:
            #Attribute on which the fact is written
            attribute = None
            object_instance_list = []

            #################################
            ## Uploading INSTANCE of Objects
            #################################
            #check if the instance_id is present
            if (0 != item.object.instance_id.data):
                #uploading the instance of object
                text = self.object_type[item.object.type.data] 
                if (text == 'CONTAINER_BOX_BLUE' or text == 'CONTAINER_BOX_RED'):
                    print ("NOT ADDING CONTAINERS")
                    continue
                else:
                    name = self.load_instance_message('object', item )
                    object_instance_list.append(name)

            else:
                #if not then
                #check quantity and loop on quantity
                if (0 != item.quantity.data):
                    for count in range(item.quantity.data):
                        #uploading the instance of object
                        name = self.load_instance_message('object', item, count)
                        object_instance_list.append(name)
                else:
                    rospy.loginfo("Instance ID not provided as well as quantity ")
                    #uploading the instance of object without ID and quantity 
                    text = self.object_type[item.object.type.data] 
                    if (text == 'CONTAINER_BOX_BLUE' or text == 'CONTAINER_BOX_RED'):
                        continue
                    else:
                        name = self.load_instance_message('object', item)
                        object_instance_list.append(name)
                        self.event_out.publish("e_failure")

            #Flag for identifying if location or container
            instance_type_identifyer = None

            #################################
            ## Uploading INSTANCE of locations
            #################################
            #check if location exist
            if (0 != item.location.type.data):
                name = self.load_instance_message('location', item)

                #fill fact "Object" ON "location"
                attribute = 'on'
                instance_type_identifyer = 'location'

            #################################
            ## Uploading INSTANCE of CONTAINER
            #################################
            ## Removing uploading instances of container doing only while task loading
            ## Causing counting problem in both inventory loading and task loading
            ## since counting is done in both and in task we sometimes dont load task for all the containers 
            ## there are situations in which the container numbering differs from inventory and task loading
            '''
            elif (0 != item.container.type.data):
                name = self.load_instance_message('container', item)
                #upload fact "Object" IN "container"
                attribute = 'in'
                instance_type_identifyer = 'container'
            '''
            #################################
            ## Uploading FACTS
            #################################

            if (attribute):
                for uploaded_instance in object_instance_list:
                    self.load_fact_message(attribute, uploaded_instance, item, instance_type_identifyer)

                attribute = None

        #Resetting the flag when the inventory contained some data
        rospy.loginfo('Received inventory data updated knowledge base')
        self.start_inventory_flag = False
        # Triggering the completion of the inventory update
        self.inventory_update_complete_flag = True

        #clearning inventory message
        self._inventory_message = None


        self.event_out.publish("e_done")


    def write_to_knowledge_base(self, msg, type):
        try:
            #get response
            #rospy.loginfo("Type :%d, msg %d , %s ,%s, %s", type, msg.knowledge_type, msg.instance_type,
            #        msg.instance_name, msg.attribute_name)

            resp1 = self.update_kb(type, msg)
            success = resp1.success
        except rospy.ServiceException, e:
            rospy.logerr( "Service call failed: %s" ,e )
            success = None

        if(success):
            rospy.loginfo('World model successfully updated')
            return 'success'
        else:
            rospy.logerr('Could not update world model, failed')
            self.event_out.publish("e_failure")
            return 'fail'

    def msg_to_instance_name(self, item, domain_name, input_count=1):

        if domain_name == 'object':
            text = self.object_type[item.object.type.data] 
            if -1 == input_count:
                text = text
            else:
                #if the count is available in the list then add 
                object_count = 0
                if self._model_number.has_key(text):
                    object_count = self._model_number[text] + 1
                    self._model_number[text] = object_count
                else:
                    self._model_number[text] = 0

                print "##########################################3"
                print self._model_number
                print "##########################################3"
                text = text + '-' + str(object_count).zfill(2)
        if domain_name == 'container':
            text = self.object_type[item.container.type.data] 
            if -1 == input_count:
                text = text
            else:
                #if the count is available in the list then add 
                object_count = 0
                if self._model_number.has_key(text):
                    object_count = self._model_number[text] + 1
                    self._model_number[text] = object_count
                else:
                    self._model_number[text] = 0

                print "##########################################3"
                print self._model_number
                print "##########################################3"
                text = text + '-' + str(object_count).zfill(2)
		#text = text + '-' + str(input_count).zfill(2)

        if domain_name == 'location':
            text = self.location_type[item.location.type.data] + \
                     str(item.location.instance_id.data).zfill(2)

        if domain_name == 'destination':
            text = self.location_type[item.destination.type.data] + \
                     str(item.destination.instance_id.data).zfill(2)
        return text

    '''
    Order callback for taking the order
    '''
    def task_callback(self, msg):
        self._tasks = atwork_ros_msgs.msg.TaskInfo(msg.tasks)
	pass

    ####################################################################
    # used for benchmarking
    ####################################################################
    def robotPoseCallBack(self, msg):
        self.robot_pose = msg

    def markerPoseCallBack(self, msg):
        self.marker_pose = msg

    ####################################################################
    # Navigation task
    ####################################################################
    def execute_navigation_task(self):
        client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
        rospy.loginfo("Waiting for Movebase actionlib server")
        client.wait_for_server()
        rospy.loginfo("Got Movebase actionlib server sending goals")
        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = 'folded'

        # start logging and tell the refbox that we're logging
        self.pub_logger.publish('e_start')
        logging_status= atwork_ros_msgs.msg.LoggingStatus()
        logging_status.is_logging.data = True
        self.pub_logging_status.publish(logging_status)

        for task in self._tasks.tasks:
            #check if the task is navigation or transportation

            if atwork_ros_msgs.msg.Task.NAVIGATION == task.type.data:
                print self.location_type[task.navigation_task.location.type.data] + \
                                              str(task.navigation_task.location.instance_id.data ).zfill(2)
                try:
                    goal.source_location = ""
                    goal.destination_location = self.location_type[task.navigation_task.location.type.data] + \
                                                 str(task.navigation_task.location.instance_id.data ).zfill(2)
                    goal.destination_orientation = self.orientation[task.navigation_task.orientation.data]
                    timeout = 60.0
                    rospy.loginfo('Sending action lib goal to move_base_safe_server'
                                     + ' , destination : ' + goal.destination_location 
                                     + ' , orientation : '+ goal.destination_orientation)
                    client.send_goal(goal)
                    client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
                    result = client.get_result()
                    if(result):
                        if True == result.success:
                            # wait for required time at location
                            d = rospy.Duration(task.navigation_task.wait_time.data.secs, 0)
                            rospy.sleep(d)
                        else:
                            rospy.logerr('Navigation task failure. TODO to retry')
                    else:
                        client.cancel_goal()
                        rospy.logerr('No Result from client')
                except:
                    rospy.logerr("FAILED . Action server MOVE_BASE_SAFE to desitnaton %s ", goal.destination_location)

        #Exiting when the action is done move to EXIT
        goal.destination_location = "EXIT"
        timeout = 60.0
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        client.cancel_goal()
        rospy.loginfo('Reached EXIT ')

        # stop logging and tell the refbox that we've stopped logging
        self.pub_logger.publish('e_stop')
        logging_status= atwork_ros_msgs.msg.LoggingStatus()
        logging_status.is_logging.data = False 
        self.pub_logging_status.publish(logging_status)



    def process_task_message(self):
        if(None == self._tasks):
            return
        if atwork_ros_msgs.msg.Task.NAVIGATION == self._tasks.tasks[0].type.data:
            self.execute_navigation_task()
        elif atwork_ros_msgs.msg.Task.TRANSPORTATION == self._tasks.tasks[0].type.data:
            #check if the inventory data was written otherwise wait for inventory update
            if False == self.inventory_update_complete_flag :
                #Return without resetting the message and order flag
                rospy.loginfo("The inventory is not updated. Waiting ")
                return
            self.load_order_to_knowledge_base()
        else:
            rospy.logerr("NO VALID TASK PRESENT")

        #Resetting flag when order came and it contined data
        self.start_order_flag = False
        #order messge
        self._tasks = None

    def load_order_to_knowledge_base(self):

        rospy.loginfo("Loading task info to knowledge base")
        #Creating the message
        instance_msg = KnowledgeItem()
        for task in self._tasks.tasks:
            ####################################################################
            # Transportation task
            ####################################################################
            if atwork_ros_msgs.msg.Task.TRANSPORTATION == task.type.data:
                # The task is transportation
                # find the object, its destination

                if (0 != task.transportation_task.container.type_id.data):
                    #container is available
                    #some order contain container in order but not in inventory, so need to add fact explicitly
                    name = self.load_instance_message('destination', task.transportation_task)
                    container_name = self.load_instance_message('container', task.transportation_task)
                    if (container_name):
                        #same the knowledge about the initial location of the container is missin, adding it
                        self.load_fact_message('on', container_name, task.transportation_task, 'destination')
                        self.load_fact_message('container', container_name, task.transportation_task, 'destination')
                        self.load_fact_message('heavy', container_name, task.transportation_task, 'destination')

                        object_name = self.fill_order(task.transportation_task, 'in', 'object', 'container', container_name=container_name)
                        if object_name:
                            self.load_fact_message('insertable', object_name, task.transportation_task, 'destination')
                   

                elif (0 != task.transportation_task.destination.instance_id.data):
                #if (0 != task.transportation_task.destination.instance_id.data): //Add when 'in' domain is not present
                    #In transportationtask sometimes the destination is not there in inventory
                    #Needs to add fact about the inventory
                    name = self.load_instance_message('destination', task.transportation_task)
                    #container not available but destination available
                    insertable_object_name = self.fill_order(task.transportation_task, 'on', 'object', 'destination')
                    ## IF PP01 then we need to load some facts
                    if 5 == task.transportation_task.destination.type.data :
                        self.load_fact_message('on', 'PP01_CAVITY', task.transportation_task, 'destination')
                        self.load_fact_message('container', 'PP01_CAVITY', task.transportation_task, 'destination')
                        self.load_fact_message('heavy', 'PP01_CAVITY', task.transportation_task, 'destination')
                        if(insertable_object_name):
                            self.load_fact_message('insertable', insertable_object_name, task.transportation_task, 'destination')
                        self.load_instance_message_PP01('object')
                else:
                    #both container and destination not available
                    rospy.logerr("No Goal set")

                if (0 != task.transportation_task.container.type_id.data) and \
                (0 != task.transportation_task.destination.instance_id.data):
                    ##Both container and destination available
                    ###ERL2016 the containers need not moved so we dont need to add goal for the containers
                    ###Add the below line when the container needs to be transported
                    #self.fill_order(task.transportation_task, 'on', 'container', 'destination' )
                    ###The Fact about the initial location of the container is missing
                    pass


        rospy.loginfo("Task info upload completed")



    def fill_order(self, order, attribute, first, second, count=1, container_name=None):
        fact_msg = KnowledgeItem()
        fact_msg.knowledge_type = KnowledgeItem.FACT
        fact_msg.instance_type = ''
        fact_msg.instance_name = ''
        fact_msg.attribute_name = attribute
        fact_msg.function_value = 0.0
        if 5 == order.destination.type.data :
            fact_msg.attribute_name = 'in'
        output_value = KeyValue()
        output_value.key = (self._predicate_param_label_list[fact_msg.attribute_name])[0]
        name_of_object = self.msg_to_instance_name(order, first, count)
        output_value.value = name_of_object
        
        ##Robocup finals hack not adding goals for object to be picked from rotation table
        if output_value.value in self.object_to_delete_goals:
            print "Not adding goal for ",output_value.value
            return

        #if 5 == order.object.type.data or 12 == order.object.type.data:
        #    print "Not adding M20_100 and distance tube goal"
        #    return

        fact_msg.values.append(output_value)
        output_value = KeyValue()
        output_value.key = (self._predicate_param_label_list[fact_msg.attribute_name])[1]
        if not container_name:
            #in order the container or location doesnt have the count
            output_value.value = self.msg_to_instance_name(order, second )
        else:
            output_value.value = container_name
        #if 'SH01' == order.destination.type.data or 'SH02' == order.destination.type.data:
        #    print "not adding goal for shelfes"
        #    return
        ## For PP01 adding insert
        if 5 == order.destination.type.data :
            # Hack for PPT for Nagoya 
            #################
            #print "Not adding Goals for PP01"
            #return
            #################
            output_value.value = 'PP01_CAVITY'

        fact_msg.values.append(output_value)

        self.write_to_knowledge_base(fact_msg, self.knowledge_update_service['ADD_GOAL'] )

        return name_of_object

def main():
    rospy.init_node('refbox_parser', anonymous=True)
    refbox_parser = RefboxParser()
    refbox_parser.start()

