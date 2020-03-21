#!/usr/bin/env python
import rospy

# for reading arguments comming into this function, in this case is for receiving
# the path to pddl problem file
import sys

# pddl parser, reads pddl file and outputs pddl vector
import mercury_planner.pddl as pddl

# create dictionary out of pddl vector
import mir_knowledge.update_knowledge_utils as utils

# for rosplan service calls
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

# to ADD_FACTS
from rosplan_knowledge_msgs.srv import rosplan_knowledge_msgs

# for the event_in callback
from std_msgs.msg import String

"""
This node reads information from problem.pddl file and uploads the knowledge
required for the robot to plan.
The way this is done right now is through mongo service call using rosplan interface

first uses an existing pddl parser (taken from downward code inside mercury planner code) and transforms into pddl vector:

the third element of this vector looks like this:

pddl_objects = [
            ':objects', 'dynamixel', '-', 'gripper', 's1', 's2', 's3', 's4', 'start', 'exit', 'cb_ramp', 'cb_trash', 'drill_loc',
            'force_fitting', 'assembly_station', '-', 'location', 'o1', 'o2', 'o3', 'o4', 'o5', 'o6', 'faulty_plate',
            'fixable_plate', 'filecard', 'tray', 'blue_box', 'drill', 'trash', '-', 'object', 'youbot-brsu-3', '-', 'robot',
            'platform_middle', 'platform_left', 'platform_right', '-', 'robot_platform']

then this vector is converted into a dictinary by using the external ros independant function utils() called by the local
pddl_object_list_to_dict() function

this dictionary is then sent por doing individual service calls via rosplan_sc_add_knowledge() function
"""

def rosplan_sc_add_knowledge(knowledge_type, instance_type, instance_name, attribute_name, values, function_value=0.0, update_type='KNOWLEDGE'):
    """
    rosplan_sc_add_knowledge = rosplan service call add knowledge

    This function receives knowledge parameters and does a service call
    to rosplan knowledge base (mongodb)

    example 1:
    # upload instance : youbot has one dynamixel gripper (dynamixel - gripper)
    rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0 # ADD_KNOWLEDGE = 0
        knowledge:
        knowledge_type: 0 # INSTANCE = 0
        instance_type: 'gripper'
        instance_name: 'dynamixel'
        attribute_name: ''
        values:
        -{}
        function_value: 0.0";
    # (on o4 S3)

    example 2: object4 (o4) is on location S3
    # upload fact :
    rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0 # ADD_KNOWLEDGE = 0
        knowledge:
        knowledge_type: 1 # FACT = 1
        instance_type: ''
        instance_name: ''
        attribute_name: 'on'
        values:
        - {key: 'o', value: 'o4'}
        - {key: 'l', value: 'S3'}
        function_value: 0.0";

    """
    msg = KnowledgeItem()
    if knowledge_type == 0:
        # instance
        msg.knowledge_type = KnowledgeItem.INSTANCE # INSTANCE (rosplan_knowledge_msgs/msg/KnowledgeItem.msg)
        msg.instance_type = instance_type
        msg.instance_name = instance_name.upper()
        msg.attribute_name = ''
    elif knowledge_type == 1:
        # fact
        msg.knowledge_type = KnowledgeItem.FACT # FACT (rosplan_knowledge_msgs/msg/KnowledgeItem.msg)
        msg.instance_type = ''
        msg.instance_name = ''
        msg.attribute_name = attribute_name
        for each in values:
            msg.values.append(KeyValue(each[0], each[1].upper()))
    else:
        rospy.logerr('error: no information will be uploaded, Knowledge type should be INSTANCE = 0, or FACT = 1')
        return False
    msg.function_value = function_value
    rospy.loginfo('======msg=========')
    rospy.loginfo(msg)
    print msg
    rospy.loginfo('================')
    rospy.loginfo('Waiting for /kcl_rosplan/update_knowledge_base to become available...')
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')
    try:
        update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base', KnowledgeUpdateService)
        #get response
        if update_type == 'KNOWLEDGE':
            response = update_kb(rosplan_knowledge_msgs.srv.KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, msg)
        elif update_type == 'GOAL':
            response = update_kb(rosplan_knowledge_msgs.srv.KnowledgeUpdateServiceRequest.ADD_GOAL, msg)
        else:
            rospy.logerr('Error : Unknown update_type, admisible values are KNOWLEDGE and GOAL')
            rospy.logwarn('Knowledge base not updated...!')
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s'%e)
    if(response.success):
        rospy.loginfo('Knowledge base successfully updated')
        # service call succesful
        return True
    else:
        rospy.logerror('Could not update world model, failed')
        # service call failed
        return False


def upload_instances(pddl_instances_as_dictionary):
    rospy.loginfo("Uploading pddl objects from specified problem pddl to knowledge base")
    for each in pddl_instances_as_dictionary:
        for element in each:
            rospy.logdebug('-------- uploading unit knowledge ---------')
            update_type = element.get('update_type')
            knowledge = element.get('knowledge')
            rospy.logdebug(update_type)
            rospy.logdebug(knowledge)
            rospy.logdebug(knowledge.get('instance_name'))
            rospy.logdebug(knowledge.get('instance_type'))
            rospy.logdebug(knowledge.get('attribute_name'))
            rospy.logdebug(knowledge.get('knowledge_type'))
            rospy.logdebug(knowledge.get('function_value'))
            rospy.logdebug('-----------------')
            # remove i_ from instances : example r_youbot-brsu-3 -> youbot-brsu-3
            try:
                rospy.logdebug(knowledge.get('instance_name').split('--')[1])
                rosplan_sc_add_knowledge(0, knowledge.get('instance_type'), knowledge.get('instance_name').split('--')[1], '', [])
            except:
                rospy.logerr('Error ocurred, please make sure that you are correctly using key--object and that no negative preconditions are present')
                return


def compute_key_values(args):
    key_value_list = []
    for each in args[1:]:
        try:
            key_value_list.append(each.split('--'))
        except:
            rospy.logerr('Error ocurred, please make sure that you are correctly using key--object and that no negative preconditions are present')
            return
    return key_value_list


def upload_facts(pddl_init_vector):
    rospy.loginfo("Uploading init predicates from minimum_required_info_problem.pddl file into knowledge base")
    for each in pddl_init_vector[1:]:
        key_values = compute_key_values(each)
        # sending for service call
        rosplan_sc_add_knowledge(1, '', '', each[0], key_values)


def upload_goals(pddl_goal_vector):
    """
    This function is here just for the sake of completition
    Is not needed, since goals are uploaded by refere box or a human, or some other master component
    """
    rospy.loginfo("Uploading goal predicates from minimum_required_info_problem.pddl file into knowledge base")
    for each in pddl_goal_vector[1][1:]:
        key_values = compute_key_values(each)
        rosplan_sc_add_knowledge(1, '', '', each[0], key_values, update_type='GOAL')


def upload_knowledge(pddl_vector):
    # identify cathegory
    pddl_cathegory = pddl_vector[0]
    if pddl_cathegory == ':objects':
        upload_instances(pddl_object_list_to_dict(pddl_vector))
    elif pddl_cathegory == ':init':
        upload_facts(pddl_vector)
    elif pddl_cathegory ==':goal':
        upload_goals(pddl_vector)
    else:
        rospy.logerr('Error : could not identify cathegory, admissible values are :objects, :init and :goal')


def pddl_object_list_to_dict(pddl_objects):
    """
    transforms the output of the pddl parser into a dictionary, that later on will be used
    to upload the knowledge via service calls
    """
    categories, objects = utils.parse_objects(pddl_objects)

    pddl_problem_dict = []
    for ii, category in enumerate(categories):
        pddl_problem_dict.append([
            utils.create_knowledge_unit_dict(0, utils.create_knowledge_dict(instance_type=category, instance_name=obj))
            for obj in objects[ii]
        ])

    return pddl_problem_dict


def getPddlProblem():
    pddl_problem = None
    if len(sys.argv) == 4: # 1 argument was received : ok proceed
        try:
            path = sys.argv[1]
            rospy.loginfo('Using minimum problem.pddl file : ' + path)
            pddl_problem = pddl.pddl_file.parse_pddl_file('problem', path)
        except:
            rospy.logerr('Could not open .pddl file !')
    else:
        rospy.logerr('No argument received!')
        rospy.logwarn('Usage : rosrun mir_knowledge upload_knowledge.py path_to_pddl_problem_file/problem.ppdl')
        rospy.logwarn('This node is recommended to be launched by using the launchfile provided in this package, are you doing that?')
    return pddl_problem


def eventInCallBack(data):
    if data.data == 'e_trigger':
        pddl_problem = getPddlProblem()
        if pddl_problem == None:
            return
        # pddl_problem[3] -> objects, pddl_problem[4] -> init, pddl_problem[5] -> goals
        # instances
        upload_knowledge(pddl_problem[3])
        # facts
        upload_knowledge(pddl_problem[4])
        # goals
        upload_knowledge(pddl_problem[5])
    else:
        rospy.logerr('Error : event_in not recognized, admisible values are : e_trigger')


def main():
    rospy.init_node('upload_knowledge')
    rospy.loginfo('Upload knowledge node initialized...')
    rospy.Subscriber("~event_in", String, eventInCallBack)
    rospy.loginfo('Waiting for event_in to upload knowledge')
    # wait for ctrl + c, don't kill the process
    rospy.spin()
