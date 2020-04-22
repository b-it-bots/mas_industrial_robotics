#!/usr/bin/env python
"""
This node reads information from problem.pddl file and uploads the knowledge
required for the robot to plan.
The way this is done right now is through service call using rosplan interface

First uses an existing pddl parser (taken from downward code inside mercury planner code)
and transforms into pddl vector.

Example:
    the third element of this vector looks like this:

    [':objects', 'dynamixel', '-', 'gripper', 's1', 's2', 's3', 's4',
     'start', 'exit', 'cb_ramp', 'cb_trash', 'drill_loc',
     'force_fitting', 'assembly_station', '-', 'location',
     'o1', 'o2', 'o3', 'o4', 'o5', 'o6', 'faulty_plate',
     'fixable_plate', 'filecard', 'tray', 'blue_box', 'drill',
     'trash', '-', 'object', 'youbot-brsu-3', '-', 'robot',
     'platform_middle', 'platform_left', 'platform_right', '-', 'robot_platform']

Above shown example shows the instances part of pddl problem file.
Init facts and goal follows after.

These vectors are converted into different data types which are a bit easier for service
requests and a bit easier to understand for humans.

These data types are converted into list of KnowledgeItem and KB is updated using
KnowledgeUpdateServiceArray.

"""

from __future__ import print_function

import rospy
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceArray
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest as Req
from rosplan_knowledge_msgs.srv import GetDomainAttributeService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

import mercury_planner.pddl as pddl # for parsing pddl file

class ProblemUploader(object):

    """
    Upload problem to KB
    
    Parameters:
    :problem_file: str (path of problem file)

    """

    def __init__(self, problem_file):
        self._problem_file = problem_file
        pddl_problem = pddl.pddl_file.parse_pddl_file('problem', self._problem_file)
        self._instances = ProblemUploader.parse_objects(pddl_problem[3])
        self._attr_to_obj_type = ProblemUploader.get_attr_to_obj_type(self._instances.keys())
        self._facts = self._parse_facts(pddl_problem[4])
        self._goals = self._parse_facts(pddl_problem[5][1])

    def upload(self):
        instance_ki_list = ProblemUploader.get_instance_knowledge_item_list(self._instances)
        success_1 = ProblemUploader.update_kb_array(instance_ki_list, Req.ADD_KNOWLEDGE)

        fact_ki_list = ProblemUploader.get_fact_knowledge_item_list(self._facts)
        success_2 = ProblemUploader.update_kb_array(fact_ki_list, Req.ADD_KNOWLEDGE)

        goal_ki_list = ProblemUploader.get_fact_knowledge_item_list(self._goals)
        success_3 = ProblemUploader.update_kb_array(goal_ki_list, Req.ADD_GOAL)

        return all([success_1, success_2, success_3])

    @staticmethod
    def get_instance_knowledge_item_list(instances):
        """
        Create a list of KnowledgeItem from instances

        :instances: dict {str: [str, str, ...], ...}
        :returns: list of rosplan_knowledge_msgs.KnowledgeItem

        """
        ki_list = []
        for instance_type, instance_names in instances.iteritems():
            for instance_name in instance_names:
                ki_list.append(KnowledgeItem(knowledge_type=KnowledgeItem.INSTANCE,
                                             instance_type=instance_type,
                                             instance_name=instance_name.upper()))
        return ki_list

    @staticmethod
    def get_fact_knowledge_item_list(facts):
        """
        Create a list of KnowledgeItem from facts

        :facts: list of tuple ( each tuple is (str, [(str, str), ...]) )
        :returns: list of rosplan_knowledge_msgs.KnowledgeItem

        """
        ki_list = []
        for attr_name, kv_list in facts:
            ki_list.append(KnowledgeItem(knowledge_type=KnowledgeItem.FACT,
                                         attribute_name=attr_name,
                                         values=[KeyValue(k, v.upper())
                                                 for k, v in kv_list]))
        return ki_list

    @staticmethod
    def update_kb_array(ki_list, update_type):
        """TODO: Docstring for update_kb_array.

        :ki_list: TODO
        :update_type: TODO
        :returns: TODO

        """
        update_kb_topic = '/rosplan_knowledge_base/update_array'
        rospy.loginfo('Waiting for ' + update_kb_topic)
        rospy.wait_for_service(update_kb_topic)

        try:
            update_kb = rospy.ServiceProxy(update_kb_topic, KnowledgeUpdateServiceArray)
            response = update_kb(update_type=[update_type]*len(ki_list),
                                 knowledge=ki_list)
            if response.success:
                rospy.loginfo('Upload succeeded')
            else:
                rospy.logerr('Upload failed')
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)

    @staticmethod
    def parse_objects(pddl_objects):
        """
        first element is ignored, then after a dash is the categories,
        and all the instances before are the objects belonging to that categories.

        :pddl_objects: list
        :returns: dict {str: [str, str, ...], ...}
        
        """
        # ignore first element
        objects = pddl_objects[1:]

        curr_list = []
        obj_dict = {}
        while len(objects) > 0:
            string = objects.pop(0).encode('utf-8')
            if string == '-':
                type_name = objects.pop(0).encode('utf-8')
                obj_dict[type_name] = curr_list
                curr_list = list()
            else:
                curr_list.append(string)
        return obj_dict

    def _parse_facts(self, pddl_facts):
        """
        parse facts from a list of strings

        :pddl_facts: list of str
        :returns: list of tuple ( each tuple is (str, [(str, str), ...]) )

        """
        facts = []
        for fact in pddl_facts[1:]:
            attr_name = fact[0].encode('utf-8')
            if attr_name not in self._attr_to_obj_type or \
                    len(fact)-1 != len(self._attr_to_obj_type[attr_name]):
                continue
            kv_list = [(key, value.encode('utf-8'))
                       for key, value in zip(self._attr_to_obj_type[attr_name], fact[1:])]
            facts.append((attr_name, kv_list))
        return facts

    @staticmethod
    def get_attr_to_obj_type(obj_names):
        """
        Create a dict to look up from attribute name to object keys
        For example: 'on' -> ('o', 'l')
                               ^    ^
                               |    |
                           object  location

        :obj_names: list
        :returns: dict

        """
        domain_attr_kb_topic = '/rosplan_knowledge_base/domain/predicates'
        rospy.loginfo('Waiting for ' + domain_attr_kb_topic)
        rospy.wait_for_service(domain_attr_kb_topic)
        rospy.loginfo('Wait complete.')
        response = None
        try:
            get_domain_attr = rospy.ServiceProxy(domain_attr_kb_topic, GetDomainAttributeService)
            response = get_domain_attr()
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)
            return None

        attr_to_obj_type = {}
        for item in response.items:
            attr_to_obj_type[item.name] = [param.key for param in item.typed_parameters]
        return attr_to_obj_type
    
    @staticmethod
    def make_srv_req_to_KB(knowledge_type_is_instance=True,
                           instance_type='', instance_name='',
                           attribute_name='', values=[],
                           update_type_is_knowledge=True):
        """
        make a service call to rosplan knowledge base (mongodb)

        example 1:
        # upload instance : youbot has one dynamixel gripper (dynamixel - gripper)
        rosservice call /rosplan_knowledge_base/update "update_type: 0 # ADD_KNOWLEDGE = 0
            knowledge:
            knowledge_type: 0 # INSTANCE = 0
            instance_type: 'gripper'
            instance_name: 'dynamixel'
            attribute_name: ''
            values:
            -{}
            function_value: 0.0";

        example 2: object4 (o4) is on location S3
        # upload fact :
        rosservice call /rosplan_knowledge_base/update "update_type: 0 # ADD_KNOWLEDGE = 0
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
        if knowledge_type_is_instance:
            msg.knowledge_type = KnowledgeItem.INSTANCE
            msg.instance_type = instance_type
            msg.instance_name = instance_name.upper()
        else:
            msg.knowledge_type = KnowledgeItem.FACT
            msg.attribute_name = attribute_name
            msg.values = [KeyValue(i[0], i[1].upper()) for i in values]

        update_kb_topic = '/rosplan_knowledge_base/update'
        rospy.loginfo('Waiting for ' + update_kb_topic)
        rospy.wait_for_service(update_kb_topic)

        try:
            update_kb = rospy.ServiceProxy(update_kb_topic, KnowledgeUpdateService)
            if update_type_is_knowledge:
                response = update_kb(Req.ADD_KNOWLEDGE, msg)
            else:
                response = update_kb(Req.ADD_GOAL, msg)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)

        if response.success:
            rospy.loginfo('KB updated successfully.')
            return True
        else:
            rospy.logerror('KB update failed.')
            return False
