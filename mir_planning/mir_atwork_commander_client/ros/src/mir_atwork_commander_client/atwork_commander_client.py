#! /usr/bin/env python

from __future__ import print_function

import copy
import rospy

from atwork_commander_msgs.msg import Task, Object
from mir_knowledge_ros.problem_uploader import ProblemUploader
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest as Req

class AtworkCommanderClient(object):

    """TODO: Docstring for AtworkCommanderClient. """

    def __init__(self):
        self._task_sub = rospy.Subscriber("~task", Task, self._task_cb)
        self._processed_task_ids = []
        self._attr_to_obj_type = ProblemUploader.get_attr_to_obj_type()

        # read object codes from Object.msg class
        self._obj_code_to_name = {}
        object_class_attributes = dir(Object)
        for attr in object_class_attributes:
            if attr.isupper() and "START" not in attr and "END" not in attr:
                self._obj_code_to_name[getattr(Object, attr)] = attr.lower()
        
    def _task_cb(self, msg):
        task = copy.deepcopy(msg)
        if task.id in self._processed_task_ids: # repeated task message
            return

        self._processed_task_ids.append(task.id)

        # populate instances
        objects = self._get_objects_from_workstations(task.arena_start_state)
        locations = [workstation.workstation_name for workstation in task.arena_start_state]
        # print("objects", objects)
        # print("locations", locations)
        instances = {"object": objects, "location": locations}
        instance_ki_list = ProblemUploader.get_instance_knowledge_item_list(
            instances
        )
        instance_update_success = ProblemUploader.update_kb_array(
            instance_ki_list,
            Req.ADD_KNOWLEDGE
        )

        # read facts and add them to the knowledge base
        facts = self._get_facts_from_workstations(task.arena_start_state)
        fact_ki_list = ProblemUploader.get_fact_knowledge_item_list(facts)
        fact_update_success = ProblemUploader.update_kb_array(fact_ki_list, Req.ADD_KNOWLEDGE)

        # read goals and add them to the knowledge base
        goals = self._get_facts_from_workstations(task.arena_target_state)
        goal_ki_list = ProblemUploader.get_fact_knowledge_item_list(goals)
        goal_update_success = ProblemUploader.update_kb_array(goal_ki_list, Req.ADD_GOAL)


    def _get_facts_from_workstations(self, workstations):
        """TODO: Docstring for _get_objects_and_locations.

        :param workstations: workstations from task message
        :type workstations: list (atwork_commander_msgs.msg.Workstation)
        :return: facts
        :rtype: list (list [str, [[str, str], ...]] )

        example returned object
            [
                ['on', [['o', 'r20'], ['l', 'SH01']]],
                ['on', [['o', 'bearing'], ['l', 'CB01']]],
                ['on', [['o', 'm30-00'], ['l', 'WS01']]],
            ]
        """
        objects_count = {}
        facts = []
        for workstation in workstations:
            for obj in workstation.objects:
                if obj.object not in self._obj_code_to_name:
                    rospy.logwarn("Could not find " + str(obj.object) + " in object codes")
                    continue

                object_name = self._obj_code_to_name[obj.object]
                if object_name in objects_count:
                    objects_count[object_name] += 1
                else:
                    objects_count[object_name] = 1
                object_full_name = object_name + "-" + str(objects_count[object_name]-1).zfill(2)

                attr_name = "on" # FIXME: deduce from object code and context
                fact = [object_full_name, workstation.workstation_name]
                kv_list = [
                    [key, value.encode("utf-8")]
                    for key, value in zip(self._attr_to_obj_type[attr_name], fact)
                ]
                facts.append([attr_name, kv_list])
        return facts

    def _get_objects_from_workstations(self, workstations):
        """TODO: Docstring for _get_objects_from_workstations.

        :param workstations: workstations from task message
        :type workstations: list (atwork_commander_msgs.msg.Workstation)
        :return: object strings
        :rtype: list (str)

        """
        objects_count = {}
        for workstation in workstations:
            for obj in workstation.objects:
                if obj.object not in self._obj_code_to_name:
                    rospy.logwarn("Could not find " + str(obj.object) + " in object codes")
                    continue

                object_name = self._obj_code_to_name[obj.object]
                if object_name in objects_count:
                    objects_count[object_name] += 1
                else:
                    objects_count[object_name] = 1
        objects = []
        for object_name in objects_count:
            for i in range(objects_count[object_name]):
                object_full_name = object_name + "-" + str(objects_count[object_name]-1).zfill(2)
                objects.append(object_full_name)
        return objects
