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
        self._cavity_start_code = getattr(Object, "CAVITY_START")
        self._cavity_end_code = getattr(Object, "CAVITY_END")
        
    def _task_cb(self, msg):
        task = copy.deepcopy(msg)
        if task.id in self._processed_task_ids: # repeated task message
            return

        self._processed_task_ids.append(task.id)

        # populate instances
        objects = self._get_objects_from_workstations(task.arena_start_state)
        locations = [workstation.workstation_name for workstation in task.arena_start_state]
        print("objects", objects)
        print("locations", locations)
        instances = {"object": objects, "location": locations}
        instance_ki_list = ProblemUploader.get_instance_knowledge_item_list(
            instances
        )
        instance_update_success = ProblemUploader.update_kb_array(
            instance_ki_list,
            Req.ADD_KNOWLEDGE
        )

        start_obj_dicts = self._get_obj_dicts_from_workstations(task.arena_start_state)
        target_obj_dicts = self._get_obj_dicts_from_workstations(task.arena_target_state)
        for obj_dict in start_obj_dicts:
            print(obj_dict)
        # read facts and add them to the knowledge base
        facts, valid_objects = self._get_facts_from_start_obj_dicts(start_obj_dicts)
        fact_ki_list = ProblemUploader.get_fact_knowledge_item_list(facts)
        fact_update_success = ProblemUploader.update_kb_array(fact_ki_list, Req.ADD_KNOWLEDGE)
        print("facts:")
        for fact in facts:
            print(fact)
        print(valid_objects)

        # read goals and add them to the knowledge base
        goals = self._get_goals_from_target_obj_dicts_and_valid_objects(target_obj_dicts, valid_objects)
        goal_ki_list = ProblemUploader.get_fact_knowledge_item_list(goals)
        goal_update_success = ProblemUploader.update_kb_array(goal_ki_list, Req.ADD_GOAL)
        print("goals:")
        for goal in goals:
            print(goal)


    def _get_facts_from_start_obj_dicts(self, start_obj_dicts):
        """TODO: Docstring for _get_objects_and_locations.

        :param start_obj_dicts: dictionary containing object info 
        :type start_obj_dicts: list of dict
        :return: facts and valid objects (ones that are not decoy)
        :rtype: tuple ( list (list [str, [[str, str], ...]] ), list (str) )

        example of returned facts
            [
                ['on', [['o', 'r20'], ['l', 'SH01']]],
                ['on', [['o', 'bearing'], ['l', 'CB01']]],
                ['on', [['o', 'm30-00'], ['l', 'WS01']]],
            ]
        """
        valid_objects = []
        objects_count = {}
        facts = []
        for obj_dict in start_obj_dicts:
            object_name = obj_dict["object"]
            if object_name in objects_count:
                objects_count[object_name] += 1
            else:
                objects_count[object_name] = 1
            object_full_name = AtworkCommanderClient.get_object_full_name(
                    object_name, objects_count[object_name])

            # TODO: remove this
            if "container" in object_full_name:
                continue

            if not obj_dict["decoy"]:
                valid_objects.append(object_full_name)

            attr_name = "on" # FIXME: deduce from object code and context
            fact = [object_full_name, obj_dict["location"]]
            kv_list = [
                [key, value.encode("utf-8")]
                for key, value in zip(self._attr_to_obj_type[attr_name], fact)
            ]
            facts.append([attr_name, kv_list])
        return facts, valid_objects

    def _get_goals_from_target_obj_dicts_and_valid_objects(self, target_obj_dicts, valid_objects):
        """TODO: Docstring for _get_objects_and_locations.

        :param target_obj_dicts: dictionary containing object info 
        :type target_obj_dicts: list of dict
        :param valid_objects: Objects that are not decoy
        :type valid_objects: list (str)
        :return: facts
        :rtype: list (list [str, [[str, str], ...]] )

        example of returned goals
            [
                ['on', [['o', 'r20'], ['l', 'SH01']]],
                ['on', [['o', 'bearing'], ['l', 'CB01']]],
                ['on', [['o', 'm30-00'], ['l', 'WS01']]],
            ]
        """
        objects_count = {}
        goals = []
        for obj_dict in target_obj_dicts:
            if obj_dict["decoy"]:
                continue

            object_name = obj_dict["object"]
            if object_name in objects_count:
                objects_count[object_name] += 1
            else:
                objects_count[object_name] = 1
            object_full_name = AtworkCommanderClient.get_object_full_name(
                    object_name, objects_count[object_name])

            # TODO: remove this
            if "container" in object_full_name:
                continue

            while True:
                object_full_name = AtworkCommanderClient.get_object_full_name(
                        object_name, objects_count[object_name])
                if object_full_name in valid_objects:
                    break

                if objects_count[object_name] > 20: # Sanity check. Should never succeed
                    rospy.logerr("Count exceeded 100 for " + object_name +\
                                 ". There is something wrong")
                    break
                objects_count[object_name] += 1

            attr_name = "on" # FIXME: deduce from object code and context
            fact = [object_full_name, obj_dict["location"]]
            kv_list = [
                [key, value.encode("utf-8")]
                for key, value in zip(self._attr_to_obj_type[attr_name], fact)
            ]
            goals.append([attr_name, kv_list])
        return goals

    def _get_obj_dicts_from_workstations(self, workstations):
        obj_dicts = []
        for workstation in workstations:
            for obj in workstation.objects:

                # ignore cavities
                if self._cavity_start_code <= obj.object < self._cavity_end_code:
                    continue

                if obj.object not in self._obj_code_to_name:
                    rospy.logwarn("Could not find " + str(obj.object) + " in object codes")
                    continue
                object_name = self._obj_code_to_name[obj.object]

                # TODO: check target
                target_name = None

                obj_dict = {
                        "object": object_name,
                        "location": workstation.workstation_name,
                        "target": target_name,
                        "decoy": obj.decoy
                        }
                obj_dicts.append(obj_dict)
        return obj_dicts

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
                object_full_name = AtworkCommanderClient.get_object_full_name(object_name, objects_count[object_name])
                objects.append(object_full_name)
        return objects

    @staticmethod
    def get_object_full_name(object_name, count):
        return object_name + "-" + str(count-1).zfill(2)
