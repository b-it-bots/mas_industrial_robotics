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

        start_obj_dicts = self._get_obj_dicts_from_workstations(task.arena_start_state)
        target_obj_dicts = self._get_obj_dicts_from_workstations(task.arena_target_state)

        obj_dicts = self._get_entire_knowledge_from_obj_dicts(start_obj_dicts, target_obj_dicts)
        # for obj_dict in obj_dicts:
        #     print(obj_dict)

        # read facts and add them to the knowledge base
        facts = self._get_facts_from_obj_dicts(obj_dicts)
        fact_ki_list = ProblemUploader.get_fact_knowledge_item_list(facts)
        fact_update_success = ProblemUploader.update_kb_array(fact_ki_list, Req.ADD_KNOWLEDGE)

        # read goals and add them to the knowledge base
        goals = self._get_goals_from_obj_dicts(obj_dicts)
        goal_ki_list = ProblemUploader.get_fact_knowledge_item_list(goals)
        goal_update_success = ProblemUploader.update_kb_array(goal_ki_list, Req.ADD_GOAL)

        # populate instances
        objects = [obj_dict["object_full_name"] for obj_dict in obj_dicts]
        locations = [workstation.workstation_name for workstation in task.arena_start_state]

        instances = {"object": objects, "location": locations}
        # print(instances)
        instance_ki_list = ProblemUploader.get_instance_knowledge_item_list(
            instances
        )
        instance_update_success = ProblemUploader.update_kb_array(
            instance_ki_list,
            Req.ADD_KNOWLEDGE
        )

        self._print_task(obj_dicts)

    def _get_entire_knowledge_from_obj_dicts(self, start_obj_dicts, target_obj_dicts):
        """TODO: 

        :param start_obj_dicts: dictionary containing object info at the start
        :type start_obj_dicts: list of dict
        :param target_obj_dicts: dictionary containing object info at the end
        :type target_obj_dicts: list of dict
        :return: obj_dicts
        :rtype: obj_dicts: list of dict

        """
        obj_dicts = []

        cavity_included = False
        for obj_dict in start_obj_dicts:
            new_obj_dict = copy.deepcopy(obj_dict)
            # remove repeated occurence of pp01_cavity
            if "cavity" in new_obj_dict["object"]:
                if cavity_included:
                    continue
                else:
                    cavity_included = True
            obj_dicts.append(new_obj_dict)

        # initialise full name in dict
        objects_count = {}
        for obj_dict in obj_dicts:
            object_name = obj_dict["object"]
            if object_name in objects_count:
                objects_count[object_name] += 1
            else:
                objects_count[object_name] = 1
            object_full_name = AtworkCommanderClient.get_object_full_name(
                    object_name, objects_count[object_name])
            obj_dict["object_full_name"] = object_full_name

        # set target for decoys and heavy objects
        for obj_dict in obj_dicts:
            if obj_dict["decoy"]:
                obj_dict["target"] = obj_dict["location"]

            if "container" in obj_dict["object"] or "cavity" in obj_dict["object"]:
                obj_dict["target"] = obj_dict["location"]

        # match target obj dicts with obj dicts
        for target_obj_dict in target_obj_dicts:
            if target_obj_dict["decoy"]:
                continue

            if "container" in target_obj_dict["object"] or\
                  "cavity" in target_obj_dict["object"]:
                continue

            obj_dict_index = self._find_obj_dict_with(
                    obj_dicts, target=target_obj_dict["target"],
                    object_name=target_obj_dict["object"])
            if obj_dict_index == -1:
                rospy.logwarn("Did not find matching object for " +\
                        target_obj_dict["object"])
                continue

            if target_obj_dict["target"] == "empty":
                obj_dicts[obj_dict_index]["target"] = target_obj_dict["location"]
            elif "container" in target_obj_dict["target"]:
                container_obj_dict_index = self._find_obj_dict_with(obj_dicts,
                        location=target_obj_dict["location"],
                        object_name=target_obj_dict["target"])
                if container_obj_dict_index == -1:
                    rospy.logwarn("Did not find a container " +\
                            target_obj_dict["target"] + " on " +\
                            target_obj_dict["location"] + " for " +\
                            target_obj_dict["object"])
                    continue
                obj_dicts[obj_dict_index]["target"] = \
                        obj_dicts[container_obj_dict_index]["object_full_name"]
            elif "cavity" in target_obj_dict["target"]:
                cavity_obj_dict_index = self._find_obj_dict_with(obj_dicts,
                        object_name=target_obj_dict["target"])
                if cavity_obj_dict_index == -1:
                    rospy.logwarn("Did not find a cavity " + target_obj_dict["target"] +\
                                  " for " + target_obj_dict["object"])
                    continue
                obj_dicts[obj_dict_index]["target"] = \
                        obj_dicts[cavity_obj_dict_index]["object_full_name"]
        return obj_dicts

    def _find_obj_dict_with(self, obj_dicts, object_name=None, object_full_name=None,
                            decoy=None, location=None, target=None):
        for i, obj_dict in enumerate(obj_dicts):
            bool_list = []
            bool_list.append(object_name is None or object_name == obj_dict["object"])
            bool_list.append(object_full_name is None or object_full_name == obj_dict["object_full_name"])
            bool_list.append(decoy is None or decoy == obj_dict["decoy"])
            bool_list.append(location is None or location == obj_dict["location"])
            bool_list.append(target is None or target == obj_dict["target"])
            if all(bool_list):
                return i
        return -1


    def _get_facts_from_obj_dicts(self, obj_dicts):
        """TODO: Docstring for _get_objects_and_locations.

        :param obj_dicts: dictionary containing object info 
        :type obj_dicts: list of dict
        :return: facts
        :rtype: list (list [str, [[str, str], ...]] )

        example of returned facts
            [
                ['on', [['o', 'r20'], ['l', 'SH01']]],
                ['on', [['o', 'bearing'], ['l', 'CB01']]],
                ['on', [['o', 'm30-00'], ['l', 'WS01']]],
            ]
        """
        facts = []
        for obj_dict in obj_dicts:
            if "container" in obj_dict["object"] or "cavity" in obj_dict["object"]:
                facts.append(self._get_fact_from_attr_and_values(
                    "container",
                    [obj_dict["object_full_name"]]))
                facts.append(self._get_fact_from_attr_and_values(
                    "heavy",
                    [obj_dict["object_full_name"]]))

            if "container" in obj_dict["target"] or "cavity" in obj_dict["target"]:
                facts.append(self._get_fact_from_attr_and_values(
                    "insertable",
                    [obj_dict["object_full_name"]]))

            facts.append(self._get_fact_from_attr_and_values(
                "on",
                [obj_dict["object_full_name"], obj_dict["location"]])
            )

        return facts

    def _get_goals_from_obj_dicts(self, obj_dicts):
        """TODO: 

        :param obj_dicts: dictionary containing object info 
        :type obj_dicts: list of dict
        :return: facts
        :rtype: list (list [str, [[str, str], ...]] )

        example of returned goals
            [
                ['on', [['o', 'r20'], ['l', 'SH01']]],
                ['on', [['o', 'bearing'], ['l', 'CB01']]],
                ['on', [['o', 'm30-00'], ['l', 'WS01']]],
            ]
        """
        goals = []
        for obj_dict in obj_dicts:
            if obj_dict["target"] == obj_dict["location"]:
                continue

            if obj_dict["target"] == "empty":
                continue

            if "container" in obj_dict["target"] or "cavity" in obj_dict["target"]:
                goals.append(self._get_fact_from_attr_and_values(
                    "in",
                    [obj_dict["object_full_name"], obj_dict["target"]])
                )
            else:
                goals.append(self._get_fact_from_attr_and_values(
                    "on",
                    [obj_dict["object_full_name"], obj_dict["target"]])
                )

        return goals


    def _get_obj_dicts_from_workstations(self, workstations):
        """TODO: 

        :param workstations: workstations from task message
        :type workstations: list (atwork_commander_msgs.msg.Workstation)
        :return: object strings
        :rtype: list (dict)

        """
        obj_dicts = []
        for workstation in workstations:
            for obj in workstation.objects:

                if obj.object not in self._obj_code_to_name:
                    rospy.logwarn("Could not find " + str(obj.object) + " in object codes")
                    continue

                if self._cavity_start_code <= obj.object < self._cavity_end_code:
                    object_name = "pp01_cavity"
                else:
                    object_name = self._obj_code_to_name[obj.object]

                if obj.target not in self._obj_code_to_name:
                    rospy.logwarn("Could not find " + str(obj.target) + " in object codes")
                    continue

                if self._cavity_start_code <= obj.target < self._cavity_end_code:
                    target_name = "pp01_cavity"
                else:
                    target_name = self._obj_code_to_name[obj.target]

                obj_dict = {
                        "object": object_name,
                        "location": workstation.workstation_name,
                        "target": target_name,
                        "decoy": obj.decoy
                }
                obj_dicts.append(obj_dict)
        return obj_dicts

    def _get_fact_from_attr_and_values(self, attr_name, values):
        """TODO: 

        :param attr_name: name of attribute
        :type attr_name: str
        :param values: values for key value pair
        :type values: list [str, ...]
        :return: fact
        :rtype: list [str, [[str, str], ...]]

        example of returned fact
                ['on', [['o', 'r20'], ['l', 'SH01']]],
        """
        kv_list = [
            [key, value.encode("utf-8")]
            for key, value in zip(self._attr_to_obj_type[attr_name], values)
        ]
        return [attr_name, kv_list]

    def _print_task(self, obj_dicts):
        string = ""
        for obj_dict in obj_dicts:
            if obj_dict["target"] == obj_dict["location"]:
                continue

            if obj_dict["target"] == "empty":
                continue

            string += obj_dict["object_full_name"].ljust(20) + " : "
            string += obj_dict["location"] + " -> " + obj_dict["target"].ljust(20)

            if "container" in obj_dict["target"] or "cavity" in obj_dict["target"]:
                container_obj_dict_index = self._find_obj_dict_with(obj_dicts,
                        object_full_name=obj_dict["target"])
                if container_obj_dict_index != -1:
                    string += " (" + obj_dicts[container_obj_dict_index]["location"] + ")"
            string += "\n"
        print(string.upper())

    @staticmethod
    def get_object_full_name(object_name, count):
        """TODO: 

        :param object_name: name of object
        :type object_name: str
        :param count: unique num to be added as suffix
        :type count: int
        :return: object_full_name
        :rtype: str

        """
        name = ""
        if object_name == "container_red":
            name = "container_box_red"
        elif object_name == "container_blue":
            name = "container_box_blue"
        else:
            name = object_name
        return name + "-" + str(count-1).zfill(2)
