#!/usr/bin/env python

"""
Test unit for the functions used in upload_knowledge.py
"""

PKG = 'mcr_knowledge'

import unittest
import rosunit
import mcr_knowledge.update_knowledge_utils as utils
import mcr_knowledge_ros.upload_knowledge as upload_knowledge

class TestPddlObjectUpload(unittest.TestCase):
    def test_parse_objects(self):
        """
        Assuming that our problem.pddl file looks like the vector in pddl_objects, the expected return
        value comming from the function is described in expected, if equal then the test is passed
        """
        pddl_objects = [
            ':objects', 'dynamixel', '-', 'gripper', 's1', 's2', 's3', 's4', 'start', 'exit', 'cb_ramp', 'cb_trash', 'drill_loc',
            'force_fitting', 'assembly_station', '-', 'location', 'o1', 'o2', 'o3', 'o4', 'o5', 'o6', 'faulty_plate',
            'fixable_plate', 'filecard', 'tray', 'blue_box', 'drill', 'trash', '-', 'object', 'youbot-brsu-3', '-', 'robot',
            'platform_middle', 'platform_left', 'platform_right', '-', 'robot_platform']

        grippers = ['dynamixel']
        locations = ['s1', 's2', 's3', 's4', 'start', 'exit', 'cb_ramp', 'cb_trash', 'drill_loc',
            'force_fitting', 'assembly_station']
        objects = ['o1', 'o2', 'o3', 'o4', 'o5', 'o6', 'faulty_plate',
            'fixable_plate', 'filecard', 'tray', 'blue_box', 'drill', 'trash']
        robots = ['youbot-brsu-3']
        robot_platforms = ['platform_middle', 'platform_left', 'platform_right']

        grippers_knowledge = [
            utils.create_knowledge_unit_dict(0, utils.create_knowledge_dict(instance_type='gripper', instance_name=gripper))
            for gripper in grippers
        ]
        
        locations_knowledge = [
            utils.create_knowledge_unit_dict(0, utils.create_knowledge_dict(instance_type='location', instance_name=location))
            for location in locations
        ]
        
        objects_knowledge = [
            utils.create_knowledge_unit_dict(0, utils.create_knowledge_dict(instance_type='object', instance_name=obj))
            for obj in objects
        ]
        
        robots_knowledge = [
            utils.create_knowledge_unit_dict(0, utils.create_knowledge_dict(instance_type='robot', instance_name=robot))
            for robot in robots
        ]
        
        robot_platforms_knowledge = [
            utils.create_knowledge_unit_dict(0, utils.create_knowledge_dict(instance_type='robot_platform', instance_name=robot_platform))
            for robot_platform in robot_platforms
        ]
        
        expected = [grippers_knowledge, locations_knowledge, objects_knowledge, robots_knowledge, robot_platforms_knowledge] 
 
        result = upload_knowledge.pddl_object_list_to_dict(pddl_objects)

        self.assertEqual(result, expected, msg='result:\n{0}\n\nexpected:\n{1}'.format(result, expected))


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_pddl_object_upload', TestPddlObjectUpload)
