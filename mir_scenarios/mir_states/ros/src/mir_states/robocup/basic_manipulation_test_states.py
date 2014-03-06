#!/usr/bin/env python

import tf
import rospy
import smach

import mir_states.common.manipulation_states as gms
import mir_states.common.navigation_states as gns
import mir_states.common.perception_states as gps

import moveit_commander

class load_objects(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'load_object',
                                       'find_objects'],
                             io_keys=['found_objects'],
                             input_keys=['task'],
                             output_keys=['object'])
        self.schedule = None

    def execute(self, userdata):
        if self.schedule is None:
            self.create_schedule(userdata.task)
        rospy.loginfo('Found %i objects on the workspace.' % len(userdata.found_objects))
        while userdata.found_objects:
            obj = userdata.found_objects.pop(0)
            rospy.loginfo('Object name %s, task is %s' % (obj.name, str(userdata.task.objects)))
            if obj.name in self.schedule['needed_objects']:
                self.schedule['needed_objects'].remove(obj.name)
                userdata.object = obj
                return 'load_object'
        if self.schedule['needed_objects'] and self.schedule['second_try']:
            self.schedule['second_try'] = False
            return 'find_objects'
        return 'succeeded'

    def create_schedule(self, task):
        self.schedule = dict()
        self.schedule['needed_objects'] = task.objects
        self.schedule['second_try'] = False


class unload_objects(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'unload_object'],
                             io_keys=['rear_platform'],
                             input_keys=['task'],
                             output_keys=['src_location', 'dst_location'])
        self.schedule = None

    def execute(self, userdata):
        if self.schedule is None:
            self.create_schedule(userdata.rear_platform, userdata.task)
        while self.schedule:
            src, dst = self.schedule.pop()
            userdata.src_location = src
            userdata.dst_location = dst
            return 'unload_object'
        return 'succeeded'

    def create_schedule(self, rear_platform, task):
        self.schedule = list()
        for i, l in enumerate(rear_platform.get_occupied_locations(), 1):
            self.schedule.append((l, '{0}/{0}_{1}'.format(task.cfg, i)))


class setup_move_base_to(smach.State):
    def __init__(self, key_name):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['task'],
                             output_keys=['move_base_to'])
        self.key_name = key_name

    def execute(self, userdata):
        assert hasattr(userdata.task, self.key_name)
        userdata.move_base_to = userdata.task.__getattr__(self.key_name)
        return 'succeeded'



class get_obj_poses_for_goal_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'configuration_poses_not_available'],
            input_keys=['task_spec','obj_goal_configuration_poses'],
            output_keys=['obj_goal_configuration_poses'])
        
        #FIXME: is there a moveit Group for gripper?
        self.gripper_command = moveit_commander.MoveGroupCommander('arm_1_gripper')
        
    def execute(self, userdata):
        
        self.gripper_command.set_named_target("open")
        self.gripper_command.go()
        print userdata.task_spec.object_config 
        
        if (not rospy.has_param("/script_server/arm/" + userdata.task_spec.object_config)):
            rospy.logerr("configuration <<" + userdata.task_spec.object_config + ">> NOT available on parameter server")
            return 'configuration_poses_not_available'
            
        pose_names = rospy.get_param("/script_server/arm/" + userdata.task_spec.object_config)

        print pose_names
        
        for pose_name in pose_names:
            print "cfg pose: ", pose_name
            userdata.obj_goal_configuration_poses.append((userdata.task_spec.object_config + "/" + pose_name))
    

        userdata.obj_goal_configuration_poses.sort()
        
                
        return 'succeeded'
