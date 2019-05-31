#!/usr/bin/python

import rospy

import smach

import mir_states.robocup.basic_transportation_test_states as btts
import smach_ros

import tf

import std_msgs.msg
import geometry_msgs.msg
import mas_perception_msgs.msg
import moveit_msgs.msg
import math

import mir_states.common.manipulation_states as manipulation

def print_all_found_holes(holes):
    rospy.loginfo("all_found_holes: ")
    for hole in holes:
        rospy.loginfo("      %s", hole.name)

class check_platform_type(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['ppt','btt'],
            input_keys=['base_pose_to_approach', 'ppt_platform_location'])

    def execute(self, userdata):
        if userdata.base_pose_to_approach == userdata.ppt_platform_location:
            rospy.loginfo("at PPT platform")
            return 'ppt'

        rospy.loginfo("at BTT platform")
        return 'btt'


class select_hole(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['hole_selected','no_match','no_more_obj_for_this_workspace'],
            input_keys=['selected_hole_pose','selected_hole','rear_platform_occupied_poses', 'task_list', 'base_pose_to_approach', 'all_found_holes'],
            output_keys=['selected_hole_pose','selected_hole'])

    def execute(self, userdata):
        btts.print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        btts.print_task_spec(userdata.task_list)

        print_all_found_holes(userdata.all_found_holes)

        #get objects to be placed at current workstation
        objs_for_this_ws = []
        for task in userdata.task_list:
            if task.type == 'destination' and task.location == userdata.base_pose_to_approach:
                objs_for_this_ws = task.object_names

        # filter out all objects, that the robot is currently carrying AND that must be placed at the current platform
        stop = False
        list_of_carried_objects_that_must_be_placed = []
        for i in range(len(userdata.rear_platform_occupied_poses)):
            for obj_name in objs_for_this_ws:
                rospy.loginfo("userdata.rear_platform_occupied_poses[i].obj.name: %s     obj_name: %s", userdata.rear_platform_occupied_poses[i].obj.name, obj_name)
                if userdata.rear_platform_occupied_poses[i].obj.name == obj_name:
                    list_of_carried_objects_that_must_be_placed.append(obj_name)
                    stop = True
                    break;
            if stop:
                break

        if len(list_of_carried_objects_that_must_be_placed) == 0:
            return 'no_more_obj_for_this_workspace'

        # from the objects, that the robot is carrying AND which must be placed at the current platform,
        # filter out those objects, for which there is a hole in the current platform
        for obj_name in list_of_carried_objects_that_must_be_placed:
            for hole in userdata.all_found_holes:
                rospy.loginfo("hole: %s     obj_name: %s", hole, obj_name)
                if hole.name in obj_name:
                    userdata.selected_hole = hole
                    return 'hole_selected'

        # no carried object matched any of the found holes
        return 'no_match'


# select all objects from the rear platform that needs to be placed at the PPT workspace
class select_objects_to_place(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['objects_selected','no_more_obj_for_this_workspace'],
            input_keys=['rear_platform_occupied_poses', 'task_list', 'base_pose_to_approach'],
            output_keys=['selected_objects'])

    def execute(self, userdata):
        btts.print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        btts.print_task_spec(userdata.task_list)

        #get objects to be placed at current workstation
        objs_for_this_ws = []
        for task in userdata.task_list:
            if task.type == 'destination' and task.location == userdata.base_pose_to_approach:
                objs_for_this_ws = task.object_names

        # filter out all objects, that the robot is currently carrying AND that must be placed at the current platform
        stop = False
        list_of_carried_objects_that_must_be_placed = []
        for i in range(len(userdata.rear_platform_occupied_poses)):
            for obj_name in objs_for_this_ws:
                rospy.loginfo("userdata.rear_platform_occupied_poses[i].obj.name: %s     obj_name: %s", userdata.rear_platform_occupied_poses[i].obj.name, obj_name)
                if userdata.rear_platform_occupied_poses[i].obj.name == obj_name:
                    list_of_carried_objects_that_must_be_placed.append(userdata.rear_platform_occupied_poses[i].obj)

        if len(list_of_carried_objects_that_must_be_placed) == 0:
            return 'no_more_obj_for_this_workspace'

        userdata.selected_objects = list_of_carried_objects_that_must_be_placed
        return 'objects_selected'


# select one object to place based on recognized cavities
# publish pose of the selected object/selected cavity
class select_object_to_place(smach.State):
    OBJECT_POSE_TOPIC='/mir_states/object_selector/object_pose'
    
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['object_selected','no_more_cavities', 'no_more_objects'],
            input_keys=['rear_platform_occupied_poses', 'cavity_pose', 'found_cavities'],
            output_keys=['selected_object','cavity_pose'])

        self.object_pose_pub = rospy.Publisher(self.OBJECT_POSE_TOPIC, geometry_msgs.msg.PoseStamped)

    def execute(self, userdata):
        if len(userdata.found_cavities) == 0:
            return 'no_more_cavities'
        else:
            look_for = userdata.found_cavities[-1].object_name
            for i in range(len(userdata.rear_platform_occupied_poses)):
                if userdata.rear_platform_occupied_poses[i].obj.name == look_for:
                    userdata.selected_object = userdata.rear_platform_occupied_poses[i].obj
                    userdata.cavity_pose = userdata.found_cavities[-1]
                    del userdata.found_cavities[-1]
                    self.object_pose_pub.publish(userdata.cavity_pose.pose)
                    print "Selected %s to place" %userdata.rear_platform_occupied_poses[i].obj.name
                    return 'object_selected'
            return 'no_more_objects'



class grasp_obj_for_hole_from_pltf(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['object_grasped', 'no_more_obj_for_this_workspace'],
            input_keys=['last_grasped_obj','rear_platform_free_poses','rear_platform_occupied_poses','selected_object', 'task_list', 'base_pose_to_approach'],
            output_keys=['last_grasped_obj','rear_platform_free_poses','rear_platform_occupied_poses', 'task_list'])

    def execute(self, userdata):
        btts.print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        #rospy.loginfo('userdata.selected_object: %s', userdata.selected_object)

        pltf_obj_pose = None
        for i in range(len(userdata.rear_platform_occupied_poses)):
            rospy.loginfo("userdata.rear_platform_occupied_poses[i].obj.name: %s", userdata.rear_platform_occupied_poses[i].obj.name)
            if  userdata.selected_object.name in userdata.rear_platform_occupied_poses[i].obj.name:
                pltf_obj_pose = userdata.rear_platform_occupied_poses.pop(i)
                userdata.rear_platform_free_poses.append(pltf_obj_pose)
                userdata.last_grasped_obj = pltf_obj_pose.obj
                rospy.loginfo("LAST OBJ: %s", userdata.last_grasped_obj.name)
                break

        btts.print_occupied_platf_poses(userdata.rear_platform_occupied_poses)

        #delete placed obj from task list
        for j in range(len(userdata.task_list)):
            if userdata.task_list[j].type == 'destination' and userdata.task_list[j].location == userdata.base_pose_to_approach:
                print "lllll: ", userdata.last_grasped_obj.name
                print "list: ", userdata.task_list[j].object_names 
                userdata.task_list[j].object_names.remove(userdata.last_grasped_obj.name)
                
                if len(userdata.task_list[j].object_names) == 0:
                    userdata.task_list.pop(j)
                
                break
            
        btts.print_task_spec(userdata.task_list)

        if not pltf_obj_pose:
            return 'no_more_obj_for_this_workspace'


        print "plat_pose: ", pltf_obj_pose.platform_pose
        print "plat_name: ", pltf_obj_pose.obj.name

        manipulation.gripper_command.set_named_target("open")
        manipulation.gripper_command.go()

        manipulation.arm_command.set_named_target(str(pltf_obj_pose.platform_pose) + "_pre")
        manipulation.arm_command.go()

        manipulation.arm_command.set_named_target(str(pltf_obj_pose.platform_pose))
        manipulation.arm_command.go()

        manipulation.gripper_command.set_named_target("close")
        manipulation.gripper_command.go()

        manipulation.arm_command.set_named_target(str(pltf_obj_pose.platform_pose) + "_pre")
        manipulation.arm_command.go()

        return 'object_grasped'

class select_arm_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['arm_pose_selected'],
            input_keys=['move_arm_to','selected_objects'],
            output_keys=['move_arm_to'])

    def execute(self, userdata):
        '''
        # TODO: Transform to base_link
        pose = userdata.selected_hole.pose
        rpy = tf.transformations.euler_from_quaternion(pose.pose.orientation)
        yaw = rpy[2]
        pi_4 = math.pi / 4.0

        rospy.loginfo('Hole yaw: %d', yaw)

        if -pi_4 <= yaw <= pi_4:
            rospy.loginfo('Using horizontal placement')
            manipulation.arm_command.set_named_target('place_horizontal')
        else:
            rospy.loginfo('Using vertical placement')
            manipulation.arm_command.set_named_target('place_vertical')
        '''
        userdata.move_arm_to = 'line/line_2'

        return 'arm_pose_selected'

class clear_cavities(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded'],
            input_keys=['found_cavities', 'best_matched_cavities'],
            output_keys=['found_cavities', 'best_matched_cavities'])

    def execute(self, userdata):
        userdata.found_cavities = []
        userdata.best_matched_cavities = []
        return 'succeeded'

class ppt_wiggle_arm(smach.State):

    def __init__(self, wiggle_offset=0.0):
        smach.State.__init__(self,
                             outcomes=['succeeded','failed'])
        self.wiggle_offset = wiggle_offset
        self.blocking = True

    def execute(self, userdata):
        joint_values = manipulation.arm_command.get_current_joint_values()
        joint_values[0] = joint_values[0] + self.wiggle_offset
        try:
            manipulation.arm_command.set_joint_value_target(joint_values)
        except Exception as e:
            rospy.logerr('unable to set target position: %s' % (str(e)))
            return 'failed'

        error_code = manipulation.arm_command.go(wait=self.blocking)

        if error_code == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return 'succeeded'
        else:
            rospy.logerr("Arm movement failed with error code: %d", error_code)
            return 'failed'

