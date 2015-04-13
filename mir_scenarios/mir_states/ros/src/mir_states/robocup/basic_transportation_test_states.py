#!/usr/bin/python

import rospy

import smach
import smach_ros
import tf

from geometry_msgs.msg import PoseStamped
from mir_navigation_msgs.msg import OrientToBaseAction, OrientToBaseActionGoal

from actionlib.simple_action_client import GoalStatus

import mir_states.common.manipulation_states as manipulation

import math

def print_task_spec(task_list):
    
    rospy.loginfo("task spec")
    for task in task_list:
        rospy.loginfo("      %s %s %s ", task.location, task.object_names, task.type)
    
    return

def print_occupied_platf_poses(poses):
    
    rospy.loginfo("occupied_platform poses: ")
    for item in poses:
        rospy.loginfo("     %s", item.obj.name)
    
    return



class Bunch:
    def __init__(self, **kwds):
         self.__dict__.update(kwds)

class select_object_to_be_grasped(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['obj_selected', 'no_obj_selected','no_more_free_poses_at_robot_platf'],
            input_keys=['recognized_objects', 'objects_to_be_grasped', 'object_to_grasp', 'rear_platform_free_poses','object_to_be_adjust_to'],
            output_keys=['object_to_grasp', 'object_to_be_adjust_to'])
        
    def execute(self, userdata):
        
        if(len(userdata.rear_platform_free_poses) == 0):
            rospy.logerr("NO more free poses on robot rear platform")
            return 'no_more_free_poses_at_robot_platf'

        
        for rec_obj in userdata.recognized_objects:
            for obj_grasp in userdata.objects_to_be_grasped:
                if rec_obj.name == obj_grasp:
                    userdata.object_to_grasp = rec_obj
                    userdata.object_to_be_adjust_to = rec_obj.pose
                    print "selected obj: ", userdata.object_to_grasp.name
                    return 'obj_selected'
                
        return 'no_obj_selected'

class select_btt_subtask(smach.State):
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['task_selected','no_more_task_for_given_type'],
            input_keys=['task_list','base_pose_to_approach','objects_to_be_grasped'],
            output_keys=['base_pose_to_approach', 'objects_to_be_grasped'])
        
        self.type = type
        
    def execute(self, userdata):
        
        # print_task_spec(userdata.task_list)

        # check if there is a empty obj_names list for a given location and remove it
        for i in range(len(userdata.task_list)):
            if len(userdata.task_list[i].object_names) == 0 and userdata.task_list[i].type == 'source': 
                userdata.task_list.pop(i)     
                break;     
        for i in range(len(userdata.task_list)):
            if len(userdata.task_list[i].object_names) == 0 and userdata.task_list[i].type == 'destination': 
                userdata.task_list.pop(i)   
                break;                     
        print_task_spec(userdata.task_list)
              
        selected = 'false'  
        # select next task
        for i in range(len(userdata.task_list)):
            if userdata.task_list[i].type == self.type:     
                userdata.base_pose_to_approach = userdata.task_list[i].location  
                userdata.objects_to_be_grasped = userdata.task_list[i].object_names          
                selected_task_index = i
                selected = 'true'
                break;

        if selected == 'true':
            temp_task_list = userdata.task_list
           
            for i in range(len(userdata.task_list)):
                if userdata.task_list[i].type == "destination" and userdata.base_pose_to_approach == userdata.task_list[i].location:
                    for destobj in range(len(userdata.task_list[i].object_names)):                      
                        for graspobj in range(len(userdata.task_list[selected_task_index].object_names)):  
                            print "source objects",userdata.task_list[selected_task_index].object_names[0]
                            print "dest object set",userdata.task_list[i].object_names[0]                           
                            print "grasp obj",graspobj
                            print "dest obj",destobj
                            if (userdata.task_list[i].object_names[destobj] == userdata.task_list[selected_task_index].object_names[graspobj]):    
                                userdata.task_list[selected_task_index].object_names.pop(graspobj)  
                                userdata.task_list[i].object_names.pop(destobj)
                                userdata.objects_to_be_grasped = userdata.task_list[selected_task_index].object_names  
                            print_task_spec(userdata.task_list)
                            break;                                                
                                         
                                         
            return 'task_selected'
            
        return 'no_more_task_for_given_type'
                     

class get_obj_poses_for_goal_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'configuration_poses_not_available'],
            input_keys=['task_spec','obj_goal_configuration_poses'],
            output_keys=['obj_goal_configuration_poses'])
       
    def execute(self, userdata):
        
        manipulation.gripper_command.set_named_target("open")
        
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


class select_delivery_workstation(smach.State):
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['success', 'no_more_dest_tasks'],
            input_keys=['rear_platform_occupied_poses', 'task_list'],
            output_keys=['base_pose_to_approach', 'objects_to_be_grasped', 'objects_goal_configuration'])
        
        
    def execute(self, userdata):
        print_task_spec(userdata.task_list)
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        
        for task in userdata.task_list:
            for platform_item in userdata.rear_platform_occupied_poses:
                if task.type == 'destination':
                    for dest in task.object_names:
                        if dest == platform_item.obj.name:
                            userdata.base_pose_to_approach = task.location
                            userdata.objects_goal_configuration = task.object_config
                            return 'success'
        '''                
        if len(userdata.task_list) > 0:
            userdata.base_pose_to_approach = task.location
            userdata.objects_goal_configuration = task.object_config
            return 'success'
        '''
        
        return 'no_more_dest_tasks'
                        

class setup_task(smach.State):
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['success'],
            input_keys=['task_list', 'destinaton_free_poses','source_visits'],
            output_keys=['destinaton_free_poses','source_visits'])
        
        
    def execute(self, userdata):
        print_task_spec(userdata.task_list)
        
        for task in userdata.task_list:
            if task.type == 'destination':
                poses = ['3','3','1', '2', '3']
                loc_free_poses = Bunch(location = task.location, free_poses = poses)
                userdata.destinaton_free_poses.append(loc_free_poses)
            if task.type == 'source':
                source_loc_visits = Bunch(location = task.location, visits = 0)
                userdata.source_visits.append(source_loc_visits)
                
        print_task_spec(userdata.task_list)
        return 'success'
        
        
class grasp_obj_from_pltf_btt(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['object_grasped', 'no_more_obj_for_this_workspace'], 
                             input_keys=['rear_platform_occupied_poses', 'rear_platform_free_poses', 'base_pose_to_approach', 'task_list', 'last_grasped_obj'],
                             output_keys=['rear_platform_occupied_poses', 'rear_platform_free_poses', 'last_grasped_obj'])
       
    def execute(self, userdata):   
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        print_task_spec(userdata.task_list)
        
        #get objects to be placed at current workstation
        objs_for_this_ws = []
        for task in userdata.task_list:
            if task.type == 'destination' and task.location == userdata.base_pose_to_approach:
                objs_for_this_ws = task.object_names
        
        pltf_obj_pose = 0
        stop = False
        for i in range(len(userdata.rear_platform_occupied_poses)):
            for obj_name in objs_for_this_ws:
                rospy.loginfo("userdata.rear_platform_occupied_poses[i].obj.name: %s     obj_name: %s", userdata.rear_platform_occupied_poses[i].obj.name, obj_name)
                if userdata.rear_platform_occupied_poses[i].obj.name == obj_name:
                    pltf_obj_pose =  userdata.rear_platform_occupied_poses.pop(i)
                    userdata.rear_platform_free_poses.append(pltf_obj_pose)
                    userdata.last_grasped_obj = pltf_obj_pose.obj
                    print "LAST OBJ: ", userdata.last_grasped_obj.name
                    stop = True
                    break;
            if stop:
                break
                
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)        
        
        if pltf_obj_pose == 0:
            return 'no_more_obj_for_this_workspace'
        
       
        print "plat_pose: ", pltf_obj_pose.platform_pose
        print "plat_name: ", pltf_obj_pose.obj.name

        manipulation.arm_command.set_named_target(str(pltf_obj_pose.platform_pose)+"_pre")
        manipulation.arm_command.go()
            
        manipulation.arm_command.set_named_target(str(pltf_obj_pose.platform_pose))
        manipulation.arm_command.go()
        
        manipulation.gripper_command.set_named_target("close")
        manipulation.gripper_command.go()
        
        manipulation.arm_command.set_named_target(str(pltf_obj_pose.platform_pose)+"_pre")
        manipulation.arm_command.go()

        return 'object_grasped'


class place_object_in_configuration_btt(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'no_more_cfg_poses'],
            input_keys=['base_pose_to_approach', 'objects_goal_configuration', 'last_grasped_obj', 'task_list', 'destinaton_free_poses', 'obj_goal_configuration_poses'],
            output_keys=['destinaton_free_poses', 'task_list'])
               
    def execute(self, userdata):
        
        print_task_spec(userdata.task_list)
        
        if len(userdata.destinaton_free_poses) == 0:
            rospy.logerr("no more configuration poses")
            return 'no_more_cfg_poses'
        
        pose = 0
        for i in range(len(userdata.destinaton_free_poses)):
            if userdata.destinaton_free_poses[i].location == userdata.base_pose_to_approach:
                pose = userdata.destinaton_free_poses[i].free_poses.pop()
        
        
        cfg_goal_pose = userdata.objects_goal_configuration + "/" + userdata.objects_goal_configuration + "_" + pose
        print "goal pose taken: ",cfg_goal_pose
        print "rest poses: ", userdata.destinaton_free_poses[i].free_poses
                
        manipulation.arm_command.set_named_target(cfg_goal_pose)
        manipulation.arm_command.go()
        
        manipulation.gripper_command.set_named_target("open")
        manipulation.gripper_command.go()
                
        #delete placed obj from task list
        for j in range(len(userdata.task_list)):
            if userdata.task_list[j].type == 'destination' and userdata.task_list[j].location == userdata.base_pose_to_approach:
                print "lllll: ", userdata.last_grasped_obj.name
                print "list: ", userdata.task_list[j].object_names 
                userdata.task_list[j].object_names.remove(userdata.last_grasped_obj.name)
                
                if len(userdata.task_list[j].object_names) == 0:
                    userdata.task_list.pop(j)
                
                break
            
        print_task_spec(userdata.task_list)
        
    
        return 'succeeded'

#FIXME: replace by generic state
class place_obj_on_rear_platform_btt(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_free_poses'], 
                                   input_keys=['object_to_grasp', 'rear_platform_free_poses', 'rear_platform_occupied_poses', 'task_list', 'base_pose_to_approach'], 
                                   output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses', 'task_list'])
       
    def execute(self, userdata):   
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        
        print_task_spec(userdata.task_list)
        
        if(len(userdata.rear_platform_free_poses) == 0):
            rospy.logerr("NO more free poses on platform")
            return 'no_more_free_poses'

        # Removing pre poses (Do we need these poses?)
        pltf_pose = userdata.rear_platform_free_poses.pop();

        manipulation.arm_command.set_named_target(pltf_pose.platform_pose+"_pre")
        manipulation.arm_command.go()
            
        manipulation.arm_command.set_named_target(pltf_pose.platform_pose)
        manipulation.arm_command.go()
            
        manipulation.gripper_command.set_named_target("open")
        manipulation.gripper_command.go()
        
        print "object_to_grasp: ", userdata.object_to_grasp.name
        #delete from task list
        for i in range(len(userdata.task_list)):
            if userdata.task_list[i].type == 'source' and userdata.task_list[i].location == userdata.base_pose_to_approach:
                print "obj_names:", userdata.task_list[i].object_names
                userdata.task_list[i].object_names.remove(userdata.object_to_grasp.name)
                print "obj_names after remove from task list: ", userdata.task_list[i].object_names
                break
        
        print_task_spec(userdata.task_list)
        
        # remember what is on the platform
        pltf_pose.obj = userdata.object_to_grasp
        userdata.rear_platform_occupied_poses.append(pltf_pose)
        
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        
        manipulation.arm_command.set_named_target(pltf_pose.platform_pose+"_pre")
        manipulation.arm_command.go()
            
        return 'succeeded'



class check_if_platform_has_still_objects(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['still_objs_on_robot_pltf', 'no_more_objs_on_robot_pltf'],
                                   input_keys=['rear_platform_occupied_poses','source_visits','task_list'],
                                   output_keys=['source_visits','lasttask'])

    def execute(self, userdata):   
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        
        if len(userdata.rear_platform_occupied_poses) == 0:
            for j in range(len(userdata.source_visits)):
                for task in userdata.task_list:
                    source_loc_visits = Bunch(location = task.location, visits = 0)
                    userdata.source_visits.append(source_loc_visits)
            userdata.lasttask = Bunch(location="", obj_names="")
            return 'no_more_objs_on_robot_pltf'


        return 'still_objs_on_robot_pltf'


class skip_pose(smach.State):

    def __init__(self, type=""):
        smach.State.__init__(self, outcomes=['pose_skipped','pose_skipped_but_platform_limit_reached'], 
                                   input_keys=['task_list', 'base_pose_to_approach','source_visits','rear_platform_occupied_poses','lasttask'],
                                   output_keys=['task_list','source_visits','lasttask'])
        self.type = type

    def execute(self, userdata):   
        
        print_task_spec(userdata.task_list)
        
        for i in range(len(userdata.task_list)):
            if userdata.task_list[i].type == self.type and userdata.task_list[i].location == userdata.base_pose_to_approach:
                userdata.lasttask = userdata.task_list.pop(i)
                userdata.task_list.append(userdata.lasttask)              
                sum_visits = 0
                for j in range(len(userdata.source_visits)):
                    if userdata.base_pose_to_approach == userdata.source_visits[j].location:
                        userdata.source_visits[j].visits=userdata.source_visits[j].visits+1
                    sum_visits = sum_visits+userdata.source_visits[j].visits

                if sum_visits >= len(userdata.source_visits) and len(userdata.rear_platform_occupied_poses) > 0:
                    for j in range(len(userdata.source_visits)):
                        for task in userdata.task_list:
                            source_loc_visits = Bunch(location = task.location, visits = 0)
                            userdata.source_visits.append(source_loc_visits)
                    return 'pose_skipped_but_platform_limit_reached'
                else:
                    return 'pose_skipped'
            
        print_task_spec(userdata.task_list)
                
        return 'pose_skipped'

class transform_pose_to_reference_frame(smach.State):

    def __init__(self, frame_id=None):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'tf_error'],
                             input_keys=['object_pose'],
                             output_keys=['object_pose'])
        self.tf_listener = tf.TransformListener()
        self.frame_id = frame_id

    def execute(self, userdata):

        pose = userdata.object_pose.pose
     
        print 'object_pose', pose

        try:
            t = self.tf_listener.getLatestCommonTime(self.frame_id,
                                                     pose.header.frame_id)
            pose.header.stamp = t
            transformed_pose = self.tf_listener.transformPose(self.frame_id, pose)

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr('Tf error: %s' % str(e))
            return 'tf_error'

        userdata.object_pose.pose = transformed_pose

        print 'transformed_pose:', transformed_pose

        return 'succeeded'

class compute_base_shift_to_object_old(smach.State):

    FRAME_ID = '/base_link'

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'tf_error'],
                             input_keys=['object_pose','move_base_by'],
                             output_keys=['move_base_by'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        pose = userdata.object_pose.pose

        try:
            t = self.tf_listener.getLatestCommonTime(self.FRAME_ID,
                                                     pose.header.frame_id)
            pose.header.stamp = t
            relative = self.tf_listener.transformPose(self.FRAME_ID, pose)

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr('Tf error: %s' % str(e))
            return 'tf_error'
       
        userdata.move_base_by = (0, relative.pose.position.y, 0)

        return 'succeeded'

class compute_base_shift_to_object(smach.State):

    def __init__(self, target_frame=None, source_frame=None):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'tf_error'],
                             input_keys=['object_pose','move_base_by'],
                             output_keys=['move_base_by'])
        self.listener = tf.TransformListener()
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.br = tf.TransformBroadcaster()

    def execute(self, userdata):
        pose = userdata.object_pose.pose.pose
        rospy.sleep(2.0)

        try:
            self.listener.waitForTransform(
                self.target_frame, self.source_frame,
                rospy.Time(0), rospy.Duration(0.1)
            )

            (trans,rot) = self.listener.lookupTransform(self.target_frame, 
                self.source_frame, rospy.Time(0))

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr('Tf error: %s' % str(e))
            return 'tf_error'

        base_shift_x_direction = pose.position.x - trans[0] - 0.05

        base_shift_y_direction = pose.position.y - trans[1]

        base_shift_z_direction = pose.position.z - trans[2]

        if (base_shift_x_direction > 0.25):
           rospy.logwarn("Base shift in x direction is greater than 25 cm")
           base_shift_x_direction = 0.0
        userdata.move_base_by = (base_shift_x_direction, base_shift_y_direction, 0.0)
        print 'userdata.move_base_by' , userdata.move_base_by 
        return 'succeeded'

class loop_for(smach.State):
    '''
    This state will return 'loop' MAX+1 times.
    On the MAX execute, 'continue' is returned.
    '''
    def __init__(self, max_loop_count=2):
        smach.State.__init__(self, outcomes=['loop', 'continue'],
                             input_keys=['vscount'],
                             output_keys=['vscount'])
        self.max_loop_count = max_loop_count
        self.loop_count = 0
       
    def execute(self, userdata):
        self.loop_count = userdata.vscount
        if self.loop_count <= self.max_loop_count:
            rospy.loginfo('run number: %d' % self.loop_count)
            userdata.vscount = userdata.vscount + 1
            return 'loop'
        else:
            userdata.vscount = 0
            manipulation.arm_command.set_named_target("platform_intermediate")
            manipulation.arm_command.go()

            return 'continue'  

