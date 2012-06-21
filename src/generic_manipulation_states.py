#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import math
import arm_navigation_msgs.msg
import std_srvs.srv

from grasp_object import *

from simple_script_server import *
sss = simple_script_server()

class grasp_random_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_list'])
        
    def execute(self, userdata):
        sss.move("gripper", "open", blocking=False)
       # sss.move("arm", "zeroposition")
        
        for object in userdata.object_list:         
            # ToDo: need to be adjusted to correct stuff           
            if object.pose.pose.position.z <= 0.05 and object.pose.pose.position.z >= 0.30:
                continue
                        
            handle_arm = sss.move("arm", [object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, 0, ((math.pi/2) + (math.pi/4)), 0, "/base_link"])
   
            if handle_arm.get_state() == 3:
                sss.move("gripper", "close", blocking=False)
                rospy.sleep(2.0)
                sss.move("arm", "zeroposition")        
                return 'succeeded'    
            else:
                rospy.logerr('could not find IK for current object')

        return 'failed'
        

class grasp_obj_with_visual_servering(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_to_grasp'])
        
        self.visual_serv_srv = rospy.ServiceProxy('/raw_blob_detection/start', std_srvs.srv.Empty)
    def execute(self, userdata):

        sss.move("gripper", "open")
        sss.move("arm", "pregrasp_laying_mex")
    
        print "wait for service: /raw_blob_detection/start "
        rospy.wait_for_service('/raw_blob_detection/start', 30)
    
        print "do visual serv"
        resp = self.visual_serv_srv()
        print "done"


        '''
        print userdata.object_to_grasp
        sss.move("arm", [float(userdata.object_to_grasp.pose.position.x), float(userdata.object_to_grasp.pose.position.y), (float(userdata.object_to_grasp.pose.position.z) + 0.02),"/base_link"])

        sss.move("gripper", "close")
        rospy.sleep(3)
        sss.move("arm", "zeroposition")
        '''

        grasper = Grasper()
        print("waiting 0.02 for arm joint values")
        rospy.sleep(0.05)
        grasper.simple_grasp("laying")
        print("did it work?")

        sss.move("arm","grasp_laying_mex")

    
        print "do visual serv"
        resp = self.visual_serv_srv()
        print "done"

        sss.move("gripper", "close")
        rospy.sleep(3)

        sss.move("arm", "zeroposition")
        
        return 'succeeded'


class place_obj_on_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'], 
								output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'])

    def execute(self, userdata):   
        
        sss.move("arm", "zeroposition")
        sss.move("arm", "pregrasp_back_init")
        #sss.move("arm", "pregrasp_back")
        
        if(len(userdata.rear_platform_free_poses) > 0):
            pltf_pose = userdata.rear_platform_free_poses.pop()
            sss.move("arm", pltf_pose)
        else:
            pltf_pose = [0.033 + 0.024 - 0.32, 0.0, 0.12, 0, -math.pi + 0.2, 0, "/arm_link_0"];
            sss.move("arm", [pltf_pose[0], pltf_pose[1], pltf_pose[2], pltf_pose[3], pltf_pose[4], pltf_pose[5], pltf_pose[6]])
       
        
        sss.move("gripper", "open")

        userdata.rear_platform_occupied_poses.append(pltf_pose)

        sss.move("arm", "pregrasp_back")
        
        return 'succeeded'


class grasp_obj_from_pltf(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_obj_on_pltf'], 
                                    input_keys=['rear_platform_occupied_poses'], 
                                    output_keys=['rear_platform_occupied_poses'])
        
    def execute(self, userdata):
        
        if(len(rear_platform_occupied_poses) == 0):
            return 'no_more_obj_on_pltf'
        
        
        print userdata.rear_platform_occupied_poses.pop()
        
        return 'succeeded'
    
    
class move_arm_out_of_view(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):   
        sss.move("arm", "zeroposition")
        sss.move("arm", "pregrasp_back_init")
        sss.move("arm", "pregrasp_back")
        sss.move("arm", "arm_out_of_view")
           
        return 'succeeded'
    
    
class grasp_obj_from_pltf(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_obj_on_pltf'], 
                             input_keys=['rear_platform_occupied_poses'],
                             output_keys=['rear_platform_occupied_poses'])

    def execute(self, userdata):   
        
        pltf_obj_pose = userdata.rear_platform_occupied_poses.pop()
        
        sss.move("arm", pltf_obj_pose)
        
        sss.move("gripper", "close")
        rospy.sleep(3)
        
        sss.move("arm", "initposition")
           
        return 'succeeded'


class place_object_in_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'no_more_cfg_poses'],
            input_keys=['obj_goal_configuration_poses'],
            output_keys=['obj_goal_configuration_poses'])
        
    def execute(self, userdata):
        
        if len(userdata.obj_goal_configuration_poses.pop() == 0)
            rospy.logerr("no more configuration poses")
            return 'no_more_cfg_poses'
        
        cfg_goal_pose = userdata.obj_goal_configuration_poses.pop()
        
        sss.move("arm", cfg_goal_pose)
        
        sss.move("gripper","open")
        rospy.sleep(2)
                
        return 'succeeded'
    
    
    