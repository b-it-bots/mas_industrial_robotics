#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import math
import arm_navigation_msgs.msg

from simple_script_server import *
sss = simple_script_server()

import std_srvs.srv
import raw_srvs.srv




class is_object_grasped(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['obj_grasped', 'obj_not_grasped', 'srv_call_failed'])

        self.obj_grasped_srv_name = '/arm_1/gripper_controller/is_gripper_closed'

        self.obj_grasped_srv = rospy.ServiceProxy(self.obj_grasped_srv_name, raw_srvs.srv.ReturnBool)
        
    def execute(self, userdata):   
                
        try:
            rospy.loginfo("wait for service:  %s", self.obj_grasped_srv_name)
            rospy.wait_for_service(self.obj_grasped_srv_name, 5)
        
            sss.move("gripper", "close")
            rospy.sleep(2.0)
            
            is_gripper_closed = self.obj_grasped_srv()
        except:
            rospy.logerr("could not call service  %s", self.obj_grasped_srv_name)
            return "srv_call_failed"

        print is_gripper_closed

        if is_gripper_closed.value:
            return 'obj_not_grasped'
        else:
            return 'obj_grasped'


class place_object_in_drawer(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):   
        
        
        
        return 'succeeded'


class grasp_drawer(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['drawer_pose'])

    def execute(self, userdata):   

        # ToDo: sample range for gripper orientation
        #sss.move("arm", [userdata.drawer_pose.pose.position.x, userdata.drawer_pose.pose.position.y, userdata.drawer_pose.pose.position.z,
        #                0, 3.14, 0, 
        #                "/base_link"])

        sss.move("gripper", "open")
        sss.move("arm", [0.48, 0, -0.02, 0, 3.1, 1.57, "/base_link"])
        sss.move("gripper", "close")

        return 'succeeded'
   
   
class grasp_random_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_list'])
        
    def execute(self, userdata):
        sss.move("gripper", "open", blocking=False)
       # sss.move("arm", "zeroposition")
        
        for object in userdata.object_list:         
            
            # ToDo: need to be adjusted to correct stuff           
            if object.pose.pose.position.z <= 0.0 or object.pose.pose.position.z >= 0.10:
                continue
    
            sss.move("arm", "zeroposition")                             

            #object.pose.pose.position.z = object.pose.pose.position.z + 0.02
            object.pose.pose.position.x = object.pose.pose.position.x + 0.01
            object.pose.pose.position.y = object.pose.pose.position.y - 0.005

            handle_arm = sss.move("arm", [object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, "/base_link"])

            if handle_arm.get_state() == 3:
                sss.move("gripper", "close", blocking=False)
                rospy.sleep(3.0)
                sss.move("arm", "zeroposition")        
                return 'succeeded'    
            else:
                rospy.logerr('could not find IK for current object')

        return 'failed'
 
class grasp_obj_with_visual_servering(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_to_grasp'])
        
        self.visual_serv_srv = rospy.ServiceProxy('/raw_visual_servoing/start', std_srvs.srv.Empty)
    def execute(self, userdata):

        sss.move("gripper", "open")
        sss.move("arm", "pregrasp_laying_mex")
    
        print "wait for service: /raw_visual_servoing/start "
        rospy.wait_for_service('/raw_visual_servoing/start', 30)
        
        visual_done = False
        while not visual_done:
            try:
                print "do visual serv"
                resp = self.visual_serv_srv()
                print "done"
                visual_done = True
            except:
                visual_done = False

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

    
        #print "do visual serv"
        #resp = self.visual_serv_srv()
        #print "done"

        sss.move("gripper", "close")
        rospy.sleep(3)

        sss.move("arm", "zeroposition")

        return 'succeeded'


class place_obj_on_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_free_poses'], input_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'], 
								output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'])

    def execute(self, userdata):   
        
        sss.move("arm", "zeroposition")
        sss.move("arm", "platform_intermediate")

        
        if(len(userdata.rear_platform_free_poses) == 0):
            rospy.logerr("NO more free poses on platform")
            return 'no_more_free_poses'
            
        pltf_pose = userdata.rear_platform_free_poses.pop()
        # untested
        sss.move("arm", pltf_pose+"_pre")
        #
        sss.move("arm", pltf_pose)
        
        
        sss.move("gripper", "open")
        rospy.sleep(2)

        userdata.rear_platform_occupied_poses.append(pltf_pose)
        #untested
        sss.move("arm", pltf_pose+"_pre")
        sss.move("arm", "platform_intermediate")

        return 'succeeded'
    


class move_arm(smach.State):

    def __init__(self, position = "zeroposition"):
        smach.State.__init__(self, outcomes=['succeeded'])
        
        self.position = position

    def execute(self, userdata):   
        sss.move("arm", self.position)
                   
        return 'succeeded'

  
class move_arm_out_of_view(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):   
        sss.move("arm", "zeroposition")
        sss.move("arm", "arm_out_of_view")
           
        return 'succeeded'
    
    
class grasp_obj_from_pltf(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_obj_on_pltf'], 
                             input_keys=['rear_platform_occupied_poses'],
                             output_keys=['rear_platform_occupied_poses'])

    def execute(self, userdata):   

        if len(userdata.rear_platform_occupied_poses) == 0:
            rospy.logerr("NO more objects on platform")
            return 'no_more_obj_on_pltf'

        pltf_obj_pose = userdata.rear_platform_occupied_poses.pop()
        
        sss.move("arm", "platform_intermediate")
        # untested
        sss.move("arm", pltf_obj_pose+"_pre")
        #
        sss.move("arm", pltf_obj_pose)
        
        sss.move("gripper", "close")
        rospy.sleep(3)
        
        # untested
        sss.move("arm", pltf_obj_pose+"_pre")
        #
        sss.move("arm", "platform_intermediate")
        sss.move("arm", "zeroposition")
           
        return 'succeeded'
    
    
class place_object_in_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'no_more_cfg_poses'],
            input_keys=['obj_goal_configuration_poses'],
            output_keys=['obj_goal_configuration_poses'])
        
    def execute(self, userdata):
        
        if len(userdata.obj_goal_configuration_poses) == 0:
            rospy.logerr("no more configuration poses")
            return 'no_more_cfg_poses'
        
        cfg_goal_pose = userdata.obj_goal_configuration_poses.pop()
        print "goal pose taken: ",cfg_goal_pose
        print "rest poses: ", userdata.obj_goal_configuration_poses
        
        sss.move("arm", cfg_goal_pose)
        
        sss.move("gripper","open")
        rospy.sleep(2)
                
        return 'succeeded'
