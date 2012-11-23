#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import math
import arm_navigation_msgs.msg

from grasp_object import *
from simple_script_server import *
sss = simple_script_server()

from tf.transformations import euler_from_quaternion
import std_srvs.srv
import hbrs_srvs.srv


planning_mode = ""            # no arm planning
#planning_mode = "planned"    # using arm planning


class Bunch:
    def __init__(self, **kwds):
         self.__dict__.update(kwds)

class is_object_grasped(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['obj_grasped', 'obj_not_grasped', 'srv_call_failed'])

        self.obj_grasped_srv_name = '/arm_1/gripper_controller/is_gripper_closed'

        self.obj_grasped_srv = rospy.ServiceProxy(self.obj_grasped_srv_name, hbrs_srvs.srv.ReturnBool)
        
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
        
        
        '''
        # transform to base_link
        try:
            tf_listener = tf.TransformListener()
        except Exception, e:
            rospy.logerr("tf exception in grasp_drawer: create transform listener: %s", e)

        tf_worked = False
        while not tf_worked:
            try:
                print "HEADER: ", userdata.drawer_pose.header.frame_id
                userdata.drawer_pose.header.stamp = rospy.Time.now()
                tf_listener.waitForTransform('/base_link', userdata.drawer_pose.header.frame_id, rospy.Time(0), rospy.Duration(5))
                obj_pose_transformed = tf_listener.transformPose('/base_link', userdata.drawer_pose)
                tf_worked = True
            except Exception, e:
                rospy.logerr("tf exception in grasp_drawer: transform: %s", e)
                rospy.sleep(0.2)
                tf_worked = False
        
        
        print "grasp drawer: ", obj_pose_transformed      
        
        # calculate a pregrasp pose
        pre_grasp_x = obj_pose_transformed.pose.position.x - 0.02
        pre_grasp_y = obj_pose_transformed.pose.position.y
        pre_grasp_z = obj_pose_transformed.pose.position.z = obj_pose_transformed.pose.position.z + 0.05
        
        print "new grasp drawer: ", obj_pose_transformed      
        
        sss.move("arm", [float(pre_grasp_x), float(pre_grasp_y), float(pre_grasp_z), "/base_link"], mode=planning_mode)
        rospy.sleep(3)
        
        sss.move("arm", [float(obj_pose_transformed.pose.position.x), float(obj_pose_transformed.pose.position.y), float(obj_pose_transformed.pose.position.z), "/base_link"], mode=planning_mode)
        '''
        
        #sss.move("arm", [0.4874590962250121, 0.0055892878817233524, -0.05027115597514983811, 0.2, 2.0, 1.57, "/base_link"])
        sss.move("arm", "drawer_prepre_grasp")
        rospy.sleep(0.5)
        sss.move("arm", "drawer_pre_grasp")
        rospy.sleep(0.5)
        sss.move("arm", "drawer_grasp")
        rospy.sleep(0.5)
        
        
        #global planning_mode
        #sss.move("arm", [0.48, 0, -0.02, 0, 3.1, 1.57, "/base_link"], mode=planning_mode)
        #sss.move("gripper", "close")
        
        
        ##sss.move("arm", "initposition")

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
    
            global planning_mode
            sss.move("arm", "zeroposition", mode=planning_mode)                             

            #object.pose.pose.position.z = object.pose.pose.position.z + 0.02
            object.pose.pose.position.x = object.pose.pose.position.x + 0.01
            object.pose.pose.position.y = object.pose.pose.position.y - 0.005

            handle_arm = sss.move("arm", [object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, "/base_link"], mode=planning_mode)

            if handle_arm.get_state() == 3:
                sss.move("gripper", "close", blocking=False)
                rospy.sleep(3.0)
                sss.move("arm", "zeroposition", mode=planning_mode)        
                return 'succeeded'    
            else:
                rospy.logerr('could not find IK for current object')

        return 'failed'
 
class grasp_obj_with_visual_servering(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'vs_timeout'], input_keys=['object_to_grasp'])
        
        self.visual_serv_srv_name = "/raw_visual_servoing/do_visual_servoing"
        self.visual_serv_srv = rospy.ServiceProxy(self.visual_serv_srv_name, hbrs_srvs.srv.ReturnBool)
    def execute(self, userdata):
        global planning_mode
        sss.move("gripper", "open")
        
        
        #sss.move("arm", "pregrasp_laying_mex", mode=planning_mode)
        #pt = userdata.object_to_grasp.pose.position
        #r = userdata.object_to_grasp.pose.orientation
        #[r, p, y] = euler_from_quaternion([r.w, r.x, r.y, r.z])
        #d = [pt.x, pt.y, pt.z, 0, 0, math.pi / 2, userdata.object_to_grasp.header.frame_id]
        sss.move("arm", "pregrasp_laying_mex", mode=planning_mode)
        #sss.move("arm", d)

        #print "wait for service: ", self.visual_serv_srv_name
        rospy.wait_for_service(self.visual_serv_srv_name, 30)
        
        visual_done = False
        print "do visual serv"
        while not visual_done:
            try:
                
                resp = self.visual_serv_srv()

                if not(resp.value):
                    rospy.logerr("visual servoing exited with timeout")
                    visual_done = False
                    sss.move("arm", "zeroposition", mode=planning_mode)
                    return 'vs_timeout'

                print "done"
                visual_done = True
            except:
                visual_done = False
        
        #print userdata.object_to_grasp
        #sss.move("arm", [float(userdata.object_to_grasp.pose.position.x), float(userdata.object_to_grasp.pose.position.y), (float(userdata.object_to_grasp.pose.position.z) + 0.02),"/base_link"])

        #sss.move("gripper", "close")
        #rospy.sleep(3)
        #sss.move("arm", "zeroposition")
        
         
        grasper = Grasper()
        print("waiting 0.02 for arm joint values")
        rospy.sleep(0.05)
        grasper.bin_grasp("laying")
        print("did it work?")
        

        sss.move("arm","grasp_laying_mex", mode=planning_mode)

    
        #print "do visual serv"
        #resp = self.visual_serv_srv()
        #print "done"
        
        sss.move("gripper", "close")
        rospy.sleep(3)

        sss.move("arm", "zeroposition", mode=planning_mode)

        return 'succeeded'


class place_obj_on_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_free_poses'], input_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'], 
								output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'])

    def execute(self, userdata):   
        global planning_mode
        
        if planning_mode != "planned":
            sss.move("arm", "zeroposition", mode=planning_mode)
            sss.move("arm", "platform_intermediate", mode=planning_mode)

        
        if(len(userdata.rear_platform_free_poses) == 0):
            rospy.logerr("NO more free poses on platform")
            return 'no_more_free_poses'
            
        pltf_pose = userdata.rear_platform_free_poses.pop()

        if planning_mode != "planned":
            sss.move("arm", pltf_pose+"_pre")

        sss.move("arm", pltf_pose, mode=planning_mode)
        
        
        sss.move("gripper", "open")
        rospy.sleep(2)
        
        print "appending to platform occuoied poses"
        userdata.rear_platform_occupied_poses.append(pltf_pose)

        if planning_mode != "planned":
            sss.move("arm", pltf_pose+"_pre")

        sss.move("arm", "platform_intermediate", mode=planning_mode)

        return 'succeeded'
    


class move_arm(smach.State):

    def __init__(self, position = "zeroposition", do_blocking = True):
        smach.State.__init__(self, outcomes=['succeeded'])
        
        self.position = position
        self.do_blocking = do_blocking

    def execute(self, userdata):
        sss.move("gripper", "open")
        rospy.sleep(2.0)
   
        global planning_mode
        sss.move("arm", self.position, mode=planning_mode, blocking = self.do_blocking)
                   
        return 'succeeded'

  
class move_arm_out_of_view(smach.State):

    def __init__(self, do_blocking = True):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.do_blocking = do_blocking

    def execute(self, userdata):   
        global planning_mode

        if planning_mode != "planned":
            sss.move("arm", "zeroposition", blocking = self.do_blocking)
    
        sss.move("arm", "arm_out_of_view", mode=planning_mode, blocking = self.do_blocking)
           
        return 'succeeded'
    
    
class grasp_obj_from_pltf(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_obj_on_pltf'], 
                             input_keys=['rear_platform_occupied_poses'],
                             output_keys=['rear_platform_occupied_poses'])

    def execute(self, userdata):   
        global planning_mode
        
        if len(userdata.rear_platform_occupied_poses) == 0:
            rospy.logerr("NO more objects on platform")
            return 'no_more_obj_on_pltf'

        pltf_obj_pose = userdata.rear_platform_occupied_poses.pop()
        
        if planning_mode != "planned":
            sss.move("arm", "platform_intermediate")
            sss.move("arm", pltf_obj_pose+"_pre")
        
        sss.move("arm", pltf_obj_pose, mode=planning_mode)
        
        sss.move("gripper", "close")
        rospy.sleep(3)

        if planning_mode != "planned":        
            sss.move("arm", pltf_obj_pose+"_pre")
            sss.move("arm", "platform_intermediate")
        
        sss.move("arm", "zeroposition", mode=planning_mode)
           
        return 'succeeded'
    
    
class place_object_in_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'no_more_cfg_poses'],
            input_keys=['obj_goal_configuration_poses'],
            output_keys=['obj_goal_configuration_poses'])
        
    def execute(self, userdata):
        global planning_mode
        
        if len(userdata.obj_goal_configuration_poses) == 0:
            rospy.logerr("no more configuration poses")
            return 'no_more_cfg_poses'
        
        cfg_goal_pose = userdata.obj_goal_configuration_poses.pop()
        print "goal pose taken: ",cfg_goal_pose
        print "rest poses: ", userdata.obj_goal_configuration_poses
        
        sss.move("arm", cfg_goal_pose, mode=planning_mode)
        
        sss.move("gripper","open")
        rospy.sleep(2)

        sss.move("arm", "zeroposition", mode=planning_mode)
                
        return 'succeeded'
