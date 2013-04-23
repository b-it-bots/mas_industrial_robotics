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

from move_arm_cart_script_server import MoveArmCartScriptServer
arm_cart = MoveArmCartScriptServer() 

from tf.transformations import euler_from_quaternion
import std_srvs.srv
import hbrs_srvs.srv


planning_mode = ""            # no arm planning
#planning_mode = "planned"    # using arm planning

from arm import *
from rear_platform import *
arm = Arm(planning_mode='')


class Bunch:
    def __init__(self, **kwds):
         self.__dict__.update(kwds)

class is_object_grasped(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['obj_grasped', 'obj_not_grasped', 'srv_call_failed'])

        self.obj_grasped_srv_name = '/gripper_controller/is_gripper_closed'

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



class grasp_random_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_list'])
        
    def execute(self, userdata):
        sss.move("gripper", "open", blocking=False)
       # sss.move("arm", "candle")
        
        for object in userdata.object_list:         
            
            # ToDo: need to be adjusted to correct stuff           
            if object.pose.pose.position.z <= 0.0 or object.pose.pose.position.z >= 0.10:
                continue
    
            global planning_mode
            sss.move("arm", "candle", mode=planning_mode)                             

            #object.pose.pose.position.z = object.pose.pose.position.z + 0.02
            object.pose.pose.position.x = object.pose.pose.position.x + 0.01
            object.pose.pose.position.y = object.pose.pose.position.y - 0.005

            handle_arm = arm_cart.move([["/base_link", object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, 0.0, ((math.pi/2) + (math.pi/4)), 0.0, 0.0, 0.5, 0.0]])

            if handle_arm.get_result().error_code.val == arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS:
                sss.move("gripper", "close", blocking=False)
                rospy.sleep(3.0)
                sss.move("arm", "candle", mode=planning_mode)        
                return 'succeeded'    
            else:
                rospy.logerr('could not find IK for current object')

        return 'failed'


class put_object_on_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'rear_platform_is_full',
                                       'failed'],
                             io_keys=['rear_platform'])

    def execute(self, userdata):
        try:
            location = userdata.rear_platform.get_free_location()
            arm.move_to('candle')
            arm.move_to('platform_intermediate')
            arm.move_to('platform_%s_pre' % location)
            arm.move_to('platform_%s' % location)
            arm.gripper('open')
            rospy.sleep(3)
            userdata.rear_platform.store_object(location)
            arm.move_to('platform_%s_pre' % location)
            arm.move_to('platform_intermediate')
            return 'succeeded'
        except RearPlatformFullError as a:
            return 'rear_platform_is_full'
        except ArmNavigationError as e:
            rospy.logerr('Move arm failed: %s' % (str(e)))
            return 'failed'


class move_arm(smach.State):

    """
    Move arm to a position. Position may be fixed at construction time or set
    through userdata.

    Input
    -----
    move_arm_to: str | tuple | list
        Position where the arm should move. If it is a string, then it gives
        position name (should be availabile on the parameter server). If it as
        tuple or a list, then it is treated differently based on the length. If
        there are 5 elements, then it is a list of joint values. If the length
        is 3 or 4, then it is cartesian position and pitch angle.
    """

    def __init__(self, position=None, blocking=True, tolerance=None):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['move_arm_to'])
        self.move_arm_to = position
        self.blocking = blocking
        self.tolerance = tolerance

    def execute(self, userdata):
        position = self.move_arm_to or userdata.move_arm_to
        try:
            arm.move_to(position, blocking=self.blocking, tolerance=self.tolerance)
        except ArmNavigationError as e:
            rospy.logerr('Move arm failed: %s' % (str(e)))
            return 'failed'
        return 'succeeded'


class grasp_object(smach.State):

    """
    Should be called after visual servoing has aligned the gripper with the
    object.
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'])

    def execute(self, userdata):
        arm.gripper('open')
        rospy.sleep(2)
        # TODO: move arm down
        arm.gripper('close')
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
        
        sss.move("arm", "candle", mode=planning_mode)
           
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

        sss.move("arm", "candle", mode=planning_mode)
                
        return 'succeeded'
