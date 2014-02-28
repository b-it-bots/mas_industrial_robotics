#!/usr/bin/python

import rospy
import smach
import smach_ros
import math
import tf
## NOTE: WHAT REPLACES arm_navigation_msgs?
##import arm_navigation_msgs.msg

from simple_script_server import *
sss = simple_script_server()

## NOTE: ANY REPLACEMENT for move_arm_cart_script_server?
##from move_arm_cart_script_server import MoveArmCartScriptServer
##arm_cart = MoveArmCartScriptServer() 

from tf.transformations import euler_from_quaternion
import std_srvs.srv
##import hbrs_srvs.srv


planning_mode = ""            # no arm planning
#planning_mode = "planned"    # using arm planning

##from arm import *
##from mir_common_states_common.common.rear_platform import *
##arm = Arm(planning_mode='')

# Gripper Wait Time
GRIPPER_WAIT_TIME = 1.8

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
            rospy.sleep(GRIPPER_WAIT_TIME)
            
            is_gripper_closed = self.obj_grasped_srv()
        except:
            rospy.logerr("could not call service  %s", self.obj_grasped_srv_name)
            return "srv_call_failed"

        print is_gripper_closed

        if is_gripper_closed.value:
            return 'obj_not_grasped'
        else:
            return 'obj_grasped'


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
            rospy.sleep(GRIPPER_WAIT_TIME)
            userdata.rear_platform.store_object(location)
            arm.move_to('platform_%s_pre' % location)
            arm.move_to('platform_intermediate')
            return 'succeeded'
        except RearPlatformFullError as a:
            return 'rear_platform_is_full'
        except ArmNavigationError as e:
            rospy.logerr('Move arm failed: %s' % (str(e)))
            return 'failed'


class pick_object_from_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'rear_platform_is_empty',
                                       'failed'],
                             io_keys=['rear_platform'],
                             input_keys=['location'])

    def execute(self, userdata):
        location = (userdata.location or
                    userdata.rear_platform.get_occupied_location())
        try:
            arm.gripper('open')
            arm.move_to('platform_intermediate')
            arm.move_to('platform_%s_pre' % location)
            arm.move_to('platform_%s' % location)
            #rospy.sleep(3)
            arm.gripper('close')
            rospy.sleep(GRIPPER_WAIT_TIME)
            arm.move_to('platform_%s_pre' % location)
            arm.move_to('platform_intermediate')
            return 'succeeded'
        except RearPlatformEmptyError as a:
            return 'rear_platform_is_empty'
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
        rospy.loginfo('MOVING ARM TO')
        try:
            arm.move_to(position, blocking=self.blocking, tolerance=self.tolerance)
        except ArmNavigationError as e:
            rospy.logerr('Move arm failed: %s' % (str(e)))
            return 'failed'
        return 'succeeded'


class control_gripper(smach.State):

    """
    Open or close gripper (depending on the value passed to the constructor).
    """

    def __init__(self, action):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.action = action

    def execute(self, userdata):
        arm.gripper(self.action)
        return 'succeeded'
       

class grasp_object(smach.State):

    """
    Should be called after visual servoing has aligned the gripper with the
    object.
    """

    FRAME_ID = '/base_link'

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'tf_error'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        arm.gripper('open')
        rospy.sleep(GRIPPER_WAIT_TIME)
        try:
            t = self.tf_listener.getLatestCommonTime('/base_link',
                                                      'gripper_finger_link')
            (p, q) = self.tf_listener.lookupTransform('/base_link',
                                                      'gripper_finger_link',
                                                      t)
            rpy = tf.transformations.euler_from_quaternion(q)
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr('Tf error: %s' % str(e))
            return 'tf_error'
        try:
            dx = rospy.get_param('script_server/arm/grasp_delta/x')
            dy = rospy.get_param('script_server/arm/grasp_delta/y')
            dz = rospy.get_param('script_server/arm/grasp_delta/z')
            #rospy.logerr('read dxyz ' + dx + ',' + dy + ',' + dz)
        except KeyError:
            rospy.logerr('No Grasp Pose Change Set.')
        arm_cart.move([['/base_link', float(p[0] - dx), float(p[1] - dy), float(p[2] - dz), rpy[0], rpy[1], rpy[2]]])
        #Will need to check if above call uses blocking.
        #rospy.sleep(1)
        arm.gripper('close')
        rospy.sleep(GRIPPER_WAIT_TIME)
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
        rospy.sleep(GRIPPER_WAIT_TIME)

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
        rospy.sleep(GRIPPER_WAIT_TIME+1.0)

        sss.move("arm", "candle", mode=planning_mode)
                
        return 'succeeded'

class compute_pregrasp_pose(smach.State):

    """
    Given an object pose compute a pregrasp position that is reachable and also
    good for the visual servoing.

    THIS DOESN'T work optimally with Visual Servoing. Moved from load_object.py 
    to here for potential future use.
    """

    FRAME_ID = '/base_link'

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'tf_error'],
                             input_keys=['object'],
                             output_keys=['move_arm_to'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        pose = userdata.object.pose
        try:
            t = self.tf_listener.getLatestCommonTime(self.FRAME_ID,
                                                     pose.header.frame_id)
            pose.header.stamp = t
            pose = self.tf_listener.transformPose(self.FRAME_ID, pose)
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr('Tf error: %s' % str(e))
            return 'tf_error'
        p = pose.pose.position
        o = pose.pose.orientation
        userdata.move_arm_to = [self.FRAME_ID,
                                p.x, p.y, p.z + 0.1,
                                0, 3.14, 0]
        return 'succeeded'
