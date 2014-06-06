#!/usr/bin/python

import rospy
import smach
import smach_ros
import math
import tf

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg

arm_command = moveit_commander.MoveGroupCommander('arm_1')
arm_command.set_goal_position_tolerance(0.01)
arm_command.set_goal_orientation_tolerance(0.01)
arm_command.set_goal_joint_tolerance(0.005)


gripper_command = moveit_commander.MoveGroupCommander('arm_1_gripper')

from tf.transformations import euler_from_quaternion
import std_srvs.srv

from mcr_perception_msgs.msg import ObjectList, Object


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
            joint_positions = gripper_command.get_current_joint_values()
            
            rospy.logerr("is_object_grasped-state need to be checked!!!!!!")
            is_gripper_closed = joint_positions[0] + joint_positions[1] < 0.01
            
        except:
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
            
            #FIXME: do we need the intermediate positions with MoveIt?
            #arm_command.set_named_target("candle")
            #arm_command.go()
            
            #arm_command.set_named_target("platform_intermediate")
            #arm_command.go()
            
            #arm_command.set_named_target('platform_%s_pre' % location)
            #arm_command.go()
            
            arm_command.set_named_target('platform_%s' % location)
            arm_command.go()
            
            gripper_command.set_named_target('open')
            gripper_command.go()
            
            arm_command.set_named_target('platform_%s_pre' % location)
            arm_command.go()
            
            #arm_command.set_named_target("platform_intermediate")
            #arm_command.go()
            
            userdata.rear_platform.store_object(location)
            
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
            #FIXME: do we need the intermediate positions with MoveIt?
            
            gripper_command.set_named_target('open')
            gripper_command.go()
            
            #arm_command.set_named_target("platform_intermediate")
            #arm_command.go()
            
            #arm_command.set_named_target('platform_%s_pre' % location)
            #arm_command.go()
            
            arm_command.set_named_target('platform_%s' % location)
            arm_command.go()
            
            gripper_command.set_named_target('close')
            gripper_command.go()
            
            arm_command.set_named_target('platform_%s_pre' % location)
            arm_command.go()
            
            #arm_command.set_named_target("platform_intermediate")
            #arm_command.go()
            
            return 'succeeded'
        except RearPlatformEmptyError as a:
            return 'rear_platform_is_empty'
        except ArmNavigationError as e:
            rospy.logerr('Move arm failed: %s' % (str(e)))
            return 'failed'


class move_arm(smach.State):

    """
    Move arm to a target. target may be fixed at construction time or set
    through userdata.

    Input
    -----
    move_arm_to: str | tuple | list
        target where the arm should move. If it is a string, then it gives
        target name (should be availabile on the parameter server). If it as
        tuple or a list, then it is treated differently based on the length. If
        are 7 items, then it is cartesian pose (x, y, z, r, p ,y) + the corresponding frame
    """

    def __init__(self, target=None, blocking=True, tolerance=None):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['move_arm_to'])
        self.move_arm_to = target
        self.blocking = blocking
        self.tolerance = tolerance
        
    def execute(self, userdata):
        target = self.move_arm_to or userdata.move_arm_to

        if type(target) is str:         # target is a string specifing a joint position
            rospy.loginfo('MOVING ARM TO: ' + str(target))
            try:
                arm_command.set_named_target(target)
            except Exception as e:
                rospy.logerr('unable to set target position: %s' % (str(e)))
                return 'failed'

        elif type(target) is list:      # target is a list ...
            if len(target) == 7:        # ... of 7 items: Cartesian pose (x, y, z, r, p, y, frame_id)
                try:
                    pose = geometry_msgs.msg.PoseStamped()
                    pose.header.frame_id = target[6]
                    pose.pose.position.x = float(target[0])
                    pose.pose.position.y = float(target[1])
                    pose.pose.position.z = float(target[2])
                    
                    q = tf.transformations.quaternion_from_euler(target[3], target[4], target[5])
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]

                    arm_command.set_pose_target(pose)
                except Exception as e:
                    rospy.logerr('unable to set target position: %s' % (str(e)))
                    return 'failed'
            else:
                rospy.logerr("target list is malformed")
                return 'failed'
        else:
            rospy.logerr("no valid target specified. Target should be a string (name of a joint position) or a list of 7 items (Cartesian Pose + frame id)")
            return 'failed'

        # plan and execute arm movement        
        error_code = arm_command.go(wait=self.blocking)

        if error_code == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return 'succeeded'
        else:
            rospy.logerr("Arm movement failed with error code: %d", error_code)
            return 'failed'


class control_gripper(smach.State):

    """
    Open or close gripper (depending on the value passed to the constructor).
    """

    def __init__(self, action):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.action = action

    def execute(self, userdata):
        gripper_command.set_named_target(self.action)
        gripper_command.go()
        
        return 'succeeded'
       

class grasp_object(smach.State):

    """
    Should be called after visual servoing has aligned the gripper with the object.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.result = None
        self.event_out = rospy.Publisher('/arm_relative_motion_controller/event_in', std_msgs.msg.String)
        rospy.Subscriber('/arm_relative_motion_controller/event_out', std_msgs.msg.String, self.event_cb)
        

    def execute(self, userdata):
        self.result = None

        gripper_command.set_named_target('open')
        gripper_command.go()
        
        # start the relative approach and wait for the result
        self.event_out.publish('e_start')
        while not self.result:
            rospy.sleep(0.1)

        if self.result.data != 'e_success':
            return 'failed'

        gripper_command.set_named_target('close')
        gripper_command.go()

        return 'succeeded'

    def event_cb(self, msg):
        self.result = msg


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
        
        arm_command.set_named_target(pltf_obj_pose)
        arm_command.go()
            
        gripper_command.set_named_target("close")
        gripper_command.go()
        
        arm_command.set_named_target("platform_intermediate")
        arm_command.go()
            
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
        
        arm_command.set_named_target(cfg_goal_pose)
        arm_command.go()
        
        gripper_command.set_named_target("open")
        gripper_command.go()
        
        arm_command.set_named_target("platform_intermediate")
        arm_command.go()
                
        return 'succeeded'

class compute_pregrasp_pose(smach.State):

    """
    Given an object pose compute a pregrasp position that is reachable and also
    good for the visual servoing.
    """

    FRAME_ID = '/base_link'

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'tf_transform_failed'], input_keys=['object_pose'],output_keys=['move_arm_to'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        pose = userdata.object_pose.pose

        try:
            t = self.tf_listener.getLatestCommonTime(self.FRAME_ID,
                                             pose.header.frame_id)
            pose.header.stamp = t
            pose = self.tf_listener.transformPose(self.FRAME_ID, pose)

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr('Tf error: %s' % str(e))
            return 'tf_transform_failed'

        p = pose.pose.position
        o = pose.pose.orientation
        frame_id = pose.header.frame_id

        userdata.move_arm_to = [p.x - 0.10, p.y, p.z + 0.20, 0, (0.8 * math.pi), 0, frame_id]

        return 'succeeded'
