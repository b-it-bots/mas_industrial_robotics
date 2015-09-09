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


class move_arm(smach.State):

    """
    Move arm to a target. target may be fixed at construction time or set
    through userdata.

    Input
    -----
    move_arm_to: str | tuple | list
        target where the arm should move. If it is a string, then it gives
        target name (should be availabile on the parameter server). If it as
        tuple or a list, then it is treated differently based on the length. If it
        has 7 items, then it is cartesian pose (x, y, z, r, p ,y) + the corresponding frame.
        If it has 5 items, then it is arm configuration in join space.
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
            elif len(target) == 5:      # ... of 5 items: Joint space configuration
                try:
                    arm_command.set_joint_value_target(target)               
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
       

class linear_motion(smach.State):

    """
    Should be called after visual servoing has aligned the gripper with the object.
    Should probably be renamed in the future, or seperated into linear motion and grasping/releasing. 
    """

    def __init__(self, operation='grasp'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.operation = operation
        self.result = None
        self.event_out = rospy.Publisher('/arm_relative_motion_controller/event_in', std_msgs.msg.String)
        rospy.Subscriber('/arm_relative_motion_controller/event_out', std_msgs.msg.String, self.event_cb)
        

    def execute(self, userdata):
        self.result = None

        if self.operation == 'grasp': 
            gripper_command.set_named_target('open')
            gripper_command.go()
        elif self.operation == 'release':
            pass # Don't do anything, assume the gripper is already closed

        # start the relative approach and wait for the result
        self.event_out.publish('e_start')
        while not self.result:
            rospy.sleep(0.1)

        if self.result.data != 'e_success':
            return 'failed'

        if self.operation == 'grasp':
            gripper_command.set_named_target('close')
            gripper_command.go()
        elif self.operation == 'release':
            gripper_command.set_named_target('open')
            gripper_command.go()

        return 'succeeded'

    def event_cb(self, msg):
        self.result = msg
   
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


class update_static_elements_in_planning_scene(smach.State):

    """
    trigger component to add walls, objects, etc. to the the planning scene of moveit
    """

    def __init__(self, element, action):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.walls_event_out = rospy.Publisher('/mir_manipulation/arm_workspace_restricter/event_in', std_msgs.msg.String)

        self.action = action
        self.element = element

    def execute(self, userdata):

        # check which action to perform
        if(self.action == 'add'):
            event_command = 'e_start'
        elif(self.action == 'remove'):
            event_command = 'e_stop'

        #check which element to add/remove
        if(self.element == 'walls'):
            self.walls_event_out.publish(event_command)

        return 'succeeded'


class update_robot_planning_scene(smach.State):

    """
    trigger component to attach, detach, reattach and delete an object to/from the robot's planning scene
    """

    def __init__(self, action):
        smach.State.__init__(self, outcomes=['succeeded'],
                                   input_keys=['object'])

        self.pub_event = rospy.Publisher('/mir_manipulation/grasped_object_attacher/event_in', std_msgs.msg.String)
        self.pub_object_id = rospy.Publisher('/mir_manipulation/grasped_object_attacher/object_id', std_msgs.msg.Int32)

        self.action = action

    def execute(self, userdata):

        self.pub_object_id.publish(userdata.object.database_id)
        self.pub_event.publish("e_" + self.action)
        
        return 'succeeded'

class select_arm_pose(smach.State):

    """
    TODO
    """

    def __init__(self, pose_name_list=None):
        smach.State.__init__(self,
                             outcomes=['succeeded','failed'],
                 input_keys=['next_arm_pose_index'],
                             output_keys=['move_arm_to','next_arm_pose_index'])
        self.pose_name_list = pose_name_list
    
    def execute(self, userdata):
    if type(userdata.next_arm_pose_index) is not int:
         userdata.next_arm_pose_index = 0

    if len(self.pose_name_list) <= 0:
        rospy.logerr("pose name list is empty")
        return 'failed'

    if userdata.next_arm_pose_index >= len(self.pose_name_list):
        rospy.logerr("pose index out of range")
        return 'failed'

    userdata.move_arm_to = self.pose_name_list[userdata.next_arm_pose_index]
    userdata.next_arm_pose_index += 1
    userdata.next_arm_pose_index %= len(self.pose_name_list)

    return 'succeeded'  

    
