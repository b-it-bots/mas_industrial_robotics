#!/usr/bin/python
import rospy
import smach
import smach_ros
import actionlib 
import std_srvs.srv
import tf

from geometry_msgs.msg import PoseStamped
from actionlib.simple_action_client import GoalStatus
from simple_script_server import *
sss = simple_script_server()

from mir_navigation_msgs.msg import OrientToBaseAction, OrientToBaseActionGoal
from mcr_navigation_msgs.srv import MoveRelative

class place_base_in_front_of_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'srv_call_failed'], input_keys=['object_pose'])

        self.move_base_relative_srv_name = '/mir_relative_movements/move_base_relative'
        self.move_base_relative_srv = rospy.ServiceProxy(self.move_base_relative_srv_name, raw_srvs.srv.SetPoseStamped) 

        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
                
        # transform to base_link
        
        tf_worked = False
        while not tf_worked:
            try:
            
                userdata.object_pose.header.stamp = rospy.Time.now()
                self.tf_listener.waitForTransform('/base_link', userdata.object_pose.header.frame_id, rospy.Time.now(), rospy.Duration(5))
                obj_pose_transformed = self.tf_listener.transformPose('/base_link', userdata.object_pose)
                tf_worked = True
            except Exception, e:
                rospy.logerr("tf exception in place_base_in_front_of_object: transform: %s", e)
                rospy.sleep(0.2)
                tf_worked = False
        
        
        print obj_pose_transformed
        
        # call base placement service
        try:
            rospy.loginfo("wait for service: <<%s>>", self.move_base_relative_srv_name)   
            rospy.wait_for_service(self.move_base_relative_srv_name, 15)


            goalpose = geometry_msgs.msg.PoseStamped()
            goalpose.pose.position.x = obj_pose_transformed.pose.position.x             # WILL ONLY WORK FOR IROS FINAL
            goalpose.pose.position.y = obj_pose_transformed.pose.position.y
            goalpose.pose.position.z = 0.05 # speed
            quat = tf.transformations.quaternion_from_euler(0,0,0)

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([obj_pose_transformed.pose.orientation.x, obj_pose_transformed.pose.orientation.y, obj_pose_transformed.pose.orientation.z, obj_pose_transformed.pose.orientation.w])   

            print "YAW of MARKER: ", yaw
            
            #goalpose.pose.orientation = obj_pose_transformed.pose.orientation
            goalpose.pose.orientation.x = quat[0]
            goalpose.pose.orientation.y = quat[1]
            goalpose.pose.orientation.z = quat[2]
            goalpose.pose.orientation.w = quat[3]
            
            print goalpose
            
            #raw_input("\npress ENTER to continue \n")
            
            self.move_base_relative_srv(goalpose)
        except Exception, e:
            rospy.logerr("service call <<%s>> failed: %s", self.move_base_relative_srv_name, e)  
            return 'srv_call_failed'
        
        return 'succeeded'


class move_base(smach.State):

    """
    Move the robot to the pose stored in the 'move_base_to' field of userdata
    or the pose passed to the constructor.

    Input
    -----
    move_base_to: str
        Name of the pose that the robot should approach. The name should exist
        on the parameter server. If a pose was supplied to the state
        constructor then it will override this input.

    Output
    ------
    base_pose: str
        Base position after the movement. If succeeded, then it will be set to
        the commanded pose. If failed, will be set to None to indicate that the
        pose is not known.
    """

    def __init__(self, pose=None, timeout=120):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['move_base_to'],
                             output_keys=['base_pose'])
        self.move_base_to = pose
        self.timeout = rospy.Duration(timeout)

    def execute(self, userdata):

        pose = self.move_base_to or userdata.move_base_to
        handle_base = sss.move('base', pose, blocking=False)
        started = rospy.Time.now()
        while True:
            rospy.sleep(0.1)
            base_state = handle_base.get_state()
            if rospy.Time.now() - started > self.timeout:
                return 'failed'
            if base_state == GoalStatus.SUCCEEDED:
                userdata.base_pose = pose
                return 'succeeded'
            elif base_state == GoalStatus.ACTIVE:
                continue
            else:
                print 'Last state: ', base_state
                userdata.base_pose = None
                return 'failed'

       
class adjust_to_workspace(smach.State):

    ADJUST_SERVER = '/mir_navigation/base_placement/adjust_to_workspace'

    def __init__(self, distance=0.20):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.ac_base_adj = actionlib.SimpleActionClient(self.ADJUST_SERVER, OrientToBaseAction)
        self.distance = distance

    def execute(self, userdata):
        rospy.logdebug("Waiting for action server <<%s>>...", self.ADJUST_SERVER)
        self.ac_base_adj.wait_for_server()
        rospy.logdebug("Action server <<%s>> is ready...", self.ADJUST_SERVER)
        action_goal = OrientToBaseActionGoal()
        action_goal.goal.distance = self.distance
        rospy.logdebug("Sending action goal...")
        self.ac_base_adj.send_goal(action_goal.goal)
        rospy.loginfo("Waiting for base to adjust...")
        if self.ac_base_adj.wait_for_result():
            rospy.loginfo("Action finished: %s", self.ac_base_adj.get_state())
            return 'succeeded'
        else:
            rospy.logerr("Action did not finish before the time out!")
            return 'failed'


class adjust_pose_wrt_recognized_obj(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                             input_keys=['object_to_grasp'])
        
        #self.base_placement_srv = rospy.ServiceProxy('/raw_base_placement/calculateOptimalBasePose', raw_srvs.srv.GetPoseStamped) 
        
        self.move_base_relative_srv_name = '/mir_navigation/mir_relative_movements/move_base_relative'
        self.move_base_relative_srv = rospy.ServiceProxy(self.move_base_relative_srv_name, raw_srvs.srv.SetPoseStamped) 
    def execute(self, userdata):
        
        #rospy.loginfo("wait for service: /raw_base_placement/calculateOptimalBasePose")   
        #rospy.wait_for_service('/raw_base_placement/calculateOptimalBasePose', 30)

        print "OBJ POSE: ", userdata.object_to_grasp
        
        try:
            tf_listener = tf.TransformListener(10)
        except Exception, e:
            print "tf exception in adjust_pose_wrt_recognized_obj: create transform listener: ", e

        tf_wait_worked = False
        while not tf_wait_worked:
            try:
                print "frame_id:",userdata.object_to_grasp.header.frame_id
                tf_listener.waitForTransform(userdata.object_to_grasp.header.frame_id, '/base_link', rospy.Time.now())
                tf_wait_worked = True
            except Exception, e:
                print "tf exception in adjust_pose_wrt_recognized_obj: wait for transform: ", e
                tf_wait_worked = False
                rospy.sleep(0.5)
                   
        transformed_poses = []
       
        tf_worked = False
        obj_pose_transformed = 0
        while not tf_worked:
            try:
                userdata.object_to_grasp.header.stamp = rospy.Time.now()
                obj_pose_transformed = tf_listener.transformPose('/base_link', userdata.object_to_grasp)
                tf_worked = True
            except Exception, e:
                print "tf exception in adjust_pose_wrt_recognized_obj: transform pose ", e
                tf_worked = False
        ##moveoptimalbase_srv = rospy.ServiceProxy('/raw_base_placement/moveoptimalbase', raw_srvs.srv.SetPoseStamped) 
        
        

        goalpose = geometry_msgs.msg.PoseStamped()
        goalpose.pose.position.x = 0.0
        goalpose.pose.position.y = obj_pose_transformed.pose.position.y
        goalpose.pose.position.z = 0.05
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        goalpose.pose.orientation.x = quat[0]
        goalpose.pose.orientation.y = quat[1]
        goalpose.pose.orientation.z = quat[2]
        goalpose.pose.orientation.w = quat[3]
        
        try:
            rospy.loginfo("wait for service: %s", self.move_base_relative_srv_name)   
            rospy.wait_for_service(self.move_base_relative_srv_name, 30)
            
            self.move_base_relative_srv(goalpose)  
        except:
            rospy.logerr("could not execute service <<%s>>", self.move_base_relative_srv_name)
            return 'failed'

        # call base placement service

        #try:
        #    userdata.object_base_pose = self.base_placement_srv(userdata.object_to_grasp)
        #except:
        #    rospy.logerr("could not execute service <</raw_base_placement/calculateOptimalBasePose>>")
        #    return 'failed'

        #print "BASE_POSE", userdata.object_base_pose

        #x = userdata.object_base_pose.base_pose.pose.position.x
        #y = userdata.object_base_pose.base_pose.pose.position.y
        #(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([userdata.object_base_pose.base_pose.pose.orientation.x, userdata.object_base_pose.base_pose.pose.orientation.y, userdata.object_base_pose.base_pose.pose.orientation.z, userdata.object_base_pose.base_pose.pose.orientation.w])        
    
        
        #x = (float(userdata.object_to_grasp.pose.position.x) - 0.45)
        #y = float(userdata.object_to_grasp.pose.position.y) 
        #yaw = float(0.0)
        print "succeeded"       
        return 'succeeded'


class move_base_relative(smach.State):

    """
    Shift the robot by the offset stored in 'move_base_by' field of userdata
    or the offset passed to the constructor.

    Input
    -----
    move_base_by: 3-tuple
        x, y, and theta displacement the shift the robot by. If an offset was
        supplied to the state constructor then it will override this input.
    """

    SRV = '/mcr_navigation/move_base_relative'

    def __init__(self, offset=None):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['move_base_by'])
        self.offset = offset
        self.move_base_relative = rospy.ServiceProxy(self.SRV,MoveRelative)

    def execute(self, userdata):
        rospy.logdebug('Waiting for service <<%s>>...' % (self.SRV))
        self.move_base_relative.wait_for_service()
        offset = self.offset or userdata.move_base_by
        pose = PoseStamped()
        pose.pose.position.x = offset[0]
        pose.pose.position.y = offset[1]
        pose.pose.position.z = 0.1  # speed
        quat = tf.transformations.quaternion_from_euler(0, 0, offset[2])
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        try:
            response = self.move_base_relative(pose)
            if response.status == 'failure_obtacle_front':
                # return values required by the scenario. This situation arises when the base is close to the platform
                return 'succeeded'
        except:
            rospy.logerr('Could no execute <<%s>>' % (self.SRV))
            return 'failed'
        return 'succeeded'

## copied from old states
## same as move_base?
class approach_pose(smach.State):

    def __init__(self, pose = ""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['base_pose_to_approach'])

        self.pose = pose;    

    def execute(self, userdata):
        
        if(self.pose == ""):
            self.pose2 = userdata.base_pose_to_approach
        else:
            self.pose2 = self.pose 
        
        handle_base = sss.move("base", self.pose2)

        while True:                
            rospy.sleep(0.1)
            base_state = handle_base.get_state()
            if (base_state == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
                return "succeeded"
            elif (base_state == actionlib.simple_action_client.GoalStatus.ACTIVE):
                continue
            else:
                print 'last state: ',base_state
                return "failed"