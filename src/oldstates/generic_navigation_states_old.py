#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import actionlib 
import raw_srvs.srv
import std_srvs.srv
import raw_base_placement.msg
import tf

from simple_script_server import *
sss = simple_script_server()


class place_base_in_front_of_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'srv_call_failed'], input_keys=['object_pose'])

        self.move_base_relative_srv_name = '/raw_relative_movements/move_base_relative'
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
            

class adjust_pose_wrt_workspace(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.ac_base_adj_name = '/raw_base_placement/adjust_to_workspace'
        self.ac_base_adj = actionlib.SimpleActionClient(self.ac_base_adj_name, raw_base_placement.msg.OrientToBaseAction)

    def execute(self, userdata):
        
            
        rospy.loginfo("Waiting for action server <<%s>> to start ...", self.ac_base_adj_name);
        self.ac_base_adj.wait_for_server()
        rospy.loginfo("action server <<%s>> is ready ...", self.ac_base_adj_name);
        action_goal = raw_base_placement.msg.OrientToBaseActionGoal()
            
        action_goal.goal.distance = 0.1;
        rospy.loginfo("send action");
        self.ac_base_adj.send_goal(action_goal.goal);
        
        rospy.loginfo("wait for base to adjust");
        finished_before_timeout = self.ac_base_adj.wait_for_result()
    
        if finished_before_timeout:
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
        
        self.move_base_relative_srv_name = '/raw_relative_movements/move_base_relative'
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

class move_base_rel(smach.State):

    def __init__(self, x_distance, y_distance):
        smach.State.__init__(self, outcomes=['succeeded'])
        
        self.x_distance = x_distance
        self.y_distance = y_distance
        self.move_base_relative_srv = rospy.ServiceProxy('/raw_relative_movements/move_base_relative', raw_srvs.srv.SetPoseStamped) 

    def execute(self, userdata):
        
        print "wait for service: /raw_relative_movements/move_base_relative"   
        rospy.wait_for_service('/raw_relative_movements/move_base_relative', 30)

        goalpose = geometry_msgs.msg.PoseStamped()
        goalpose.pose.position.x = self.x_distance
        goalpose.pose.position.y = self.y_distance
        goalpose.pose.position.z = 0.1 #speed
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        goalpose.pose.orientation.x = quat[0]
        goalpose.pose.orientation.y = quat[1]
        goalpose.pose.orientation.z = quat[2]
        goalpose.pose.orientation.w = quat[3]
        
        # call base placement service
        try:
            self.move_base_relative_srv(goalpose)
        except:
            print "could not execute <</raw_relative_movements/move_base_relative>> service"  
        
        return 'succeeded'
            

class switch_to_navigation_ctrl_in_orocos(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'srv_call_failed'])
        
        self.nav_ctrl_mode_srv_name = "/raw_arm_bridge_ros_orocos/enable_navigation_ctrl_mode";
        self.nav_ctrl_mode_srv = rospy.ServiceProxy(self.nav_ctrl_mode_srv_name, std_srvs.srv.Empty)

    def execute(self, userdata):
        try:
            rospy.wait_for_service(self.nav_ctrl_mode_srv_name, 15)
            self.nav_ctrl_mode_srv()            
        except Exception, e:
            rospy.logerr("could not execute service <<%s>>: %s", self.nav_ctrl_mode_srv_name, e)
            return 'srv_call_failed'

        return "succeeded"
      
