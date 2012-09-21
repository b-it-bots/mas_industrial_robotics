#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import actionlib 
import raw_srvs.srv
import std_srvs.srv

from simple_script_server import *
sss = simple_script_server()


class place_base_in_front_of_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_pose'])

    def execute(self, userdata):
        
        # get optimal base pose with nirmals base placement component
        
        # move robot to the optimal pose with nirmals relative movements component
        
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
            

class adjust_pose_wrt_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.ac_base_adj_name = '/raw_base_placement/scan_front_orientation'
        self.ac_base_adj = actionlib.SimpleActionClient(self.ac_base_adj_name, raw_base_placement.msg.OrientToBaseAction)

    def execute(self, userdata):
        
            
        rospy.loginfo("Waiting for action server <<%s>> to start ...", self.ac_base_adj_name);
        self.ac_base_adj.wait_for_server()
        rospy.loginfo("action server <<%s>> is ready ...", self.ac_base_adj_name);
        self.action_goal = raw_base_placement.msg.OrientToBaseActionGoal()
            
        action_goal.goal.distance = 0.02;
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
        
        self.shiftbase_srv_name = '/raw_relative_movement/shiftbase'
        self.shiftbase_srv = rospy.ServiceProxy(self.shiftbase_srv_name, raw_srvs.srv.SetPoseStamped) 
    def execute(self, userdata):
        
        #rospy.loginfo("wait for service: /raw_base_placement/calculateOptimalBasePose")   
        #rospy.wait_for_service('/raw_base_placement/calculateOptimalBasePose', 30)

        print "OBJ POSE: ", userdata.object_to_grasp
        
        tf_listener = tf.TransformListener()

        tf_wait_worked = False
        while not tf_wait_worked:
            try:
                #print "frame_id:",userdata.object_to_grasp.header.frame_id
                tf_listener.waitForTransform(userdata.object_to_grasp.header.frame_id, '/base_link', rospy.Time.now(), rospy.Duration(2))
                tf_wait_worked = True
            except Exception, e:
                print "tf exception: wait for transform: ", e
                tf_wait_worked = False
                rospy.sleep(0.5)
                   
        transformed_poses = []
       
        tf_worked = False
        obj_pose_transformed = 0
        while not tf_worked:
            try:
                obj_pose_transformed = tf_listener.transformPose('/base_link', userdata.object_to_grasp)
                tf_worked = True
            except Exception, e:
                print "tf exception in: ", e
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
            rospy.loginfo("wait for service: %s", self.shiftbase_srv_name)   
            rospy.wait_for_service(self.shiftbase_srv_name, 30)
            
            self.shiftbase_srv(goalpose)  
        except:
            rospy.logerr("could not execute service <</raw_base_placement/calculateOptimalBasePose>>")
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
    
        '''
        x = (float(userdata.object_to_grasp.pose.position.x) - 0.45)
        y = float(userdata.object_to_grasp.pose.position.y) 
        yaw = float(0.0)
        '''
        
        return 'succeeded'

class move_base_rel(smach.State):

    def __init__(self, x_distance, y_distance):
        smach.State.__init__(self, outcomes=['succeeded'])
        
        self.x_distance = x_distance
        self.y_distance = y_distance
        self.shiftbase_srv = rospy.ServiceProxy('/raw_relative_movements/shiftbase', raw_srvs.srv.SetPoseStamped) 

    def execute(self, userdata):
        
        print "wait for service: /raw_relative_movements/shiftbase"   
        rospy.wait_for_service('/raw_relative_movements/shiftbase', 30)

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
            self.shiftbase_srv(goalpose)
        except:
            print "could not execute <</raw_relative_movements/shiftbase>> service"  
        
        return 'succeeded'
            
