#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import actionlib
import raw_base_placement_to_platform_in_front.msg
import raw_srvs.srv

from simple_script_server import *
sss = simple_script_server()

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

    def execute(self, userdata):
        ac_base_adj = actionlib.SimpleActionClient('/scan_front_orientation', raw_base_placement_to_platform_in_front.msg.OrientToBaseAction)
            
        rospy.loginfo("Waiting for action server <</scan_front_orientation>> to start ...");
        ac_base_adj.wait_for_server()
        rospy.loginfo("action server <</scan_front_orientation>> is ready ...");
        action_goal = raw_base_placement_to_platform_in_front.msg.OrientToBaseActionGoal()
            
        action_goal.goal.distance = 0.04;
        rospy.loginfo("send action");
        ac_base_adj.send_goal(action_goal.goal);
        
        rospy.loginfo("wait for base to adjust");
        finished_before_timeout = ac_base_adj.wait_for_result()
    
        if finished_before_timeout:
            rospy.loginfo("Action finished: %s", ac_base_adj.get_state())
            return 'succeeded'    
        else:
            rospy.logerr("Action did not finish before the time out!")
            return 'failed'


        
                
class adjust_pose_wrt_recognized_obj(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                             input_keys=['object_to_grasp','object_base_pose'], 
                             output_keys=['object_base_pose'])
        
        #self.base_placement_srv = rospy.ServiceProxy('/raw_base_placement/calculateOptimalBasePose', raw_srvs.srv.GetPoseStamped) 
        self.shiftbase_srv = rospy.ServiceProxy('/raw_motion_controller/shiftbase', raw_srvs.srv.SetPoseStamped) 
    def execute(self, userdata):
        
        #rospy.loginfo("wait for service: /raw_base_placement/calculateOptimalBasePose")   
        #rospy.wait_for_service('/raw_base_placement/calculateOptimalBasePose', 30)

    
        print "OBJ POSE: ", userdata.object_to_grasp
        

        
        ##moveoptimalbase_srv = rospy.ServiceProxy('/raw_base_placement/moveoptimalbase', raw_srvs.srv.SetPoseStamped) 

        print "wait for service: /raw_motion_controller/shiftbase"   
        rospy.wait_for_service('/raw_motion_controller/shiftbase', 30)

        goalpose = geometry_msgs.msg.PoseStamped()
        goalpose.pose.position.x = 0.0
        goalpose.pose.position.y = userdata.object_base_pose.base_pose.pose.position.y
        goalpose.pose.position.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        goalpose.pose.orientation.x = quat[0]
        goalpose.pose.orientation.y = quat[1]
        goalpose.pose.orientation.z = quat[2]
        goalpose.pose.orientation.w = quat[3]
        

        # call base placement service
        self.shiftbase_srv(goalpose)  


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

    def __init__(self, y_distance):
        smach.State.__init__(self, outcomes=['succeeded'])
        
        self.y_distance = y_distance
        self.shiftbase_srv = rospy.ServiceProxy('/raw_motion_controller/shiftbase', raw_srvs.srv.SetPoseStamped) 

    def execute(self, userdata):
        
        print "wait for service: /raw_motion_controller/shiftbase"   
        rospy.wait_for_service('/raw_motion_controller/shiftbase', 30)

        goalpose = geometry_msgs.msg.PoseStamped()
        goalpose.pose.position.x = 0.0
        goalpose.pose.position.y = y_distance
        goalpose.pose.position.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        goalpose.pose.orientation.x = quat[0]
        goalpose.pose.orientation.y = quat[1]
        goalpose.pose.orientation.z = quat[2]
        goalpose.pose.orientation.w = quat[3]
        
        # call base placement service
        self.shiftbase_srv(goalpose)  
        
        return 'succeeded'
