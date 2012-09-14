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
            
'''
class adjust_pose_wrt_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        ac_base_adj = actionlib.SimpleActionClient('/scan_front_orientation', raw_base_placement_matthias_fueller.msg.OrientToBaseAction)
            
        rospy.loginfo("Waiting for action server <</scan_front_orientation>> to start ...");
        ac_base_adj.wait_for_server()
        rospy.loginfo("action server <</scan_front_orientation>> is ready ...");
        action_goal = raw_base_placement_matthias_fueller.msg.OrientToBaseActionGoal()
            
        action_goal.goal.distance = 0.02;
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
'''
            
class adjust_pose_wrt_platform(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        self.srv_adj_pose = rospy.ServiceProxy('/raw_adjust_pose_wrt_nearest_obj/adjust_pose', std_srvs.srv.Empty) 

    def execute(self, userdata):      
            
        print "wait for service: /raw_adjust_pose_wrt_nearest_obj/adjust_pose"   
        rospy.wait_for_service('/raw_adjust_pose_wrt_nearest_obj/adjust_pose', 20)
        
        # call base placement service
        try:
            self.srv_adj_pose()
        except:
            print "could not execute <</raw_adjust_pose_wrt_nearest_obj/adjust_pose>> service"
            return 'failed'
            
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
            
