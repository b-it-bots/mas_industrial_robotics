#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import actionlib
import move_base_msgs.msg
import geometry_msgs.msg
import tf

class approach_pose(smach.State):

    def __init__(self, pose = ""):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['base_pose_to_approach'])

        self.move_base = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        self.pose = pose;    

    def execute(self, userdata):
        
        if(self.pose == ""):
            self.pose2 = userdata.base_pose_to_approach
	else:
	    self.pose2 = self.pose 
        
        if type(self.pose2) is str:
            goal_pose = rospy.get_param('/script_server/base/' + self.pose2)
            rospy.loginfo("moving to location: %s at %s", self.pose2, goal_pose) 
            
        elif type(self.pose2) is list and len(self.pose2) == 3:
            rospy.loginfo("moving to pose: %s", self.pose2)
            goal_pose = self.pose2
        else: # this should never happen
            rospy.logerr("Invalid pose format")
            return 'failed'  
         
	print 'preparing goal'                              
        goal = move_base_msgs.msg.MoveBaseGoal()
	goal.target_pose = geometry_msgs.msg.PoseStamped()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_pose[0]
        goal.target_pose.pose.position.y = goal_pose[1]
	quat = tf.transformations.quaternion_from_euler(0.0, 0.0, goal_pose[2])
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

#      	print 'wait for move base action server'          
#       self.move_base.wait_for_server()

        
	print 'sending goal'          
        self.move_base.send_goal(goal)
                
        while True:                
            rospy.sleep(1)
            base_state = self.move_base.get_state()
            if (base_state == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
                return "succeeded"
            elif (base_state == actionlib.simple_action_client.GoalStatus.ACTIVE):
                continue
            else:
                print 'last state: ',base_state
                return "failed"
            
            
            
