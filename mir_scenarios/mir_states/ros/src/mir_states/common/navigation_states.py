#!/usr/bin/python
import rospy
import smach
import smach_ros
import actionlib 
import std_srvs.srv
import tf
import move_base_msgs.msg
import actionlib_msgs.msg 
import random
import math

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from actionlib.simple_action_client import GoalStatus

from mir_navigation_msgs.msg import OrientToBaseAction, OrientToBaseActionGoal
from mcr_navigation_msgs.srv import MoveRelative


class adjust_to_workspace(smach.State):

    ADJUST_SERVER = '/mir_navigation/base_placement/adjust_to_workspace'

    def __init__(self, distance=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                                   input_keys=['desired_distance_to_workspace'])

        self.ac_base_adj = actionlib.SimpleActionClient(self.ADJUST_SERVER, OrientToBaseAction)

        self.distance = distance or userdata.desired_distance_to_workspace
        if not self.distance:
          self.distance = 0.20

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

    def __init__(self, offset=None):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'timeout'],
                             input_keys=['move_base_by'])
        self.offset = offset

        self.pub_relative_base_ctrl_command = rospy.Publisher('/mcr_navigation/relative_base_controller/command', Twist, latch=True)
        self.pub_relative_base_ctrl_event = rospy.Publisher('/mcr_navigation/relative_base_controller/event_in', String, latch=True)
        self.sub_relative_base_ctrl_event = rospy.Subscriber('/mcr_navigation/relative_base_controller/event_out', String, self.relative_base_controller_event_cb)

        self.relative_base_ctrl_event = ""

    def relative_base_controller_event_cb(self, event):
        self.relative_base_ctrl_event = event.data  

    def sample_with_boundary(self, lower, upper):
        if (lower == 0.0) and (upper == 0.0):
            return 0.0

        value = random.uniform(lower, upper)
        if -0.015 <= value <= 0.015:
            value = math.copysign(0.015, value)

        return value

    def execute(self, userdata):

        offset = self.offset or userdata.move_base_by
        if not offset: 
          offset = [0,0,0]

        relative_base_move = Twist()

        if(len(offset) == 3):
            relative_base_move.linear.x = offset[0]
            relative_base_move.linear.y = offset[1]
            relative_base_move.angular.z = offset[2]

        elif(len(offset) == 6):
            relative_base_move.linear.x = self.sample_with_boundary(offset[0], offset[1])
            relative_base_move.linear.y = self.sample_with_boundary(offset[2], offset[3])
            relative_base_move.angular.z = self.sample_with_boundary(offset[4], offset[5])

        self.relative_base_ctrl_event = ""

        # publish desired relative movement
        self.pub_relative_base_ctrl_command.publish(relative_base_move)

        # publish event to start the movement
        self.pub_relative_base_ctrl_event.publish(String("e_start"))

        timeout = rospy.Duration.from_sec(10.0)  #wait for the done event max. 10 seconds
        start_time = rospy.Time.now()
        while(True):

            if self.relative_base_ctrl_event == 'e_done':
                rospy.loginfo('relative pose reached')
                return 'succeeded'

            if (rospy.Time.now() - start_time) > timeout:
                rospy.logerr('timeout of %f seconds exceeded for relative base movement' % float(timeout.to_sec()))
                return 'timeout'

            rospy.sleep(0.01)
        
        return 'succeeded'

## copied from old states
## same as move_base?
class approach_pose(smach.State):

    def __init__(self, pose_name = ""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['base_pose_to_approach'])

        self.pose_name = pose_name;  

        self.move_base_action_name = '/move_base'
        self.move_base_action = actionlib.SimpleActionClient(self.move_base_action_name, move_base_msgs.msg.MoveBaseAction)

        self.clear_costmap_srv_name = '/move_base/clear_costmaps'
        self.clear_costmap_srv = rospy.ServiceProxy(self.clear_costmap_srv_name, std_srvs.srv.Empty)  

    def execute(self, userdata):
        # remove close obstacles from the costmap
        try:
            rospy.loginfo("wait for service: %s", self.clear_costmap_srv_name)
            rospy.wait_for_service(self.clear_costmap_srv_name, 30)

            self.clear_costmap_srv()
        except:
            rospy.logerr("could not execute service <<%s>>", self.clear_costmap_srv_name)
            return 'failed'

        # wait for action server
        rospy.loginfo("Wait for action: %s", self.move_base_action_name)
        if not self.move_base_action.wait_for_server(rospy.Duration(5)):
            rospy.logerr("%s action server not ready within timeout, aborting...", self.move_base_action_name)
            return 'failed'

        # check if argument has been passed or userdata should be used
        if(self.pose_name == ""):
            self.pose_name2 = userdata.base_pose_to_approach
        else:
            self.pose_name2 = self.pose_name 

    
        # get pose from parameter server
        if type(self.pose_name2) is str:
            parameter_name = "/script_server/base/" + self.pose_name2
            if not rospy.has_param(parameter_name):
                rospy.logerr("parameter <<%s>> does not exist on ROS Parameter Server, aborting...", parameter_name)
                return 'failed'

            self.target_pose = rospy.get_param(parameter_name)
        else:
            rospy.logerr("pose parameter <<%s>> is not a string, aborting...", self.pose_name2)
            return 'failed'


        # prepare action message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "/map"
        pose.pose.position.x = self.target_pose[0]
        pose.pose.position.y = self.target_pose[1]
        pose.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, self.target_pose[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose = pose     

        # call action
        rospy.loginfo("send navigation goal")
        self.move_base_action.send_goal(goal)

        rospy.loginfo("wait for %s action to reach %s", self.move_base_action_name, self.pose_name2)
        self.move_base_action.wait_for_result()

        # evaluate result
        result = self.move_base_action.get_state()

        if (result == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
            return "succeeded"
        else:
            return "failed"
