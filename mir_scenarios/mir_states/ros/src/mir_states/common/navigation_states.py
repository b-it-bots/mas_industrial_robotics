#!/usr/bin/python
import rospy
import smach
import smach_ros
import actionlib 
import std_srvs.srv
import tf

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from actionlib.simple_action_client import GoalStatus

from mir_navigation_msgs.msg import OrientToBaseAction, OrientToBaseActionGoal
from mcr_navigation_msgs.srv import MoveRelative

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

        # start the motion
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

    def execute(self, userdata):

        offset = self.offset or userdata.move_base_by

        relative_base_move = Twist()
        relative_base_move.linear.x = offset[0]
        relative_base_move.linear.y = offset[1]
        relative_base_move.angular.z = offset[2]

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

    def __init__(self, pose = ""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['base_pose_to_approach'])

        self.pose = pose;  
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
