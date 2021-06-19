import numpy as np
import tf
import rospy
# import time
import std_msgs
from geometry_msgs.msg import PointStamped, TwistStamped, PoseArray, PoseStamped, Point, Pose
from nav_msgs.msg import Path
from mir_trajectory_executor import mir_trajectory_velocity_generator
#import matplotlib.pyplot as plt

class mir_trajectory_executor_ros:

    def __init__(self):

        '''
        INITIALIZERS
        '''
        self.velocity_generator = mir_trajectory_veloctiy_generator(rospy.get_param('~max_cartesian_velocity'))
        self.tf_listener = tf.TransformListener()
        self.waypoints = PointStamped()
        self.event = None
        self.path = None
        self.start_end = None

        '''
        PARAMETERS
        '''
        self.max_velocity = rospy.get_param('~max_velocity')
        self.feedforward_gain = rospy.get_param('~feedforward_gain')
        self.feedback_gain = rospy.get_param('~feedback_gain')
        self.x_offset = 0.005
        self.number_of_sampling_points = 30
        self.goal_tolerance = 0.001
        self.cluster_tolerance = 0.002
        self.touch_down_tolerance = 0.001
        self.touch_down_distance = 0.021
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10)) # Node cycle rate (in hz)

        '''
        PUBLISHERS
        '''
        self.velocity_pub = rospy.Publisher('~cartesian_velocity_command', TwistStamped, queue_size=1)
        self.cluster_pub = rospy.Publisher('~line_tracing_path', Path, queue_size=1)
        self.event_pub = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)

        '''
        SUBSCRIBERS
        '''
        rospy.Subscriber('~cartetian_trajectory_waypoints', PoseArray, self.waypoint_sub_cb)
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)

    '''
    CALLBACK FUNCTIONS
    '''

    def event_in_cb(self, msg):
        """
        Starts a planned motion based on the specified arm position.
        """
        rospy.loginfo("event_in recieved")
        self.event = msg.data

    '''
    STATE MACHINE
    '''
    def start(self):
        """
        Starts the component.
        """
        rospy.loginfo("Ready to start now...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'IDLE'
        elif self.event == 'e_stop':
            self.reset_component_data()
            self.event_pub.publish('e_stopped')
            return 'INIT'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        if self.event == 'e_stop':
            self.reset_component_data()
            self.event_pub.publish('e_stopped')
            return 'INIT'
        elif self.path.any != None:
            return 'RUNNING'
        elif self.path == None and self.event == 'e_start':
            rospy.loginfo('Path not recieved')
            self.event_pub.publish('e_failure')
            self.event = None
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.reset_component_data()
            self.event_pub.publish('e_stopped')
            return 'INIT'
        else:
            self.execute()
            self.event_pub.publish('e_success')
            self.reset_component_data()
            return 'INIT'

    '''
    USER-DEFINED FUNCTIONS
    '''
    def reset_component_data(self):
        """
        Clears the data of the component.
        """
        self.event = None
        self.path = None
        self.points = None

    def extract_current_pose(self):
        while True and not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
                return trans,rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def trace_curve(self):

        rospy.loginfo("Line tracing started")
        self.trajectory_controller()
        self.path = None

    def trajectory_controller(target):

        trans,rot = self.extract_current_pose()
        current_pos = np.array([trans[0], trans[1], 0.0])

        distance = np.linalg.norm(target - current_pos)
        print "final pos is ", path[:,-1]

        while distance > self.goal_tolerance and self.event != 'e_stop':


            vel_x = self.feedback_gain * (target[0] - current_pos[0])
            vel_y = self.feedback_gain * (target[1] - current_pos[1])
            vel_z = 0.0 # vel_z should be zero

            message = TwistStamped()
            message.header.seq = count
            message.header.frame_id = "/base_link"
            message.twist.linear.x = vel_x
            message.twist.linear.y = vel_y
            message.twist.linear.z = vel_z
            self.velocity_pub.publish(message)

            trans,rot = self.get_current_pose()
            current_pos = np.array([trans[0], trans[1], 0.0])
            distance = np.linalg.norm(path[:,-1] - current_pos)
            previous_index = current_index
            count += 1


        message = TwistStamped()
        message.header.seq = count
        message.header.frame_id = "/base_link"
        message.twist.linear.x = 0.0
        message.twist.linear.y = 0.0
        message.twist.linear.z = 0.0
        self.velocity_pub.publish(message)


    def execute(self):


        self.trace_curve()
        rospy.loginfo("Curve traced")

def main():

    rospy.init_node('mir_trajectory_executor')
    obj = mir_trajectory_executor_ros()
    obj.start()
