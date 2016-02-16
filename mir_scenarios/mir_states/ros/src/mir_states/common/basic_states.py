#!/usr/bin/python

##import hbrs_srvs.srv
import rospy
import smach
import smach_ros
import commands
import os
import std_msgs.msg
import copy

import mir_states.common.manipulation_states as manipulation

class init_robot(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):

        #init gripper
        manipulation.gripper_command.set_named_target("open")
        manipulation.gripper_command.go()

        # init arm
        manipulation.arm_command.set_named_target("folded")
        manipulation.arm_command.go()

        rospy.loginfo("robot initialized")

        return 'succeeded'

class loop_for_vs(smach.State):
    '''
    This state will return 'loop' MAX-1 times.
    On the MAX execute, 'continue' is returned.
    '''
    def __init__(self, MAX, sleep_time=0.0):
        smach.State.__init__(self, outcomes=['loop', 'continue' ],
                                   input_keys=['prev_vs_result'],
                                   output_keys=['prev_vs_result'])
        self.max_loop_count = MAX
        self.loop_count = 0
        self.sleep_time = sleep_time

    def execute(self, userdata):
        print("[loop_for_vs]: {0}".format(userdata.prev_vs_result))
        if userdata.prev_vs_result == 'success':
            self.loop_count = 0
        if self.loop_count < self.max_loop_count:
            rospy.sleep(self.sleep_time)
            rospy.loginfo('run number: %d' % self.loop_count)
            self.loop_count = self.loop_count + 1
            return 'loop'
        else:
            self.loop_count = 0
            return 'continue'

class set_vs_status(smach.State):
    def __init__(self, status):
        smach.State.__init__(self, outcomes=['success'],
                                   input_keys=['prev_vs_result'],
                                   output_keys=['prev_vs_result'])
        self.status = status

    def execute(self, userdata):
        print("status: {0}".format(self.status))
        if self.status:
            userdata.prev_vs_result = 'success'
        else:
            userdata.prev_vs_result = 'fail'
        print("[set_vs_status]: {0}".format(userdata.prev_vs_result))

        return 'success'

class loop_for(smach.State):
    '''
    This state will return 'loop' MAX-1 times.
    On the MAX execute, 'continue' is returned.
    '''
    def __init__(self, MAX, sleep_time=0.0):
        smach.State.__init__(self, outcomes=['loop', 'continue' ])
        self.max_loop_count = MAX
        self.loop_count = 0
        self.sleep_time = sleep_time

    def execute(self, foo):
        if self.loop_count < self.max_loop_count:
            rospy.sleep(self.sleep_time)
            rospy.loginfo('run number: %d' % self.loop_count)
            self.loop_count = self.loop_count + 1
            return 'loop'
        else:
            return 'continue'

class send_event(smach.State):
    '''
    This state will take a list of event as input. Which are pair of name and value to publish.
    Output of this node is to publish the value in the provided topic name.
    '''

    def __init__(self, event_list):
        smach.State.__init__(self, outcomes=['success'])
        self.event_publisher_list = []
        self.expected_return_values_ = []
        self.event_names_ = []
        self.possible_event_values = ['e_start','e_stop','e_trigger']
        for event in event_list:
            if len(event) != 2:
                rospy.logerr('The event list is malformed!!')
                exit()
            elif event[1].lower() not in self.possible_event_values:
                rospy.logerr('Improper event value!!')
                exit()

            event_name = event[0]
            self.event_names_.append(event_name)
            self.expected_return_values_.append(event[1].lower())
            self.event_publisher_list.append(rospy.Publisher(event_name, std_msgs.msg.String))

    def execute(self, userdata):
        for index in range(len(self.event_publisher_list)):
            self.event_publisher_list[index].publish(self.expected_return_values_[index])
            rospy.logdebug('Published the event_name: %s event_value: %s',self.event_names_[index],self.expected_return_values_[index])
        return 'success'

class wait_for_single_event(smach.State):
    '''
    This state will take a event name as input and waits for the event to
    be published.
    '''
    def __init__(self, single_event):
        smach.State.__init__(self, outcomes=['success', 'failure', 'no_response'])
        self.event_name_ = single_event[0]
        self.expected_message_ = single_event[1]
        self.event_behavior = single_event[2]

        rospy.Subscriber(self.event_name_, std_msgs.msg.String, self.event_cb)
        self.callback_msg_ = None
        self.latest_event = ""

    def event_cb(self,callback_msg):
        self.callback_msg_ = callback_msg

    def getResult(self):
        if self.callback_msg_ is None:
           return 'no_response'
        self.latest_event = self.callback_msg_.data

        if (self.callback_msg_.data == self.expected_message_):
            self.callback_msg_ = None
            return 'success'
        elif self.event_behavior:
            self.callback_msg_ = None
            return 'failure'

        return 'no_response'

    def get_event_name(self):
        return self.event_name_

    def get_event_behavior(self):
        return self.event_behavior

    def get_latest_event(self):
        return self.latest_event

class wait_for_events(smach.State):
    '''

    This state will take a list of event as input and then split them into positive events and negative events
    based on the desired action to take.
    Postive events return success when the corresponing component return succeeds whereas Negative events have
    define opposite behavior i.e. return failure when corresponding component returns success.

    Events specification:
       [(topic name, expected message, desired behavior),
        (topic name, expected message, desired behavior),
        .........
        .........
       ]

    topic name: string type
    expected message: string type
    desired behavior: True(positive events) and False(Negative events)

    State Behavior:
       returns success: 
               if all the positive events get expected values. or
       returns failure:
               if any of the positive event get failure. or
               if any of the negative event get success.       or
       retrurns timeout:
               if all of the positive events or any of the negative event do not respond
               with in the specified timeout for the state.
    '''
    def __init__(self, event_list , timeout_duration=20):
        smach.State.__init__(self, outcomes=['success', 'failure','timeout'])
        self.event_publisher_list = []
        self.events_ = []
        self.timeout = rospy.Duration.from_sec(timeout_duration)
        self.event_type = []
        if not self.init_state(event_list):
           rospy.logerr('[wait for events state] Initialization failed.')
           exit()

    def init_state(self, event_list):
        if len(event_list) == 0:
           rospy.logerr('[wait for events] There are no events specified to subscribe.')
           return False

        for event in event_list:
            if len(event) != 3:
                rospy.logerr('[wait for events] Each specified event must contain topic name,expected message and desired behavior.')
                return False
            else:
                self.events_.append(wait_for_single_event(event))
        return True

    def execute(self, userdata):

        start_time = rospy.Time.now()
        temp_events = copy.copy(self.events_)


        while(rospy.Time.now() - start_time < self.timeout):

            num_pos_events = 0
            for event in temp_events:
               if event.get_event_behavior():
                   num_pos_events += 1
            if num_pos_events == 0:
                return 'success'

            for event in  temp_events:
                result = event.getResult().lower()
                event_behavior = event.get_event_behavior()
                if result == "failure":
                    rospy.logerr("[wait_for_events] Received event {0}:{1}".format(event.get_event_name(), event.get_latest_event()))
                    return "failure"
                elif result == "success":
                    if not event_behavior:
                       rospy.logerr("[wait_for_events] Received event {0}:{1}".format(event.get_event_name(), event.get_latest_event()))
                       return "failure"
                    else:
                       rospy.loginfo("[wait_for_events] Received event {0}:{1}".format(event.get_event_name(), event.get_latest_event()))


                    temp_events.remove(event)

            rospy.sleep(0.01)

        return 'timeout'

class wait_for_open_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):

        door_client = rospy.ServiceProxy('/raw_door_status/door_status', hbrs_srvs.srv.ReturnBool)
        rospy.wait_for_service('/raw_door_status/door_status',20) # todo error handling

        # wait for open door
        door_open = False
        while not door_open:
            print "door open?:", door_open
            try:
                res = door_client()
                door_open = res.value
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                door_open = False

            rospy.sleep(0.3)

        # sleep here to give the referees a chance to open the door completly
        rospy.sleep(1)

        return 'succeeded'

class set_named_config(smach.State):
    def __init__(self, named_config):
        smach.State.__init__(self, outcomes=['success', 'failure', 'timeout'])
        self.named_config = named_config
        self.config_name_pub = rospy.Publisher("/mcr_common/dynamic_reconfigure_client/configuration_name", std_msgs.msg.String)
        self.event_in_pub = rospy.Publisher("/mcr_common/dynamic_reconfigure_client/event_in", std_msgs.msg.String)
        self.event_out_sub = rospy.Subscriber("/mcr_common/dynamic_reconfigure_client/event_out", std_msgs.msg.String, self.event_cb)
        self.event = None

    def event_cb(self, msg):
        self.event = msg.data

    def execute(self, userdata):
        self.event = None

        self.config_name_pub.publish(self.named_config)
        self.event_in_pub.publish("e_start")

        timeout = rospy.Duration.from_sec(1.0)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < timeout:
            if self.event:
                if self.event == "e_success":
                    return 'success'
                else:
                    return 'failure'
            rate.sleep()
        return 'timeout'
