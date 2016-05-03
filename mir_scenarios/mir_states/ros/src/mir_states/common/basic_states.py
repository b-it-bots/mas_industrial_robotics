#!/usr/bin/python

import rospy
import smach
import smach_ros
import std_msgs.msg

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
