#!/usr/bin/python

import rospy

import smach
import smach_ros
from geometry_msgs.msg import PoseStamped

class increment_base_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['incremented', 'reset'],
            input_keys=['base_increments'],
            output_keys=['base_increments']),

        self.MAX_BASE_POSITIONS = 2

    def execute(self, userdata):
        if userdata.base_increments >= self.MAX_BASE_POSITIONS:
            return 'reset'
        else:
            userdata.base_increments += 1
            return 'incremented'

class reset_base(smach.State):
    def __init__(self, step_size=0.25):
        smach.State.__init__(self,
            outcomes=['done'],
            input_keys=['base_increments', 'move_base_by'],
            output_keys=['base_increments', 'move_base_by']),
        self.step_size = step_size

    def execute(self, userdata):
        userdata.move_base_by = (0.0, (userdata.base_increments)*self.step_size, 0.0)
        print "move base by: ", userdata.move_base_by
        userdata.base_increments = 0
        return 'done'

class save_gripper_pose(smach.State):
    GRIPPER_POSE_TOPIC='/rockin/gripper_pose'

    def __init__(self, timeout=2.0):
        smach.State.__init__(self,
            outcomes=['success', 'failure'],
            output_keys=['end_effector_pose']),

        self.gripper_pose = None
        self.object_pose_sub = rospy.Subscriber(self.GRIPPER_POSE_TOPIC, PoseStamped, self.gripper_pose_cb)
        self.timeout = timeout

    def gripper_pose_cb(self,callback_msg):
        self.gripper_pose = callback_msg

    def execute(self, userdata):
        self.gripper_pose = None

        timeout_dur = rospy.Duration.from_sec(self.timeout)
        start_time = rospy.Time.now()

        while(True):
            if self.gripper_pose:
                userdata.end_effector_pose = self.gripper_pose
                return 'success'
            elif (rospy.Time.now() - start_time) > timeout_dur:
                rospy.logerr('Timeout of %f seconds exceeded waiting for gripper pose' % float(timeout.to_sec()))
                return 'failure'
            rospy.sleep(0.1)

        return 'success'

