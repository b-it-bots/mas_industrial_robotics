#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import commands
import os

from arm_configuration import *

tf_listener = 0

class init_robot(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'])
        
        global tf_listener
        self.move_arm = ArmConfiguration(tf_listener)

    def execute(self, userdata):
        # init arm
        self.move_arm.moveToConfiguration("initposition")
        
        #init gripper
        self.move_arm.moveGripperOpen()
        
        rospy.loginfo("robot initialized")
        
        return 'succeeded'
    
    
class announce_failure(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded'])

    def execute(self, userdata):
        
        wav_path = commands.getoutput("rospack find raw_generic_states")
        
        filename = wav_path + "/files/beep-3.wav"
        print 'file: ', filename
        
        while True:
            os.system("aplay -q " + filename)
            rospy.sleep(1)
        
        return 'succeeded'