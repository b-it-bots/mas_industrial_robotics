#!/usr/bin/env python

import rospy
from mcr_manipulation_msgs.msg import JointSpaceWayPointsList
import std_msgs.msg
import numpy as np

def talker():
    pub = rospy.Publisher('/way_point_list', JointSpaceWayPointsList, queue_size=10)
    rospy.init_node('simple_publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    i = 0
    msg = JointSpaceWayPointsList()

    ###Filling string array
    
    pos = np.array([ 'platform_middle_pre', 'candle', 'pre_grasp']) 
    
    str_array = []
    for idx, name in enumerate(pos):
        string_data = std_msgs.msg.String()
        string_data.data = name
        str_array.append(string_data)
    msg.list_of_joint_positions = str_array
   
    '''
    Filling joint value array
    
    single_val_arr2 = std_msgs.msg.Float64MultiArray()
    single_val_arr2.data = [2.21073308038, 1.78274546916, -1.6818567472, \
    3.40570528342, 2.93894928968]
    msg.list_of_joint_values_lists.append(single_val_arr2)

    single_val_arr = std_msgs.msg.Float64MultiArray()
    single_val_arr.data = [2.16324105989, 1.12969474829, -2.55175620279, \
    1.78127440488, 2.82916591907]
    msg.list_of_joint_values_lists.append(single_val_arr)
    
    print msg
    '''

    while not rospy.is_shutdown():
        if i ==0:
            pub.publish(msg)
            i = 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass