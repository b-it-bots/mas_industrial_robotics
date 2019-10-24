#!/usr/bin/env python
import rospy
import std_msgs.msg
import geometry_msgs.msg
import re


def main():
    print "Hello world"
    rospy.init_node('kb_upload_test')
    rospy.loginfo('Updating world model robot at destination')
    # publishers
    refbox = rospy.Publisher(
        '/refbox_parser/refbox',
        std_msgs.msg.String
    )

    event_in = rospy.Publisher(
        '/refbox_parser/event_in',
        std_msgs.msg.String
    )

    inventory_message = "BNT<(C1,W,3),(S1,E,3),(T3,N,3),(S3,S,3),(T1,S,3),(D1,E,3),(S4,N,3),(S5,N,3),(T4,W,3),(T2,S,3),(S2,E,3)>"

    HARDCODED_SPECS = {'BNT': 'BNT<(C1,W,3),(S1,E,3),(T3,N,3),(S3,S,3),(T1,S,3),(D1,E,3),(S4,N,3),(S5,N,3),(T4,W,3),(T2,S,3),(S2,E,3)>',
                       'BMT': 'BMT<S2,S2,S3,line(M20_100,F20_20_G,F20_20_B,S40_40_B,S40_40_G,R20,M30),EXIT>',
                       'BTT': 'BTT<initialsituation(<S5,(R20,M30,S40_40_B)><S2,(S40_40_G,M20,R20)><S3,(F20_20_B,M20_100,F20_20_G)>);goalsituation(<C1,line(M20_100,M30,M20)><S4,line(F20_20_G,R20,R20)><S1,line(S40_40_B,S40_40_G,F20_20_B)>)>',
                       #'BTT': 'BTT<initialsituation(<S3,(S40_40_B,F20_20_B,F20_20_B,M20)>);goalsituation(<S1,line(S40_40_B,F20_20_B,F20_20_B,M20)>)>',
                       'PPT': 'PPT<S3,(S40_40_B,M20,F20_20_B),S4>',
                       'CBT': 'CBT<C1>'}
    #task_spec =  re.findall('[^<>]+' , inventory_message)
    #print task_spec[1]

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        event_in.publish("e_trigger")
        print("send e_trigger")
        refbox.publish(HARDCODED_SPECS['PPT'])
        rate.sleep()

if __name__ == "__main__":
    main()
