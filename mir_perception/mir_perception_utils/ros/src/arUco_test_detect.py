#!/usr/bin/env python3
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class DetectArucoMarker():
    
    def __init__(self):

        #subscriber
        self.arUco_ids = None
        self.arUco_len = None
        self.bridge = CvBridge()
        print("in Init")
        rospy.Subscriber("/tower_cam3d_middle/color/image_raw", Image, self.astra_callbck)

    def astra_callbck(self, Image):

        Image = self.bridge.imgmsg_to_cv2(Image, "bgr8")
        # print('Image data', Image.data)

        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_1000)
        parameters = cv2.aruco.DetectorParameters_create()

        # print("detected")
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(Image, dictionary, parameters=parameters)
        Image = cv2.aruco.drawDetectedMarkers(Image, corners, ids)

        self.arUco_ids = ids

        if ids is not None: #add NoneType or greater than 1 condition for this check
            if len(ids)>0:
                self.arUco_len = len(ids)
                print(f'Detected arUco markers : ',self.arUco_len)
                print(f'Detected Ids of markers : ', self.arUco_ids)

    # def 

def main():
    rospy.init_node("detect_arUco_test", anonymous=True)
    print(cv2.__version__)
    detect_arUco_test = DetectArucoMarker()
    # detect_arUco_test.start()

if __name__ == "__main__":
    main()
    rospy.spin()