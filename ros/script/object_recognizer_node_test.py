#!/usr/bin/env python

PACKAGE = "mir_object_recognition"
NODE = "object_reconizer_squeezdet_test"

import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

import pickle
import numpy as np
import time

from mcr_perception_msgs.srv import RecognizeImage
from mcr_perception_msgs.srv import RecognizeImageResponse
from mcr_perception_msgs.msg import ImageList
from mcr_perception_msgs.srv import GetSegmentedImage
from mcr_perception_msgs.srv import GetSegmentedImageResponse
from mcr_perception_msgs.srv import GetSegmentedImageRequest
from sensor_msgs.msg import Image

import struct
import colorsys

from rgb_object_recognition.squeezedet import squeezeDet
# def detect_obj(img):
#     print "Waiting"
    
#     try:
#         squeezedet = rospy.ServiceProxy('/object_reconizer_squeezdet/object_recognizer_squeezdet', GetSegmentedImage)
#         resp1 = squeezedet(img)
#     except rospy.ServiceException, e:
#        print "Service call failed: %s"%e

if __name__ == '__main__':
    rospy.init_node(NODE)
    print (NODE, " started")
    #bridge = CvBridge()
    ##sub = rospy.Subscriber("/camera/color/image_raw", Image, queue_size=1)
    #squeezedet = rospy.ServiceProxy('/object_reconizer_squeezdet/object_recognizer_squeezdet', GetSegmentedImage)
    #CNNFV.infer_one_object(input_data, input_label)
    #req = GetSegmentedImage()
    pub_img = rospy.Publisher("/mcr_perception/squeezedet/rgb/input/images", ImageList, queue_size=1) 
    def img_callback(img):
        img_list = ImageList()
        img_list.images = [img]
        pub_img.publish(img_list)

    while not rospy.is_shutdown():
        
        subscriber = rospy.Subscriber("/arm_cam3d/rgb/image_raw", Image, img_callback,  queue_size = 1)
        rospy.sleep(3.0)
        rospy.spin()