#!/usr/bin/env python
from __future__ import print_function

PACKAGE = "mir_cavity_detector"
NODE = "cavity_classifier"

import os
import sys
import roslib
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

import pickle
import numpy as np
import time

from mas_perception_msgs.msg import ImageList, ObjectList, Object

from sensor_msgs.msg import Image, RegionOfInterest

import struct
import colorsys
from mir_cavity_detector.mobileNet_classification import mobileNet
from datetime import datetime


class CavityRecognizer():

    def __init__(self, model_dir, model_name, debug_mode=False):


        self.cvbridge = CvBridge()
        self.debug = debug_mode
        self.pub_result = rospy.Publisher("/mir_perception/cavity_finder/output/cavities_list", ObjectList, queue_size=1)
        self.sub_img = rospy.Subscriber("/mir_perception/cavity_finder/output/cropped_cavities", ImageList, self.image_recognition_cb)
        self.pub_debug = rospy.Publisher("/mir_perception/cavity_finder/output/classifier_debug_image", Image, queue_size=1)
        self.model_name = model_name
        self.model_path = model_dir
        self.model = mobileNet(model_dir + model_name)
        
        rospy.loginfo("RGB Recognizer is ready using [%s]", self.model_name)

    def image_recognition_cb(self, img_msg):

        if img_msg.images:
            result_list = ObjectList()
            cavities_labels = []
            rospy.loginfo("[{}] images received: {} ".format(len(img_msg.images), self.model_name))
            now = datetime.now()
            concat_images = np.zeros((96,96))
            font = cv2.FONT_HERSHEY_SIMPLEX
            count = 0

            
            for img in img_msg.images:
                
                print("count ", count)

                try:
                    result = Object()
                    time = now.strftime("%H:%M:%S")
                    cv_image = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
                    label = self.model.predict_one_label(cv_image)
                    cv2.imwrite("/tmp/mobileNet_"+str(label)+str(time)+".jpg", cv_image)
                    cv2.putText(cv_image, str(label), (0,20), font, 1, (200, 255, 155), 2, cv2.LINE_AA)
                    '''                    
                    if count == 0:
                        cv2.resize(cv_image, (96,96))
                        concat_images = cv_image
                    else:
                        cv2.resize(cv_image, (96,96))
                        concat_images = np.hstack((concat_images, cv_image))
                    '''
                    #self.publish_debug_image(cv_image)
                    
                    count += 1 
                    print ("predicted label ", label)
                    # cavities_labels.append(label)
                    result.name = label
                    cavities_labels.append(result)
                # Publish result_list
                    #print ("cavities_labels ", cavities_labels)

                except CvBridgeError as e:
                    print ("inside except cv bridge")
                    print(e)
                    return


            cv2.imwrite("/tmp/classifier_image.jpg", concat_images)
            #self.publish_debug_img(concat_images)
            #self.pub_classifier_debug_image.publish(concat_images)
            result_list.objects= cavities_labels
            self.pub_result.publish(result_list)

    def publish_debug_img(self, debug_img):
        #debug_img = np.array(debug_img, dtype=np.uint8)
        debug_img = self.cvbridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.pub_debug.publish(debug_img)

if __name__ == '__main__':
    rospy.init_node(NODE)
    rospy.loginfo('Started Cavity Classification Node')

    path = "trained_model/"
    dataset_dir = os.path.join(roslib.packages.get_pkg_dir(PACKAGE), path)

    while not rospy.is_shutdown():
        object_recognizer = CavityRecognizer(model_dir=dataset_dir, model_name="output_graph.pb", debug_mode=True)
        rospy.sleep(0.025)
        rospy.spin()



































# #!/usr/bin/env python

# import rospy
# from std_msgs.msg import String

# def talker():

#   pub = rospy.Publisher('chatter', String, queue_size=10)
#   rospy.init_node('talker', anonymous=True)
#   rate = rospy.Rate(10) # 10hz
#   while not rospy.is_shutdown():
#      hello_str = "hello world %s" % rospy.get_time()
#      rospy.loginfo(hello_str)
#     pub.publish(hello_str)
#   rate.sleep()

# if __name__ == '__main__':
#   try:
#     talker()
#   except rospy.ROSInterruptException:
#     pass
