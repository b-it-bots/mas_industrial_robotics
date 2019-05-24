#!/usr/bin/env python

PACKAGE = "mir_object_recognition"
NODE = "rgb_object_recognizer"
SERVICE = "~rgb_object_recognizer"

# Color map for classes
cls2clr = {
    's40_40_G':(0, 0, 255), 
    's40_40_B':(0, 0, 255),
    'r20':(0, 0, 255),
    'motor':(0, 0, 255),
    'm30':(0, 0, 255), 
    'm20_100':(0, 0, 255), 
    'm20':(0, 0, 255), 
    'f20_20_G':(255, 191, 0), 
    'f20_20_B':(0, 0, 255),
    'em_02':(0, 0, 255), 
    'em_01':(0, 0, 255), 
    'distance_tube':(0, 0, 255), 
    'container_box_red':(0, 0, 255),
    'container_box_blue':(0, 0, 255), 
    'bearing_box_ax16':(0, 0, 255),
    'bearing_box_ax01':(0, 0, 255),
    'bearing':(0, 0, 255),
    'axis':(0, 0, 255)
}
class_label = {
    's40_40_G':'S40_40_G', 
    's40_40_B':'S40_40_B',
    'r20':'R20',
    'motor':'MOTOR',
    'm30':'M30', 
    'm20_100':'M20_100', 
    'm20':'M20', 
    'f20_20_G':'F20_20_G', 
    'f20_20_B':'F20_20_B',
    'em_02':'EM-02', 
    'em_01':'EM-01', 
    'distance_tube':'DISTANCE_TUBE', 
    'container_box_red':'RED_CONTAINER',
    'container_box_blue':'BLUE_CONTAINER', 
    'bearing_box_ax16':'BEARING_BOX',
    'bearing_box_ax01':'BEARING_BOX',
    'bearing':'BEARING',
    'axis':'AXIS'
}

import os
import sys
import roslib
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

import pickle
import numpy as np
import time

from mas_perception_msgs.srv import RecognizeImage
from mas_perception_msgs.srv import RecognizeImageResponse
from mas_perception_msgs.msg import ImageList, ObjectList, Object
from mas_perception_msgs.srv import GetSegmentedImage
from mas_perception_msgs.srv import GetSegmentedImageResponse
from sensor_msgs.msg import Image, RegionOfInterest

import struct
import colorsys

from rgb_object_recognition.squeezedet import squeezeDet
from rgb_object_recognition.ssdlite_mobilenet import SSDLiteMobilenet
from rgb_object_recognition.utils import util

class ObjectRecognizer():
    def __init__(self, model_dir, net='detection', model_name='squeezeDet', debug_mode=False):
        self.cvbridge = CvBridge()
        self.debug = debug_mode
        self.pub_debug = rospy.Publisher("/mir_perception/multimodal_object_recognition/recognizer/rgb/output/debug_image", Image, queue_size=1)
        self.pub_result = rospy.Publisher("output/object_list", ObjectList, queue_size=1)
        self.sub_img = rospy.Subscriber("input/images", ImageList, self.image_recognition_cb)
        self.net = net
        self.model_name = model_name
        
        if self.net == 'detection':
            if self.model_name == 'squeezeDet':
                self.model = squeezeDet(model_dir)
                print ("Using Squeezedet")
            elif self.model_name == 'SSDLiteMobilenet':
                self.model = SSDLiteMobilenet(model_dir)
                print ('Using SSDLiteMobilenet')
        elif self.net == 'classification':
           print "TODO: MobileNet"
        
        rospy.loginfo("RGB Recognizer is ready using [%s : %s]", self.net, self.model_name)

    def image_recognition_cb(self, img_msg):
        if img_msg.images:
            result_list = ObjectList()
            objects = []
            rospy.loginfo("[{}] images received: {} ".format(len(img_msg.images), self.model_name))
            if self.net == 'detection':
                try:
                    cv_image = self.cvbridge.imgmsg_to_cv2(img_msg.images[0], "bgr8")
                    bboxes, probs, classes, labels = self.model.infer_one_image(cv_image)
                    for i in range(len(classes)):
                        result = Object()
                        result.name = class_label[labels[i]]
                        result.probability = probs[i]
                        roi = RegionOfInterest()
                        bbox = util.bbox_transform(bboxes[i])
                        roi.x_offset = int(bbox[0])
                        roi.y_offset = int(bbox[1])
                        roi.width = int(bbox[2] - bbox[0])
                        roi.height = int(bbox[3] - bbox[1])
                        result.roi = roi
                        objects.append(result)

                    # Publish result_list
                    result_list.objects = objects
                    self.pub_result.publish(result_list)

                    if self.debug:
                        # Draw bounding box on image
                        debug_img = cv_image
                        debug_img = debug_img.astype(np.float32, copy=False)
                        debug_img = cv2.resize(debug_img, (self.model.image_width, self.model.image_height))
                        # Get display labels
                        class_labels = [class_label[label] for label in labels]
                        util.draw_box_on_img(debug_img, bboxes, probs, class_labels, cls2clr)
                        self.publish_debug_img(debug_img)

                except CvBridgeError as e:
                    print(e)
                    return

            elif self.net == 'classification':
                print "TODO: MobileNet"
                
    def publish_debug_img(self, debug_img):
        debug_img = np.array(debug_img, dtype=np.uint8)
        debug_img = self.cvbridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.pub_debug.publish(debug_img)


if __name__ == '__main__':
    rospy.init_node(NODE)
    rospy.loginfo('Started object recognition node.')
    #TODO
    #Get all param here, net, model_name, debug_mode
    net = rospy.get_param("~net")
    classifier_name = rospy.get_param("~classifier")
    dataset_type = rospy.get_param("~dataset_type")
    dataset_dir = os.path.join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'config', classifier_name, dataset_type)

    while not rospy.is_shutdown():
        object_recognizer = ObjectRecognizer(model_dir=dataset_dir, net=net, model_name=classifier_name, debug_mode=True)
        #rospy.sleep(0.025)
        rospy.spin()