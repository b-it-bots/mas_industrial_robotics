#!/usr/bin/env python3

import colorsys
import os
import pickle
import struct
import sys
import time

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import yaml
from cv_bridge import CvBridge, CvBridgeError
from mas_perception_msgs.msg import ImageList, Object, ObjectList
from mas_perception_msgs.srv import RecognizeImage
# from rgb_object_recognition import SqueezeDetClassifier, SSDLiteMobilenet, util
from .rgb_object_recognition.yolov5 import yolov5_classifier
from sensor_msgs.msg import Image, RegionOfInterest

class RGBObjectRecognizer(Node):
    def __init__(self, debug_mode=True):
        super().__init__('rgb_object_recognizer')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('~model_dir', '/home/mas/catkin_ws/src/mas_perception_msgs/models/'),
                ('~net', 'detection'),
                ('~classifier', 'yolov5'),
                # ('~dataset')
            ])

        self.model_dir = self.get_parameter('~model_dir').value
        self.net = self.get_parameter('~net').value
        self.model_name = self.get_parameter('~classifier').value
        self.debug = debug_mode

        self.cvbridge = CvBridge()
        self.pub_debug = self.create_publisher(
            Image, "/mir_perception/multimodal_object_recognition/recognizer/rgb/output/debug_image", 1)
        self.pub_result = self.create_publisher(
            ObjectList, "output/object_list", 1)
        self.sub_img = self.create_subscription(
            ImageList, "input/images", self.image_recognition_cb, 10)
        self.confidence_threshold = 0.6

        config_file = os.path.join(get_package_share_directory("mir_object_recognition"),
                                   'ros', 'config', "rgb_classifier_config.yaml")

        self.yolo_data_config_file = os.path.join(get_package_share_directory("mir_recognizer_scripts"),
                                        'mir_recognizer_scripts','rgb_object_recognition', "data_for_detect.yaml")

        if os.path.isfile(config_file):
            configs = {}
            with open(config_file, 'r') as infile:
                configs = yaml.safe_load(infile)

            # model_config = configs['model'][model_name]
            self.classes = configs['classes']
            self.colors = configs['colors']

            if self.net == 'detection':
                # if self.model_name == 'squeezeDet':
                #     self.model = SqueezeDetClassifier(config=model_config,
                #                                       checkpoint_path=model_dir)

                # elif self.model_name == 'ssdLiteMobilenet':
                #     self.model = SSDLiteMobilenet(model_dir)
                if self.model_name == 'dummy':
                    pass

                elif self.model_name == 'yolov5':
                    print("TODO: Link YOLOv5 to original gitrepo")
                    print("TODO: Classify YOLOv5 inference code")

            elif self.net == 'classification':
                print("TODO: MobileNet")
        else:
            self.get_logger().error("Config file: {} not found".format(config_file))

    def image_recognition_cb(self, img_msg):
        if img_msg.images:
            result_list = ObjectList()
            objects = []
            self.get_logger().info("[{}] images received: {} ".format(len(img_msg.images), self.model_name))
            if self.net == 'detection':
                try:
                    cv_image = self.cvbridge.imgmsg_to_cv2(
                        img_msg.images[0], "bgr8")

                    if self.model_name == 'yolov5':

                        predictions = yolov5_classifier.run(weights=self.model_dir,
                                                            data=self.yolo_data_config_file,
                                                            source=cv_image)
                        # prediction bounding boxes are in [x1, y1, x2, y2] format

                        self.confidence_threshold = 0.6

                        output_bb_ary = predictions['boxes']
                        output_labels_ary = predictions['labels']
                        output_scores_ary = predictions['scores']

                        detected_object_list = []
                        detected_object_score = []
                        detected_bb_list = []

                        # Extract required objects from prediction output with confidence score greater than 0.5
                        print("---------------------------")
                        print("Name of the objects, Score\n")
                        for idx, value in enumerate(output_labels_ary):
                            object_name = value
                            score = output_scores_ary[idx]

                            if score >= self.confidence_threshold:
                                detected_object_list.append(object_name)
                                detected_object_score.append(score)
                                detected_bb_list.append(output_bb_ary[idx])

                                print("{}, {}".format(object_name, score))

                        print("---------------------------")

                        bboxes, probs, labels = detected_bb_list, detected_object_score, detected_object_list

                    else:
                        bboxes, probs, labels = self.model.classify(cv_image)

                    for i in range(len(labels)):
                        result = Object()
                        result.name = labels[i].upper()
                        result.probability = probs[i]
                        roi = RegionOfInterest()
                        roi.x_offset = int(bboxes[i][0])
                        roi.y_offset = int(bboxes[i][1])
                        roi.width = int(bboxes[i][2]) - int(bboxes[i][0])
                        roi.height = int(bboxes[i][3]) - int(bboxes[i][1])
                        result.roi = roi

                        objects.append(result)

                    # Publish result_list
                    result_list.objects = objects
                    self.pub_result.publish(result_list)

                    if self.debug:

                        # Draw bounding box on image
                        debug_img = cv_image
                        debug_img = debug_img.astype(np.float32, copy=False)
                        debug_img = cv2.resize(debug_img, (img_msg.images[0].width,
                                                           img_msg.images[0].height))
                        # Get display labels
                        classes = [label.upper() for label in labels]

                        # draw bounding boxes on image, probability and class
                        for i in range(len(bboxes)):
                            x1 = int(bboxes[i][0])
                            y1 = int(bboxes[i][1])
                            x2 = int(bboxes[i][2])
                            y2 = int(bboxes[i][3])
                            cv2.rectangle(debug_img, (x1, y1), (x2, y2),
                                          (0, 0, 255), 2)
                            cv2.putText(debug_img, "{} {:.2f}".format(classes[i], probs[i]),
                                        (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                        # publish bbox and label
                        self.publish_debug_img(debug_img)

                except CvBridgeError as e:
                    print(e)
                    return

            elif self.net == 'classification':
                print("TODO: MobileNet")

    def publish_debug_img(self, debug_img):
        debug_img = np.array(debug_img, dtype=np.uint8)
        debug_img = self.cvbridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.pub_debug.publish(debug_img)


def main():
    rclpy.init()

    # net = rclpy.get_param("~net")
    # classifier_name = rclpy.get_param("~classifier")
    # dataset = rclpy.get_param("~dataset")
    # model_dir = rclpy.get_param("~model_dir")

    recognizer = RGBObjectRecognizer(debug_mode=True)
        # model_dir=model_dir, net=net, model_name=classifier_name, debug_mode=True)

    rclpy.spin(recognizer)

    # recognizer.destroy_node()

    # rclpy.shutdown()




if __name__ == '__main__':
    main()
