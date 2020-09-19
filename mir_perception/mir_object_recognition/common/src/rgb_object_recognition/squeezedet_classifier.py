from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import cv2
import time
import sys
import os
import glob
import yaml

import numpy as np
import tensorflow as tf

from rgb_object_recognition.config import kitti_squeezeDet_config
from rgb_object_recognition.nets import SqueezeDet

from rgb_object_recognition.image_classifier import ImageClassifier 

class SqueezeDetClassifier(ImageClassifier):
    """
    SqueezeDet object detection and classification
    """

    def __init__(self, **kwargs):
        super(SqueezeDetClassifier, self).__init__(**kwargs)

        self.squeezedet_config = kitti_squeezeDet_config()
        self.squeezedet_config.BATCH_SIZE = 1
        self.squeezedet_config.LOAD_PRETRAINED_MODEL = False
        
        config = kwargs.get("config", None)
        if config is not None:
            self.squeezedet_config.IMAGE_WIDTH = int(config['image']['width'])
            self.squeezedet_config.IMAGE_HEIGHT = int(config['image']['height'])
            
        checkpoint_path = kwargs.get("checkpoint_path", None)
        
        if checkpoint_path is not None:
            checkpoint_path = os.path.join(checkpoint_path, 'model.ckpt')
            self.model = SqueezeDet(self.squeezedet_config)
            saver = tf.train.Saver(self.model.model_params)
            tf_config = tf.ConfigProto()
            tf_config.allow_soft_placement = True
            self.sess = tf.Session(config=tf_config)
            saver.restore(self.sess, checkpoint_path)

            print ('\033[92m'+"SqueezeDet model is loaded")

    def preprocess_image(self, image):
        """
        Preprocess image: resize, subtract with bgr means.

        :param image:  The input rgb image to be preprocessed
        :type name:         numpy.array

        :return:  preprocessed image

        """
        image = image.astype(np.float32, copy=False)
        image = cv2.resize(image, (self.squeezedet_config.IMAGE_WIDTH, 
                                   self.squeezedet_config.IMAGE_HEIGHT))
        image = image - self.squeezedet_config.BGR_MEANS

        return image
    
    def classify(self, image):
        """
        Detect and classify image

        :param image:  The input rgb image
        :type name:         numpy.array

        :return:  bounding boxes, probabilities and classes
        """

        image = self.preprocess_image(image)
        det_boxes, det_probs, det_class = self.sess.run(
                [self.model.det_boxes, self.model.det_probs, self.model.det_class],
                feed_dict={self.model.image_input:[image]}
            )

        boxes, probs, classes = self.model.filter_prediction(det_boxes[0], 
                                                             det_probs[0], 
                                                             det_class[0])

        keep_idx    = [idx for idx in range(len(probs)) \
                       if probs[idx] > self.squeezedet_config.PLOT_PROB_THRESH]

        keep_boxes = [boxes[idx] for idx in keep_idx]
        keep_probs = [probs[idx] for idx in keep_idx]
        keep_classes = [classes[idx] for idx in keep_idx]
        
        return keep_boxes, keep_probs, keep_classes
