# Author: Mohammad Wasil

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import cv2
import time
import sys
import os
import glob

import numpy as np
import tensorflow as tf

from rgb_object_recognition.config import kitti_squeezeDet_config
from rgb_object_recognition.nets import SqueezeDet
BASE_DIR = os.path.dirname(os.path.abspath('__file__'))
sys.path.append(BASE_DIR)

class squeezeDet():
  
  def __init__(self, checkpoint, net='squeezeDet'): 
    self.CHECKPOINT = os.path.join(checkpoint, 'model.ckpt')
    with tf.Graph().as_default():
      # Load config
      self.mc = kitti_squeezeDet_config()
      self.mc.BATCH_SIZE = 1
      # model parameters will be restored (squeezeNet)
      self.mc.LOAD_PRETRAINED_MODEL = False
      self.model = SqueezeDet(self.mc)
      self.image_width = self.mc.IMAGE_WIDTH
      self.image_height = self.mc.IMAGE_HEIGHT
      
      saver = tf.train.Saver(self.model.model_params)
      config = tf.ConfigProto()
      config.allow_soft_placement = True
      self.sess = tf.Session(config=config)
      saver.restore(self.sess, self.CHECKPOINT)

      print ('\033[92m'+"SqueezeDet model is loaded")

  def infer_one_image(self, input_img):
    """Detect image."""
    im = input_img
    im = im.astype(np.float32, copy=False)
    im = cv2.resize(im, (self.mc.IMAGE_WIDTH, self.mc.IMAGE_HEIGHT))
    input_image = im - self.mc.BGR_MEANS

    # Detect
    det_boxes, det_probs, det_class = self.sess.run(
        [self.model.det_boxes, self.model.det_probs, self.model.det_class],
        feed_dict={self.model.image_input:[input_image]})

    # Filter
    final_boxes, final_probs, final_class = self.model.filter_prediction(
        det_boxes[0], det_probs[0], det_class[0])

    keep_idx    = [idx for idx in range(len(final_probs)) \
                        if final_probs[idx] > self.mc.PLOT_PROB_THRESH]

    final_boxes = [final_boxes[idx] for idx in keep_idx]
    final_probs = [final_probs[idx] for idx in keep_idx]
    final_class = [final_class[idx] for idx in keep_idx]
    final_label = [self.mc.CLASS_NAMES[idx] for idx in final_class]
    
    return final_boxes, final_probs, final_class, final_label

