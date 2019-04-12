# Author: Mohammad Wasil (mwasil@outlook.co.id) 09/04/2019

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

from mir_object_recognition.config import *
from mir_object_recognition.nets import *
BASE_DIR = os.path.dirname(os.path.abspath('__file__'))
sys.path.append(BASE_DIR)  

CHECKPOINT = '/home/emha/ros_catkin_ws/src/b-it-bots/unmerged_packages_for_testing/perception/mir_object_recognition/common/src/mir_object_recognition/log/model.ckpt'
MODE = 'image'
INPUT_PATH = './data/sample.png'
OUT_DIR = './data/out/'
DEMO_NET = 'squeezeDet'

class ModelWrapper():
  
  def __init__(self, model_name="squeezeDet"): 
    # service
    self.model_name = model_name
    with tf.Graph().as_default():
    # Load model
      self.model_preload_config(self.model_name)
      saver = tf.train.Saver(self.model.model_params)
      config = tf.ConfigProto()
      config.allow_soft_placement = True
      self.sess = tf.Session(config=config)
      saver.restore(self.sess, CHECKPOINT)

  def model_preload_config(self):
      if self.model_name == "squeezeDet":
        self.mc = kitti_squeezeDet_config()
        self.mc.BATCH_SIZE = 1
        # model parameters will be restored from checkpoint
        self.mc.LOAD_PRETRAINED_MODEL = False
        self.model = SqueezeDet(self.mc)
        self.image_width = self.mc.IMAGE_WIDTH
        self.image_height = self.mc.IMAGE_HEIGHT
      elif self.model_name == "SSD-MobileNet":
        print "SSD MobileNet model"

  def infer_one_image(self, input_img):
    im = input_img
    im = im.astype(np.float32, copy=False)
    im = cv2.resize(im, (self.mc.IMAGE_WIDTH, self.mc.IMAGE_HEIGHT))
    input_image = im - self.mc.BGR_MEANS

    if self.model_name == "squeezeDet":
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
    
    elif self.model_name == "SSD-MobileNet":
        print ("SSD-MobileNet")

  # Draw bounding box on image
  def draw_box_on_img(self, img, final_boxes, final_probs, final_class, cls2clr):
      # Draw boxes
      self._draw_box(
          img, final_boxes,
          [self.mc.CLASS_NAMES[idx]+': (%.2f)'% prob \
              for idx, prob in zip(final_class, final_probs)],
          cdict=cls2clr,
      )

  def _draw_box(self, im, box_list, label_list, color=(0,255,0), cdict=None, form='center'):
    assert form == 'center' or form == 'diagonal', \
        'bounding box format not accepted: {}.'.format(form)
    print (label_list)
    for bbox, label in zip(box_list, label_list):

      if form == 'center':
        bbox = self.bbox_transform(bbox)

      xmin, ymin, xmax, ymax = [int(b) for b in bbox]

      l = label.split(':')[0] # text before "CLASS: (PROB)"
      if cdict and l in cdict:
        c = cdict[l]
      else:
        c = color

      # draw box
      cv2.rectangle(im, (xmin, ymin), (xmax, ymax), c, 1)
      # draw label
      font = cv2.FONT_HERSHEY_SIMPLEX
      cv2.putText(im, label, (xmin, ymax), font, 0.3, c, 1)

  def bbox_transform(self, bbox):
      """convert a bbox of form [cx, cy, w, h] to [xmin, ymin, xmax, ymax]. Works
      for numpy array or list of tensors.
      """
      cx, cy, w, h = bbox
      out_box = [[]]*4
      out_box[0] = cx-w/2
      out_box[1] = cy-h/2
      out_box[2] = cx+w/2
      out_box[3] = cy+h/2

      return out_box

  def bbox_transform_inv(self, bbox):
      """convert a bbox of form [xmin, ymin, xmax, ymax] to [cx, cy, w, h]. Works
      for numpy array or list of tensors.
      """
      xmin, ymin, xmax, ymax = bbox
      out_box = [[]]*4

      width       = xmax - xmin + 1.0
      height      = ymax - ymin + 1.0
      out_box[0]  = xmin + 0.5*width 
      out_box[1]  = ymin + 0.5*height
      out_box[2]  = width
      out_box[3]  = height

      return out_box
# def main(argv=None):
#   if not tf.gfile.Exists(FLAGS.out_dir):
#     tf.gfile.MakeDirs(FLAGS.out_dir)
#   if FLAGS.mode == 'image':
#     image_demo()
#   else:
#     video_demo()

#if __name__ == '__main__':
    #tf.app.run()
