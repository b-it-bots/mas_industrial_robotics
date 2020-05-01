#!/usr/bin/env python

import os
import pickle
import numpy as np
import tensorflow as tf

import pcl_object_recognition.models.dgcnn as dgcnn
import pcl_object_recognition.utils.pc_utils as pc_utils

class CNNBasedClassifiers():
    """
    CNN based classifier

    """

    def __init__(self, checkpoint_path, model):
        #self.CHECKPOINT_FILE = os.path.join(checkpoint_path, 'model.ckpt')
        log_dir = "/home/emha/ros_catkin_ws/src/b-it-bots/unmerged_packages_for_testing/perception/mir_object_recognition/common/config/DGCNN_asus"
        self.CHECKPOINT_FILE = os.path.join(log_dir, 'model.ckpt')
        with tf.Graph().as_default():
            K = 6
            self.points_pl = tf.placeholder(tf.float32, [1, 2048, K])
            self.is_training_pl = tf.placeholder(tf.bool)
            
            _, end_points = dgcnn.get_model(self.points_pl, num_classes=15, is_training=self.is_training_pl, bn_decay=None, K=K)
            
            #predictions that is not one_hot_encoded.
            self.probabilities = end_points['Probabilities']
            self.predictions = tf.argmax(self.probabilities, 1)
                                
            saver = tf.train.Saver()
            
            config = tf.ConfigProto()
            config.allow_soft_placement = True
            self.sess = tf.Session(config=config)
            saver.restore(self.sess, self.CHECKPOINT_FILE)
            

    def infer_one_cloud(self, points, pad_pc=True, num_points=2048, normalize=True): 
        if normalize:
            points = pc_utils.normalize_pointcloud(points)
        
        if pad_pc:
            points = points.tolist()
            while (len(points) < num_points):
                points.append([0,0,0,0,0,0])

        points = np.asarray(points)
        points = np.expand_dims(points, 0)

        feed_dict = {self.points_pl: points,
                     self.is_training_pl: False}
        
        pred_val, probs = self.sess.run([self.predictions, self.probabilities],
                                         feed_dict=feed_dict)
        probs = probs[0][pred_val][0]
        return pred_val, probs 