import os
import pickle
import numpy as np
import tensorflow as tf

import pcl_object_recognition.models.dgcnn as dgcnn
import pcl_object_recognition.utils.pc_utils as pc_utils
from pcl_object_recognition.cnn_based_classifiers import CNNBasedClassifiers

class DGCNNClassifier(CNNBasedClassifiers):
    """
    Point cloud classifier using Dynamic graph CNN (DGCNN) 

    """

    def __init__(self, **kwargs):
        super(DGCNNClassifier, self).__init__(**kwargs)
        
        # extract argumen from key word arg
        checkpoint_path = kwargs.get("checkpoint_path", None)
        self.num_classes = kwargs.get("num_classes", None)
        self.num_points = kwargs.get("num_points", None)
        self.cloud_dim = kwargs.get("cloud_dim", None)

        with tf.Graph().as_default():
            pointcloud_pl = tf.placeholder(tf.float32, [1, self.num_points, self.cloud_dim])
            is_training_pl = tf.placeholder(tf.bool)
            
            _, end_points = dgcnn.get_model(pointcloud_pl, num_classes=self.num_classes, 
                                            is_training=is_training_pl)
            
            probabilities = end_points['Probabilities']
            predictions = tf.argmax(probabilities, 1)

            self.ops = {'pointcloud': pointcloud_pl,
                        'is_training': is_training_pl,
                        'probabilities': probabilities,
                        'predictions': predictions}
                                
            saver = tf.train.Saver()
            
            config = tf.ConfigProto()
            config.allow_soft_placement = True
            self.sess = tf.Session(config=config)
            saver.restore(self.sess, checkpoint_path)
    
    def classify(self, pointcloud, center=True, rotate=True, pad=True): 
        """
        Classify point cloud

        :param pointcloud:  The input pointcloud (BxNxD), D can be XYZ or XYZRGB
        :type name:         numpy.array
        :param center:      If true, pointcloud will be centered
        :type name:         Bool
        :param rotate:      If true, pointcloud will be rotated to their principal axes
        :type name:         Bool
        :param pad:         If true, pointcloud will be padded
        :type name:         Bool

        :return:  Predicted label and probablity
        """
        if center:
            pointcloud = pc_utils.center_pointcloud(pointcloud)
        
        if rotate:
            pointcloud = pc_utils.rotate_pointcloud(pointcloud)

        if pad:
            pointcloud = pointcloud.tolist()
            while (len(pointcloud) < self.num_points):
                pointcloud.append([0,0,0,0,0,0])

        pointcloud = pc_utils.scale_to_unit_sphere(np.asarray(pointcloud), normalize=False)
        pointcloud = np.expand_dims(pointcloud, 0)

        feed_dict = {self.ops['pointcloud']: pointcloud,
                     self.ops['is_training']: False}
        
        pred_label, probs = self.sess.run([self.ops['predictions'], self.ops['probabilities']],
                                         feed_dict=feed_dict)
        probs = probs[0][pred_label][0]

        return pred_label[0], probs 