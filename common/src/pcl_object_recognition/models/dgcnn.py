import os
import sys
import math

import tensorflow as tf
import numpy as np

import pcl_object_recognition.utils.tf_util as tf_util
from transform_nets import input_transform_net

def get_model(pointcloud, num_classes, is_training, base=False):
    """ 
    Classification model DGCNN, input is BxNx3, output Bxnum_classes 

    :param pointcloud:  The input pointcloud (BxNxD), D can be XYZ or XYZRGB
    :type name:         Array
    :param num_classes: The number of classes
    :type name:         Int
    :param is_training: Training mode or not
    :type name:         Bool
    :param base:        If true, return DGCNN backbone
    :type name:         Bool

    :return:  Logits and endpoints
    """
    batch_size = pointcloud.get_shape()[0].value
    end_points = {}
    k = 20

    adj_matrix = tf_util.pairwise_distance(pointcloud)
    nn_idx = tf_util.knn(adj_matrix, k=k)
    edge_feature = tf_util.get_edge_feature(pointcloud, nn_idx=nn_idx, k=k)

    net = tf_util.conv2d(edge_feature, 64, [1,1],
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='dgcnn1', bn_decay=None)
    net = tf.reduce_max(net, axis=-2, keep_dims=True)
    net1 = net
    end_points['dgcnn1'] = net1

    adj_matrix = tf_util.pairwise_distance(net)
    nn_idx = tf_util.knn(adj_matrix, k=k)
    edge_feature = tf_util.get_edge_feature(net, nn_idx=nn_idx, k=k)

    net = tf_util.conv2d(edge_feature, 64, [1,1],
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='dgcnn2', bn_decay=None)
    net = tf.reduce_max(net, axis=-2, keep_dims=True)
    net2 = net
    end_points['dgcnn2'] = net2

    adj_matrix = tf_util.pairwise_distance(net)
    nn_idx = tf_util.knn(adj_matrix, k=k)
    edge_feature = tf_util.get_edge_feature(net, nn_idx=nn_idx, k=k) 

    net = tf_util.conv2d(edge_feature, 64, [1,1],
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='dgcnn3', bn_decay=None)
    net = tf.reduce_max(net, axis=-2, keep_dims=True)
    net3 = net
    end_points['dgcnn3'] = net3

    adj_matrix = tf_util.pairwise_distance(net)
    nn_idx = tf_util.knn(adj_matrix, k=k)
    edge_feature = tf_util.get_edge_feature(net, nn_idx=nn_idx, k=k)  

    net = tf_util.conv2d(edge_feature, 128, [1,1],
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='dgcnn4', bn_decay=None)
    net = tf.reduce_max(net, axis=-2, keep_dims=True)
    net4 = net
    end_points['dgcnn4'] = net4

    net = tf_util.conv2d(tf.concat([net1, net2, net3, net4], axis=-1), 1024, [1, 1], 
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='agg', bn_decay=None)

    net = tf.reduce_max(net, axis=1, keep_dims=True) 

    if base:
        end_points['DGCNN_PreFC'] = net
        return end_points

    # MLP on global point cloud vector
    net = tf.reshape(net, [batch_size, -1]) 
    net = tf_util.fully_connected(net, 512, bn=True, is_training=is_training,
                                     scope='fc1', bn_decay=None)
    net = tf_util.dropout(net, keep_prob=0.5, is_training=is_training, scope='dp1')
    
    net = tf_util.fully_connected(net, 256, bn=True, is_training=is_training,
                                     scope='fc2', bn_decay=None)
    net = tf_util.dropout(net, keep_prob=0.5, is_training=is_training,scope='dp2')

    net = tf_util.fully_connected(net, num_classes, activation_fn=None, scope='fc3')

    end_points['Logits'] = net
    end_points['Probabilities'] = tf.nn.softmax(net, name='Probabilities')

    return net, end_points

def get_model_with_tfnet(pointcloud, num_classes, is_training, base=False):
    """ 
    Classification model DGCNN with tfnet, input is BxNx3, output Bxnum_classes 
    :param pointcloud:  The input pointcloud
    :type name: Array
    :param num_classes:  The number of classes
    :type name: Int
    :param is_training:  True indicating training mode
    :type name: Bool
    :param base:  If true, return DGCNN backbone
    :type name: Bool

    :return:  Logits and endpoints
    """
    batch_size = pointcloud.get_shape()[0].value
    pc_dim = pointcloud.get_shape()[2].value
    end_points = {}
    k = 20

    adj_matrix = tf_util.pairwise_distance(pointcloud)
    nn_idx = tf_util.knn(adj_matrix, k=k)
    edge_feature = tf_util.get_edge_feature(pointcloud, nn_idx=nn_idx, k=k)

    with tf.variable_scope('transform_net1') as sc:
        transform = input_transform_net(edge_feature, is_training, K=pc_dim)
        end_points['transform'] = transform
    
    pointcloud_transformed = tf.matmul(pointcloud, transform)
    adj_matrix = tf_util.pairwise_distance(pointcloud_transformed)
    nn_idx = tf_util.knn(adj_matrix, k=k)
    edge_feature = tf_util.get_edge_feature(pointcloud_transformed, nn_idx=nn_idx, k=k)

    net = tf_util.conv2d(edge_feature, 64, [1,1],
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='dgcnn1', bn_decay=None)
    net = tf.reduce_max(net, axis=-2, keep_dims=True)
    net1 = net
    end_points['dgcnn1'] = net1

    adj_matrix = tf_util.pairwise_distance(net)
    nn_idx = tf_util.knn(adj_matrix, k=k)
    edge_feature = tf_util.get_edge_feature(net, nn_idx=nn_idx, k=k)

    net = tf_util.conv2d(edge_feature, 64, [1,1],
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='dgcnn2', bn_decay=None)
    net = tf.reduce_max(net, axis=-2, keep_dims=True)
    net2 = net
    end_points['dgcnn2'] = net2

    adj_matrix = tf_util.pairwise_distance(net)
    nn_idx = tf_util.knn(adj_matrix, k=k)
    edge_feature = tf_util.get_edge_feature(net, nn_idx=nn_idx, k=k) 

    net = tf_util.conv2d(edge_feature, 64, [1,1],
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='dgcnn3', bn_decay=None)
    net = tf.reduce_max(net, axis=-2, keep_dims=True)
    net3 = net
    end_points['dgcnn3'] = net3

    adj_matrix = tf_util.pairwise_distance(net)
    nn_idx = tf_util.knn(adj_matrix, k=k)
    edge_feature = tf_util.get_edge_feature(net, nn_idx=nn_idx, k=k)  

    net = tf_util.conv2d(edge_feature, 128, [1,1],
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='dgcnn4', bn_decay=None)
    net = tf.reduce_max(net, axis=-2, keep_dims=True)
    net4 = net
    end_points['dgcnn4'] = net4

    net = tf_util.conv2d(tf.concat([net1, net2, net3, net4], axis=-1), 1024, [1, 1], 
                        padding='VALID', stride=[1,1],
                        bn=True, is_training=is_training,
                        scope='agg', bn_decay=None)

    net = tf.reduce_max(net, axis=1, keep_dims=True) 

    if base:
        end_points['DGCNN_PreFC'] = net
        return end_points

    # MLP on global point cloud vector
    net = tf.reshape(net, [batch_size, -1]) 
    net = tf_util.fully_connected(net, 512, bn=True, is_training=is_training,
                                     scope='fc1', bn_decay=None)
    net = tf_util.dropout(net, keep_prob=0.5, is_training=is_training, scope='dp1')
    
    net = tf_util.fully_connected(net, 256, bn=True, is_training=is_training,
                                     scope='fc2', bn_decay=None)
    net = tf_util.dropout(net, keep_prob=0.5, is_training=is_training,scope='dp2')

    net = tf_util.fully_connected(net, num_classes, activation_fn=None, scope='fc3')

    end_points['Logits'] = net
    end_points['Probabilities'] = tf.nn.softmax(net, name='Probabilities')

    return net, end_points