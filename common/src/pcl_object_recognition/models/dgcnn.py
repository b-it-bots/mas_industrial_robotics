import os
import sys
import math

import tensorflow as tf
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, '../utils'))
sys.path.append(os.path.join(BASE_DIR, '../../utils'))

from pcl_object_recognition.utils import tf_util as dgcnn_util
from transform_nets import input_transform_net

def placeholder_inputs(batch_size, num_point):
  pointclouds_pl = tf.placeholder(tf.float32, shape=(batch_size, num_point, 3))
  labels_pl = tf.placeholder(tf.int32, shape=(batch_size))
  return pointclouds_pl, labels_pl


def get_model(point_cloud, num_classes, is_training, bn_decay=None, base=False, K=3):
  """ Classification PointNet, input is BxNx3, output Bx40 """
  batch_size = point_cloud.get_shape()[0].value
  num_point = point_cloud.get_shape()[1].value
  end_points = {}
  k = 20

  adj_matrix = dgcnn_util.pairwise_distance(point_cloud)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(point_cloud, nn_idx=nn_idx, k=k)
  
  print ("input: ", point_cloud.shape)
  print ("adj matrix0: ", adj_matrix.shape)
  print ("nn_idx0: ", nn_idx.shape)
  print ("edge_feature0: ", edge_feature.shape)

#   with tf.variable_scope('transform_net1') as sc:
#     transform = input_transform_net(edge_feature, is_training, bn_decay, K=K)
#     end_points['transform'] = transform
#     print ("Transform: ", transform.shape)
    
#   point_cloud_transformed = tf.matmul(point_cloud, transform)
#   print ("Transformed cloud: ", point_cloud_transformed.shape)
#   adj_matrix = dgcnn_util.pairwise_distance(point_cloud_transformed)
#   print ("adj matrix: ", adj_matrix.shape)
#   nn_idx = dgcnn_util.knn(adj_matrix, k=k)
#   print ("nn_idx: ", nn_idx.shape)
#   edge_feature = dgcnn_util.get_edge_feature(point_cloud_transformed, nn_idx=nn_idx, k=k)
#   print ("edge_feature: ", edge_feature.shape)
  
  net = dgcnn_util.conv2d(edge_feature, 64, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn1', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net1 = net
  end_points['dgcnn1'] = net1

  print ("net: ", net.shape)
  adj_matrix = dgcnn_util.pairwise_distance(net)
  print ("adj matrix: ", adj_matrix.shape)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  print ("nn_idx: ", nn_idx.shape)
  edge_feature = dgcnn_util.get_edge_feature(net, nn_idx=nn_idx, k=k)
  print ("edge_feature: ", edge_feature.shape)

  net = dgcnn_util.conv2d(edge_feature, 64, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn2', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net2 = net
  end_points['dgcnn2'] = net2
 
  adj_matrix = dgcnn_util.pairwise_distance(net)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(net, nn_idx=nn_idx, k=k) 
  
  print ("net2: ", net.shape)
  print ("adj matrix2: ", adj_matrix.shape)
  print ("nn_idx2: ", nn_idx.shape)
  print ("edge_feature2: ", edge_feature.shape)

  net = dgcnn_util.conv2d(edge_feature, 64, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn3', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net3 = net
  end_points['dgcnn3'] = net3

  adj_matrix = dgcnn_util.pairwise_distance(net)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(net, nn_idx=nn_idx, k=k)  
  
  print ("net3: ", net.shape)
  print ("adj matrix3: ", adj_matrix.shape)
  print ("nn_idx3: ", nn_idx.shape)
  print ("edge_feature3: ", edge_feature.shape)
  
  net = dgcnn_util.conv2d(edge_feature, 128, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn4', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net4 = net
  end_points['dgcnn4'] = net4
  
  print ("dgcnn1", net1)
  print ("dgcnn2", net2)
  print ("dgcnn3", net3)
  print ("dgcnn4", net4)
  
  net = dgcnn_util.conv2d(tf.concat([net1, net2, net3, net4], axis=-1), 1024, [1, 1], 
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='agg', bn_decay=bn_decay)
 
  net = tf.reduce_max(net, axis=1, keep_dims=True) 
  print ("dgcnn-agg", net)
  if base:
    end_points['DGCNN_PreFC'] = net
    return end_points
  # MLP on global point cloud vector
  net = tf.reshape(net, [batch_size, -1]) 
  net = dgcnn_util.fully_connected(net, 512, bn=True, is_training=is_training,
                                scope='fc1', bn_decay=bn_decay)
  net = dgcnn_util.dropout(net, keep_prob=0.5, is_training=is_training,
                         scope='dp1')
  net = dgcnn_util.fully_connected(net, 256, bn=True, is_training=is_training,
                                scope='fc2', bn_decay=bn_decay)
  net = dgcnn_util.dropout(net, keep_prob=0.5, is_training=is_training,
                        scope='dp2')
  net = dgcnn_util.fully_connected(net, num_classes, activation_fn=None, scope='fc3')
  
  end_points['Logits'] = net
  end_points['Probabilities'] = tf.nn.softmax(net, name='Probabilities')

  return net, end_points

def get_loss(pred, label, end_points, num_classes=49):
  """ pred: B*NUM_CLASSES,
      label: B, """
  labels = tf.one_hot(indices=label, depth=num_classes)
  #loss = tf.losses.softmax_cross_entropy(onehot_labels=labels, logits=pred, label_smoothing=0.2)
  loss = tf.losses.softmax_cross_entropy(onehot_labels=labels, logits=pred)
  classify_loss = tf.reduce_mean(loss)
  return classify_loss


def get_model_v3(point_cloud, num_classes, is_training, bn_decay=None):
  """ Classification PointNet, input is BxNx3, output Bx40 """
  batch_size = point_cloud.get_shape()[0].value
  num_point = point_cloud.get_shape()[1].value
  end_points = {}
  k = 20

  adj_matrix = dgcnn_util.pairwise_distance(point_cloud)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(point_cloud, nn_idx=nn_idx, k=k)

  with tf.variable_scope('transform_net1') as sc:
    transform = input_transform_net(edge_feature, is_training, bn_decay, K=3)

  point_cloud_transformed = tf.matmul(point_cloud, transform)
  adj_matrix = dgcnn_util.pairwise_distance(point_cloud_transformed)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(point_cloud_transformed, nn_idx=nn_idx, k=k)

  net = dgcnn_util.conv2d(edge_feature, 64, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn1', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net1 = net
  end_points['dgcnn1'] = net1

  adj_matrix = dgcnn_util.pairwise_distance(net)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(net, nn_idx=nn_idx, k=k)

  net = dgcnn_util.conv2d(edge_feature, 64, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn2', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net2 = net
  end_points['dgcnn2'] = net2
 
  adj_matrix = dgcnn_util.pairwise_distance(net)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(net, nn_idx=nn_idx, k=k)  

  net = dgcnn_util.conv2d(edge_feature, 64, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn3', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net3 = net
  end_points['dgcnn3'] = net3

  adj_matrix = dgcnn_util.pairwise_distance(net)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(net, nn_idx=nn_idx, k=k)  
  
  net = dgcnn_util.conv2d(edge_feature, 128, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn4', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net4 = net
  end_points['dgcnn4'] = net4
  
  print ("dgcnn1", net1)
  print ("dgcnn2", net2)
  print ("dgcnn3", net3)
  print ("dgcnn4", net4)
  
  net = dgcnn_util.conv2d(tf.concat([net1, net2, net3, net4], axis=-1), 1024, [1, 1], 
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='agg', bn_decay=bn_decay)
 
  net = tf.reduce_max(net, axis=1, keep_dims=True) 
  print ("dgcnn-agg", net)
  end_points['dgcnn-agg'] = net
  
  # MLP on global point cloud vector
  net = tf.reshape(net, [batch_size, -1]) 
  net = dgcnn_util.fully_connected(net, 512, bn=True, is_training=is_training,
                                scope='fc1', bn_decay=bn_decay)
  net = dgcnn_util.dropout(net, keep_prob=0.5, is_training=is_training,
                         scope='dp1')
  net = dgcnn_util.fully_connected(net, 256, bn=True, is_training=is_training,
                                scope='fc2', bn_decay=bn_decay)
  net = dgcnn_util.dropout(net, keep_prob=0.5, is_training=is_training,
                        scope='dp2')
  net = dgcnn_util.fully_connected(net, num_classes, activation_fn=None, scope='fc3')
  
  end_points['Logits'] = net
  end_points['Predictions'] = tf.nn.softmax(net, name='Predictions')

  return net, end_points

def get_model_v2(point_cloud, num_classes, is_training, bn_decay=None):
  """ Classification PointNet, input is BxNx3, output Bx40 """
  batch_size = point_cloud.get_shape()[0].value
  num_point = point_cloud.get_shape()[1].value
  end_points = {}
  k = 20

  adj_matrix = dgcnn_util.pairwise_distance(point_cloud)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(point_cloud, nn_idx=nn_idx, k=k)

  with tf.variable_scope('transform_net1') as sc:
    transform = input_transform_net(edge_feature, is_training, bn_decay, K=3)

  point_cloud_transformed = tf.matmul(point_cloud, transform)
  adj_matrix = dgcnn_util.pairwise_distance(point_cloud_transformed)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(point_cloud_transformed, nn_idx=nn_idx, k=k)

  net = dgcnn_util.conv2d(edge_feature, 64, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn1', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net1 = net
  end_points['dgcnn1'] = net1

  adj_matrix = dgcnn_util.pairwise_distance(net)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(net, nn_idx=nn_idx, k=k)

  net = dgcnn_util.conv2d(edge_feature, 64, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn2', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net2 = net
  end_points['dgcnn2'] = net2
 
  adj_matrix = dgcnn_util.pairwise_distance(net)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(net, nn_idx=nn_idx, k=k)  

  net = dgcnn_util.conv2d(edge_feature, 64, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn3', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net3 = net
  end_points['dgcnn3'] = net3

  adj_matrix = dgcnn_util.pairwise_distance(net)
  nn_idx = dgcnn_util.knn(adj_matrix, k=k)
  edge_feature = dgcnn_util.get_edge_feature(net, nn_idx=nn_idx, k=k)  
  
  net = dgcnn_util.conv2d(edge_feature, 128, [1,1],
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='dgcnn4', bn_decay=bn_decay)
  net = tf.reduce_max(net, axis=-2, keep_dims=True)
  net4 = net
  end_points['dgcnn4'] = net4
  
  print ("dgcnn1", net1)
  print ("dgcnn2", net2)
  print ("dgcnn3", net3)
  print ("dgcnn4", net4)
  
  net = dgcnn_util.conv2d(tf.concat([net1, net2, net3, net4], axis=-1), 1024, [1, 1], 
                       padding='VALID', stride=[1,1],
                       bn=True, is_training=is_training,
                       scope='agg', bn_decay=bn_decay)
 
  net = tf.reduce_max(net, axis=1, keep_dims=True) 
  net = tf.reshape(net, [batch_size, -1])
  end_points['dgcnn-agg'] = net
  print ("agg: ", net.shape)
  
  net = dgcnn_util.fully_connected(net, 512, bn=True, is_training=is_training,scope='fc0', bn_decay=bn_decay)
  net = dgcnn_util.dropout(net, keep_prob=0.5, is_training=is_training,scope='dp0')
  end_points['fc0'] = net
  print ("fc0: ", net.shape)

  net = dgcnn_util.fully_connected(net, 512, bn=True, is_training=is_training, scope='fc1', bn_decay=bn_decay)
  net = dgcnn_util.dropout(net, keep_prob=0.5, is_training=is_training,scope='dp1')
  end_points['fc1'] = net
  print ("fc1: ", net.shape)
  
  net = dgcnn_util.fully_connected(net, 512, bn=True, is_training=is_training,scope='fc2', bn_decay=bn_decay)
  net = dgcnn_util.dropout(net, keep_prob=0.5, is_training=is_training, scope='dp2')
  end_points['fc2'] = net
  print ("fc2: ", net.shape)

  return net, end_points


# if __name__=='__main__':
#   batch_size = 2
#   num_pt = 124
#   pos_dim = 3

#   input_feed = np.random.rand(batch_size, num_pt, pos_dim)
#   label_feed = np.random.rand(batch_size)
#   label_feed[label_feed>=0.5] = 1
#   label_feed[label_feed<0.5] = 0
#   label_feed = label_feed.astype(np.int32)

#   # # np.save('./debug/input_feed.npy', input_feed)
#   # input_feed = np.load('./debug/input_feed.npy')
#   # print input_feed

#   with tf.Graph().as_default():
#     input_pl, label_pl = placeholder_inputs(batch_size, num_pt)
#     pos, ftr = get_model(input_pl, tf.constant(True))
#     # loss = get_loss(logits, label_pl, None)

#     with tf.Session() as sess:
#       sess.run(tf.global_variables_initializer())
#       feed_dict = {input_pl: input_feed, label_pl: label_feed}
#       res1, res2 = sess.run([pos, ftr], feed_dict=feed_dict)
#       print res1.shape
#       print res1

#       print res2.shape
#       print res2












