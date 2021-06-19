#! /usr/bin/env python
import numpy as np
import tf
import rospy
from my_pcl_tutorial.msg import Cavity
import json

br = tf.TransformBroadcaster()

with open('distribution_dict.json', 'r') as fp:
    distribution_dict = json.load(fp)
    for key in distribution_dict.keys():
        distribution_dict[key]['mu'] = np.array(distribution_dict[key]['mu'])
        distribution_dict[key]['cov'] = 16*np.array(distribution_dict[key]['cov']).reshape((2,2))

def gaussian_2d_cdf(x, mu, cov):
    mahalanobis_distance_squared = np.matmul(np.matmul((x-mu), np.linalg.pinv(cov)), (x-mu).T)
    return (1 - np.exp(-mahalanobis_distance_squared/2))        

def callback(msg):
    if msg.cov_minor > 2.5e-5:
        obj_name = 'unknown'
        max_cfd = 0
        obj_cov = np.array([msg.cov_minor, msg.cov_major])
        print "cov: " 
        print obj_cov
        for key in distribution_dict.keys():
            target_distribution = distribution_dict[key]
            target_cfd = 1 - gaussian_2d_cdf(obj_cov, target_distribution['mu'], target_distribution['cov'])            
            if target_cfd > 0.5 and target_cfd > max_cfd:
                obj_name = key
                max_cfd = target_cfd
                print (-2*np.log(target_cfd))**0.5
                print key + ": " + str(target_cfd)
        obj_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        obj_quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

        br.sendTransform(obj_pose, obj_quat, rospy.Time.now(), obj_name, 'camera_color_optical_frame')
        print "object detected is : " + obj_name + ": " + str(max_cfd)
    
if __name__ == '__main__':
    rospy.init_node('ppt_classifier_node')

    rospy.Subscriber("cavity", Cavity, callback)
    rospy.spin()    
