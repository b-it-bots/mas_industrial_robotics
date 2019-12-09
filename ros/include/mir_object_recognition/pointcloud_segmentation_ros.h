/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MIR_OBJECT_RECOGNITION_POINTCLOUD_SEGMENTATION_ROS_H
#define MIR_OBJECT_RECOGNITION_POINTCLOUD_SEGMENTATION_ROS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include "mcr_scene_segmentation/cloud_accumulation.h"
#include <mcr_scene_segmentation/scene_segmentation.h>

#include <mas_perception_msgs/ObjectList.h>

#include <dynamic_reconfigure/server.h>
#include <mcr_scene_segmentation/SceneSegmentationConfig.h>
#include <string>

#include <mas_perception_libs/bounding_box.h>

//using mas_perception_libs::BoundingBox;

/**
 * This node subscribes to pointcloud topic.
 * Inputs:
 * ~event_in:
 *      - e_start: starts subscribing to pointcloud topic
 *      - e_add_cloud_start: adds pointcloud to octree, if it is on dataset collection mode,
 *                           the node will start segmenting the pointcloud.
 *      - e_add_cloud_stop: stops adding pointcloud to octree
 *      - e_find_plane: finds the plane and publishes workspace height
 *      - e_segment: starts segmentation and publish ObjectList
 *      - e_reset: clears accumulated cloud
 *      - e_stop: stops subscribing and clears accumulated pointcloud
 * Outputs:
 * ~event_out:
 *      - e_started: started listening to new messages
 *      - e_add_cloud_started: started adding the cloud to octree
 *      - e_add_cloud_stopped: stopped adding the cloud to octree
 *      - e_done: started finding the plane or started segmenting the pointcloud
 *      - e_stopped: stopped subscribing and cleared accumulated pointcloud
 */

class PointcloudSegmentationROS
{
    public:
        PointcloudSegmentationROS(ros::NodeHandle nh_sub);
        virtual ~PointcloudSegmentationROS();
    
    private:
        ros::NodeHandle nh_;

        ros::ServiceClient recognize_service;
        std::string object_recognizer_service_name_;

        tf::TransformListener transform_listener_;

        CloudAccumulation::UPtr cloud_accumulation_;

        bool add_to_octree_;
        int pcl_object_id_;
        double octree_resolution_;

        Eigen::Vector4f model_coefficients_;
        double workspace_height_;

    private:
        void findPlane();
        
        void savePcd(const PointCloud::ConstPtr &cloud, std::string obj_name);
    
    public:
        void reset_cloud_accumulation();
        void add_cloud_accumulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
        void segment_cloud(mas_perception_msgs::ObjectList &obj_list,
                            std::vector<PointCloud::Ptr> &clusters);
        void get3DBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                                                const Eigen::Vector3f& normal, 
                                                mas_perception_libs::BoundingBox &bbox,  
                                                mas_perception_msgs::BoundingBox& bounding_box_msg);
        geometry_msgs::PoseStamped getPose(const mas_perception_libs::BoundingBox &box);
    
    public:
        SceneSegmentation scene_segmentation_;
        double object_height_above_workspace_;
        std::string frame_id_;
        Eigen::Vector3f getPlaneNormal();
        double getWorkspaceHeight();
        void resetPclObjectId();
};

#endif  // MIR_OBJECT_RECOGNITION_POINTCLOUD_SEGMENTATION_ROS_H
