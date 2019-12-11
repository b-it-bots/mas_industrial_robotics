/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MIR_OBJECT_RECOGNITION_POINTCLOUD_SEGMENTATION_ROS_H
#define MIR_OBJECT_RECOGNITION_POINTCLOUD_SEGMENTATION_ROS_H

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>

#include <mcr_scene_segmentation/cloud_accumulation.h>
#include <mcr_scene_segmentation/scene_segmentation.h>
#include <mcr_scene_segmentation/SceneSegmentationConfig.h>
#include <mas_perception_libs/bounding_box.h>
#include <mas_perception_msgs/ObjectList.h>

#include <dynamic_reconfigure/server.h>

class PointcloudSegmentationROS
{
    public:
        PointcloudSegmentationROS(ros::NodeHandle nh, 
                                  boost::shared_ptr<tf::TransformListener> tf_listener=nullptr);
        virtual ~PointcloudSegmentationROS();
    
    private:
        ros::NodeHandle nh_;

        ros::ServiceClient recognize_service;
        std::string object_recognizer_service_name_;

        boost::shared_ptr<tf::TransformListener> tf_listener_;

        // Create unique pointer object of cloud_accumulation
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
        void resetCloudAccumulation();
        void addCloudAccumulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
        void segmentCloud(mas_perception_msgs::ObjectList &obj_list,
                            std::vector<PointCloud::Ptr> &clusters);
        void get3DBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                                                const Eigen::Vector3f& normal, 
                                                mas_perception_libs::BoundingBox &bbox,  
                                                mas_perception_msgs::BoundingBox& bounding_box_msg);
        geometry_msgs::PoseStamped getPose(const mas_perception_libs::BoundingBox &box);

        void transformPose(std::string &source_frame, std::string &target_frame, 
                            geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &transformed_pose);
    
    public:
        SceneSegmentation scene_segmentation_;
        double object_height_above_workspace_;
        std::string frame_id_;
        Eigen::Vector3f getPlaneNormal();
        double getWorkspaceHeight();
        void resetPclObjectId();
};

#endif  // MIR_OBJECT_RECOGNITION_POINTCLOUD_SEGMENTATION_ROS_H
