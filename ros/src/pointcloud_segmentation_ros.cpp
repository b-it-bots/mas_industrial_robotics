/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */

#include <iostream>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mir_perception_utils/impl/helpers.hpp>
#include <mir_object_recognition/pointcloud_segmentation_ros.h>

PointcloudSegmentationROS::PointcloudSegmentationROS(ros::NodeHandle nh): 
    nh_(nh),
    pcl_object_id_(0)
{    
    nh_.param("octree_resolution", octree_resolution_, 0.0025);
    cloud_accumulation_ = CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution_));
    scene_segmentation_ = SceneSegmentationUPtr(new SceneSegmentation());
    model_coefficients_ = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
    object_utils_ = ObjectUtilsUPtr(new ObjectUtils());
}

PointcloudSegmentationROS::~PointcloudSegmentationROS()
{
}

void PointcloudSegmentationROS::segmentCloud(mas_perception_msgs::ObjectList &object_list, 
                                             std::vector<PointCloud::Ptr> &clusters)
{
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = frame_id_;
    cloud_accumulation_->getAccumulatedCloud(*cloud);
        
    std::vector<BoundingBox> boxes;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    PointCloud::Ptr debug = scene_segmentation_->segmentScene(cloud, clusters, boxes, model_coefficients_, workspace_height_);
    debug->header.frame_id = "base_link";

    object_list.objects.resize(boxes.size());
    ros::Time now = ros::Time::now();
    for (int i = 0; i < clusters.size(); i++)
    {
        sensor_msgs::PointCloud2 ros_cloud;
        ros_cloud.header.frame_id = frame_id_;
        pcl::PCLPointCloud2 pc2;
        pcl::toPCLPointCloud2(*clusters[i], pc2);
        pcl_conversions::fromPCL(pc2, ros_cloud);
        // Assign unknown for every object by default then recognize it later
        object_list.objects[i].pointcloud = ros_cloud;
        object_list.objects[i].name = "unknown";
        object_list.objects[i].probability = 0.0;

        geometry_msgs::PoseStamped pose;
        object_utils_->estimatePose(boxes[i], pose);
        pose.header.stamp = now;
        pose.header.frame_id = frame_id_;

        object_list.objects[i].pose = pose;
        object_list.objects[i].database_id = pcl_object_id_;
        pcl_object_id_++;
    }
}

void PointcloudSegmentationROS::resetCloudAccumulation()
{
    cloud_accumulation_->reset();
}

void PointcloudSegmentationROS::addCloudAccumulation(const PointCloud::Ptr &cloud)
{
    cloud_accumulation_->addCloud(cloud);
}

Eigen::Vector3f PointcloudSegmentationROS::getPlaneNormal()
{
    Eigen::Vector3f normal(model_coefficients_->values[0], model_coefficients_->values[1], model_coefficients_->values[2]);
    return normal;
}

double PointcloudSegmentationROS::getWorkspaceHeight()
{
    return workspace_height_;
}

void PointcloudSegmentationROS::resetPclObjectId()
{
    pcl_object_id_ = 0;
}
