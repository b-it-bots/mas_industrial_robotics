/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#include <mas_perception_msgs/BoundingBox.h>
#include <mas_perception_msgs/BoundingBoxList.h>
#include <mas_perception_msgs/ObjectList.h>
#include <mas_perception_msgs/RecognizeObject.h>
#include "mcr_scene_segmentation/impl/helpers.hpp"
#include <mas_perception_libs/bounding_box.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <mir_object_recognition/pointcloud_segmentation_ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

PointcloudSegmentationROS::PointcloudSegmentationROS(ros::NodeHandle nh, boost::shared_ptr<tf::TransformListener> tf_listener): 
    nh_(nh),
    add_to_octree_(false), 
    pcl_object_id_(0),
    tf_listener_(tf_listener)
{    
    nh_.param("octree_resolution", octree_resolution_, 0.0025);
    cloud_accumulation_ = CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution_));
    scene_segmentation_ = SceneSegmentationUPtr(new SceneSegmentation());
    if (!tf_listener_)
    {
      ROS_ERROR_THROTTLE(2.0, "[PointcloudSegmentationROS]: TF listener not initialized.");
    }
}

PointcloudSegmentationROS::~PointcloudSegmentationROS()
{
}

void PointcloudSegmentationROS::segmentCloud(mas_perception_msgs::ObjectList &object_list, 
                                             std::vector<PointCloud::Ptr> &clusters)
{
    PointCloud::Ptr cloud(new PointCloud);
    cloud_accumulation_->getAccumulatedCloud(*cloud);
    
    std::vector<BoundingBox> boxes;
    PointCloud::Ptr debug = scene_segmentation_->segment_scene(cloud, clusters, boxes, model_coefficients_, workspace_height_);
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

        //TODO: add tranform pose to utils 
        geometry_msgs::PoseStamped pose = getPose(boxes[i]);
        pose.header.stamp = now;
        pose.header.frame_id = frame_id_;

        object_list.objects[i].pose = pose;
        object_list.objects[i].database_id = pcl_object_id_;
        pcl_object_id_++;
    }
}

geometry_msgs::PoseStamped PointcloudSegmentationROS::getPose(const BoundingBox &box)
{
    BoundingBox::Points vertices = box.getVertices();
    Eigen::Vector3f n1;
    Eigen::Vector3f n2;
    Eigen::Vector3f n3 = (vertices[4] - vertices[0]) / (vertices[4] - vertices[0]).norm();
    if ((vertices[1] - vertices[0]).norm() > (vertices[3] - vertices[0]).norm())
    {
        n1 = (vertices[1] - vertices[0]) / (vertices[1] - vertices[0]).norm();
    }
    else
    {
        n1 = (vertices[3] - vertices[0]) / (vertices[3] - vertices[0]).norm();
    }
    n2 = n3.cross(n1);
    //TODO: debug mode only
    ROS_INFO_STREAM("got norms");
    Eigen::Matrix3f m;
    m << n1 , n2 , n3;
    Eigen::Quaternion<float> q(m);
    q.normalize();

    double workspace_height = (vertices[0](2) + vertices[1](2) + vertices[2](2) + vertices[3](2)) / 4.0;

    Eigen::Vector3f centroid = box.getCenter();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = centroid(0);
    pose.pose.position.y = centroid(1);
    pose.pose.position.z = workspace_height + object_height_above_workspace_;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
}

void PointcloudSegmentationROS::transformPose(std::string &target_frame, 
                                              geometry_msgs::PoseStamped &pose, 
                                              geometry_msgs::PoseStamped &transformed_pose)
{
    if (tf_listener_)
    {
        try
        {
            ros::Time common_time;
            tf_listener_->getLatestCommonTime(pose.header.frame_id, target_frame, common_time, NULL);
            pose.header.stamp = common_time;
            tf_listener_->waitForTransform(target_frame, pose.header.frame_id, common_time, ros::Duration(0.1));
            tf_listener_->transformPose(target_frame, pose, transformed_pose);
        }
        catch(tf::LookupException& ex)
        {
            ROS_WARN("Failed to transform pose: (%s)", ex.what());
            transformed_pose = pose;
        }
    }
    else
    {
        ROS_ERROR_THROTTLE(2.0, "[PointcloudSegmentationROS]: TF listener not initialized.");
        transformed_pose = pose;
    }
}

void PointcloudSegmentationROS::savePcd(const PointCloud::ConstPtr &pointcloud, std::string logdir, std::string obj_name)
{
    std::stringstream filename;
    ros::Time time_now = ros::Time::now();
    filename.str("");
    filename << logdir << obj_name << "_" << time_now <<".pcd";
    pcl::io::savePCDFileASCII(filename.str(), *pointcloud);
}

void PointcloudSegmentationROS::resetCloudAccumulation()
{
    cloud_accumulation_->reset();
}

void PointcloudSegmentationROS::addCloudAccumulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    cloud_accumulation_->addCloud(cloud);
}

void PointcloudSegmentationROS::get3DBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                                                 const Eigen::Vector3f& normal, BoundingBox &bbox,
                                                 mas_perception_msgs::BoundingBox& bounding_box_msg)
{
    bbox = BoundingBox::create(cloud->points, normal);
    convertBoundingBox(bbox, bounding_box_msg);
}

Eigen::Vector3f PointcloudSegmentationROS::getPlaneNormal()
{
    Eigen::Vector3f normal(model_coefficients_[0],
                           model_coefficients_[1],
                           model_coefficients_[2]);
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
