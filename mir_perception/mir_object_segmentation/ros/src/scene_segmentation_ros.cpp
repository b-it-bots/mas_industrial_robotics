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
#include <mir_object_segmentation/scene_segmentation_ros.h>

#include <mir_perception_utils/object_utils_ros.h>

namespace mpu = mir_perception_utils;

SceneSegmentationROS::SceneSegmentationROS(double octree_resolution): 
    octree_resolution_(octree_resolution),
    pcl_object_id_(0)
{    
    cloud_accumulation_ = CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution_));
    scene_segmentation_ = SceneSegmentationUPtr(new SceneSegmentation());
    model_coefficients_ = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
}

SceneSegmentationROS::~SceneSegmentationROS()
{
}

void SceneSegmentationROS::segmentCloud(const PointCloud::ConstPtr &cloud,
                                        mas_perception_msgs::ObjectList &object_list, 
                                        std::vector<PointCloud::Ptr> &clusters,
                                        std::vector<BoundingBox> &boxes)
{
    std::string frame_id = cloud->header.frame_id;
    PointCloud::Ptr debug = scene_segmentation_->segmentScene(cloud, clusters, boxes, model_coefficients_, workspace_height_);
    debug->header.frame_id = frame_id;

    object_list.objects.resize(boxes.size());
    ros::Time now = ros::Time::now();
    for (int i = 0; i < clusters.size(); i++)
    {
        sensor_msgs::PointCloud2 ros_cloud;
        ros_cloud.header.frame_id = frame_id;
        pcl::PCLPointCloud2 pc2;
        pcl::toPCLPointCloud2(*clusters[i], pc2);
        pcl_conversions::fromPCL(pc2, ros_cloud);
        // Assign unknown for every object by default then recognize it later
        object_list.objects[i].pointcloud = ros_cloud;
        object_list.objects[i].name = "unknown";
        object_list.objects[i].probability = 0.0;

        geometry_msgs::PoseStamped pose;
        mpu::object::estimatePose(boxes[i], pose);
        pose.header.stamp = now;
        pose.header.frame_id = frame_id;

        object_list.objects[i].pose = pose;
        object_list.objects[i].database_id = pcl_object_id_;
        pcl_object_id_++;
    }
}

void SceneSegmentationROS::findPlane(const PointCloud::ConstPtr &cloud_in,
                                     PointCloud::Ptr &cloud_debug)
{
    PointCloud::Ptr hull(new PointCloud);
    cloud_debug = scene_segmentation_->findPlane(cloud_in, hull, model_coefficients_, workspace_height_);
    cloud_debug->header.frame_id = cloud_in->header.frame_id;
}

void SceneSegmentationROS::resetCloudAccumulation()
{
    cloud_accumulation_->reset();
}

void SceneSegmentationROS::addCloudAccumulation(const PointCloud::Ptr &cloud)
{
    cloud_accumulation_->addCloud(cloud);
}

void SceneSegmentationROS::getCloudAccumulation(PointCloud::Ptr &cloud)
{
    cloud_accumulation_->getAccumulatedCloud(*cloud);
}

Eigen::Vector3f SceneSegmentationROS::getPlaneNormal()
{
    Eigen::Vector3f normal(model_coefficients_->values[0], model_coefficients_->values[1], model_coefficients_->values[2]);
    return normal;
}

double SceneSegmentationROS::getWorkspaceHeight()
{
    return workspace_height_;
}

void SceneSegmentationROS::resetPclObjectId()
{
    pcl_object_id_ = 0;
}

void SceneSegmentationROS::setVoxelGridParams(double voxel_leaf_size, 
            std::string voxel_filter_field_name,
            double voxel_filter_limit_min, 
            double voxel_filter_limit_max)
{
    scene_segmentation_->setVoxelGridParams(voxel_leaf_size, 
            voxel_filter_field_name,
            voxel_filter_limit_min, 
            voxel_filter_limit_max);
}

void SceneSegmentationROS::setPassthroughParams(bool enable_passthrough_filter,
            std::string passthrough_filter_field_name,
            double passthrough_filter_limit_min,
            double passthrough_filter_limit_max)
{
    scene_segmentation_->setPassthroughParams(enable_passthrough_filter,
            passthrough_filter_field_name,
            passthrough_filter_limit_min,
            passthrough_filter_limit_max);
}

void SceneSegmentationROS::setNormalParams(double normal_radius_search)
{
    scene_segmentation_->setNormalParams(normal_radius_search);
}

void SceneSegmentationROS::setSACParams(int sac_max_iterations, 
            double sac_distance_threshold,
            bool sac_optimize_coefficients,
            Eigen::Vector3f axis, 
            double sac_eps_angle,
            double sac_normal_distance_weight)
{
    scene_segmentation_->setSACParams(sac_max_iterations, 
            sac_distance_threshold,
            sac_optimize_coefficients,
            axis, 
            sac_eps_angle,
            sac_normal_distance_weight);
}

void SceneSegmentationROS::setPrismParams(double prism_min_height, double prism_max_height)
{
    scene_segmentation_->setPrismParams(prism_min_height, prism_max_height);
}

void SceneSegmentationROS::setOutlierParams(double outlier_radius_search, double outlier_min_neighbors)
{
    scene_segmentation_->setOutlierParams(outlier_radius_search, outlier_min_neighbors);
}

void SceneSegmentationROS::setClusterParams(double cluster_tolerance, int cluster_min_size, 
            int cluster_max_size, double cluster_min_height, 
            double cluster_max_height, double cluster_max_length,
            double cluster_min_distance_to_polygon)
{
    scene_segmentation_->setClusterParams(cluster_tolerance, cluster_min_size, cluster_max_size,
            cluster_min_height, cluster_max_height, cluster_max_length,
            cluster_min_distance_to_polygon);
}
    
    
