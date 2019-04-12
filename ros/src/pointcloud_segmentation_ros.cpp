/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#include <mcr_perception_msgs/BoundingBox.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/RecognizeObject.h>
#include "mcr_scene_segmentation/impl/helpers.hpp"
#include <mas_perception_libs/bounding_box.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"

#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <mir_object_recognition/pointcloud_segmentation_ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

PointcloudSegmentationROS::PointcloudSegmentationROS(ros::NodeHandle nh): nh_(nh),
//    bounding_box_visualizer_("bounding_boxes", mcr::visualization::Color(mcr::visualization::Color::SEA_GREEN)),
//    cluster_visualizer_("tabletop_clusters"),
//    label_visualizer_("labels", mcr::visualization::Color(mcr::visualization::Color::TEAL)),
    add_to_octree_(false), object_id_(0), debug_mode_(false), dataset_collection_(false)
{    
    nh_.param("octree_resolution", octree_resolution_, 0.0025);
    cloud_accumulation_ = CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution_));

    nh_.param<std::string>("logdir", logdir_, "/tmp/");

}

PointcloudSegmentationROS::~PointcloudSegmentationROS()
{
}

void PointcloudSegmentationROS::segment_cloud(mcr_perception_msgs::ObjectList &obj_list, 
                                              std::vector<PointCloud::Ptr> &clusters)
{
    PointCloud::Ptr cloud(new PointCloud);
    //cloud->header.frame_id = frame_id_;
    cloud_accumulation_->getAccumulatedCloud(*cloud);
    
    //std::vector<PointCloud::Ptr> clusters;
    std::vector<BoundingBox> boxes;
    //double workspace_height;

    std::cout<<"Start segmenting cloud size "<<cloud->width<<", "<<cloud->height<<std::endl;
    PointCloud::Ptr debug = scene_segmentation_.segment_scene(cloud, clusters, boxes, workspace_height_);
    debug->header.frame_id = "base_link";

    //std_msgs::Float64 workspace_height_msg;
    //workspace_height_msg.data = workspace_height;
    
    //pub_workspace_height_.publish(workspace_height_msg);
    //pub_debug_.publish(*debug);

    mcr_perception_msgs::BoundingBoxList bounding_boxes;
    mcr_perception_msgs::ObjectList object_list;
    geometry_msgs::PoseArray poses;

    bounding_boxes.bounding_boxes.resize(boxes.size());
    object_list.objects.resize(boxes.size());

    std::vector<std::string> labels;

    //TODO:
    // 1. Conver to BBox in list
    // 2. Convert to roscloud in list
    // 3. Recognize object in list (return object list) using TOPIC
    // 4. Get poses in list (should include transform)
    // 

    // 2. Convert clusters to roscloud
    //std::vector<sensor_msgs::PointCloud2> ros_clouds;
    ros::Time now = ros::Time::now();
    for (int i = 0; i < clusters.size(); i++)
    {
        // this is defined in helper
        convertBoundingBox(boxes[i], bounding_boxes.bounding_boxes[i]);

        sensor_msgs::PointCloud2 ros_cloud;
        pcl::PCLPointCloud2 pc2;
        pcl::toPCLPointCloud2(*clusters[i], pc2);
        pcl_conversions::fromPCL(pc2, ros_cloud);
        //ros_clusters.push_back(ros_cloud);
        // Assign unknown for every object by default then recognize it later
        object_list.objects[i].pointcloud = ros_cloud;
        object_list.objects[i].label = "unknown";
        object_list.objects[i].probability = 0.0;

        //TODO: add tranform pose to utils 
        geometry_msgs::PoseStamped pose = getPose(boxes[i]);
        pose.header.stamp = now;
        pose.header.frame_id = frame_id_;
        std::string target_frame_id;
        if (nh_.hasParam("target_frame_id"))
        {
            nh_.param("target_frame_id", target_frame_id, frame_id_);
            if (target_frame_id != frame_id_)
            {
                try
                {
                    ros::Time common_time;
                    transform_listener_.getLatestCommonTime(frame_id_, target_frame_id, common_time, NULL);
                    pose.header.stamp = common_time;
                    transform_listener_.waitForTransform(target_frame_id, frame_id_, common_time, ros::Duration(0.1));
                    geometry_msgs::PoseStamped pose_transformed;
                    transform_listener_.transformPose(target_frame_id, pose, pose_transformed);
                    object_list.objects[i].pose = pose_transformed;
                }
                catch(tf::LookupException& ex)
                {
                    ROS_WARN("Failed to transform pose: (%s)", ex.what());
                    pose.header.stamp = now;
                    object_list.objects[i].pose = pose;
                }
            }
            else
            {
                object_list.objects[i].pose = pose;
            }
        }
        else
        {
            object_list.objects[i].pose = pose;
        }

        object_list.objects[i].database_id = object_id_;
        object_id_++;
    }

    // Recognize list of clouds
    // This has to be done in multimodal_obj_recog using topic
    // to make the recognition process concurent
/**
    mcr_perception_msgs::RecognizeObject srv;
    srv.request.cloud = ros_clusters;

    if (recognize_service.call(srv))
    {
        for (int i=0; i<object_list.objects.size(); i++)
        {
            object_list.objects[i].label = srv.response.name[i];
            object_list.objects[i].probability = srv.response.probability[i];
        }
    }
    else
    {
        ROS_WARN("Object recognition service call failed");   
    }
**/
    //pub_object_list_.publish(object_list);
    obj_list = object_list;
    
    //bounding_box_visualizer_.publish(bounding_boxes.bounding_boxes, frame_id_);
    //cluster_visualizer_.publish<PointT>(clusters, frame_id_);
    //label_visualizer_.publish(labels, poses);
}

void PointcloudSegmentationROS::savePcd(const PointCloud::ConstPtr &pointcloud, std::string obj_name)
{
    std::stringstream filename; // stringstream used for the conversion
    ros::Time time_now = ros::Time::now();
    filename.str(""); //clearing the stringstream
    if (debug_mode_)
    {
        filename << logdir_ << obj_name << "_" << time_now <<".pcd";
    }
    else
    {
        filename << logdir_ <<"pcd_" << time_now <<".pcd";
    }
    ROS_INFO_STREAM("Saving pointcloud to " << logdir_);
    pcl::io::savePCDFileASCII (filename.str(), *pointcloud);
}

void PointcloudSegmentationROS::findPlane()
{
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = frame_id_;
    cloud_accumulation_->getAccumulatedCloud(*cloud);

    double workspace_height;
    PointCloud::Ptr hull(new PointCloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    PointCloud::Ptr debug = scene_segmentation_.findPlane(cloud, hull, coefficients, workspace_height);
    debug->header.frame_id = cloud->header.frame_id;
    std_msgs::Float64 workspace_height_msg;
    workspace_height_msg.data = workspace_height;
    //pub_workspace_height_.publish(workspace_height_msg);
    //pub_debug_.publish(*debug);
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

// geometry_msgs::PoseStamped PointcloudSegmentationROS::transformPose(const BoundingBox &box)
// {

// }

void PointcloudSegmentationROS::reset_cloud_accumulation()
{
    cloud_accumulation_->reset();
}

void PointcloudSegmentationROS::add_cloud_accumulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    cloud_accumulation_->addCloud(cloud);
}

void PointcloudSegmentationROS::get3DBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                                                const Eigen::Vector3f& normal, 
                                                BoundingBox &bbox,
                                                mcr_perception_msgs::BoundingBox& bounding_box_msg)
{
    std::cout<<"Converting....."<<std::endl;
    bbox = BoundingBox::create(cloud->points, normal);
    convertBoundingBox(bbox, bounding_box_msg);
}

// void PointcloudSegmentationROS::eventCallback(const std_msgs::String::ConstPtr &msg)
// {
//     std_msgs::String event_out;
//     if (msg->data == "e_start")
//     {
//         sub_cloud_ = nh_.subscribe("input", 1, &PointcloudSegmentationROS::pointcloudCallback, this);
//         event_out.data = "e_started";
//     }
//     else if (msg->data == "e_add_cloud_start")
//     {
//         add_to_octree_ = true;
//         // Not needed so that not to affect the action server
//         return;
//     }
//     else if (msg->data == "e_add_cloud_stop")
//     {
//         add_to_octree_ = false;
//         event_out.data = "e_add_cloud_stopped";
//     }
//     else if (msg->data == "e_find_plane")
//     {
//         findPlane();
//         cloud_accumulation_->reset();
//         event_out.data = "e_done";
//     }
//     else if (msg->data == "e_segment")
//     {
//         segment();
//         cloud_accumulation_->reset();
//         event_out.data = "e_done";
//     }
//     else if (msg->data == "e_reset")
//     {
//         cloud_accumulation_->reset();
//         event_out.data = "e_reset";
//     }
//     else if (msg->data == "e_stop")
//     {
//         sub_cloud_.shutdown();
//         cloud_accumulation_->reset();
//         event_out.data = "e_stopped";
//     }
//     else
//     {
//         return;
//     }
//     pub_event_out_.publish(event_out);
// }

// void PointcloudSegmentationROS::configCallback(mcr_scene_segmentation::SceneSegmentationConfig &config, uint32_t level)
// {
//     scene_segmentation_.setVoxelGridParams(config.voxel_leaf_size, config.voxel_filter_field_name,
//             config.voxel_filter_limit_min, config.voxel_filter_limit_max);
//     scene_segmentation_.setPassthroughParams(config.passthrough_filter_field_name,
//             config.passthrough_filter_limit_min,
//             config.passthrough_filter_limit_max);
//     scene_segmentation_.setNormalParams(config.normal_radius_search);
//     scene_segmentation_.setSACParams(config.sac_max_iterations, config.sac_distance_threshold,
//             config.sac_optimize_coefficients, config.sac_eps_angle,
//             config.sac_normal_distance_weight);
//     std::cout<<"============================"<<std::endl;
//     std::cout<<"Normal radiuse %.2f"<<config.normal_radius_search<<std::endl;
//     scene_segmentation_.setPrismParams(config.prism_min_height, config.prism_max_height);
//     scene_segmentation_.setOutlierParams(config.outlier_radius_search, config.outlier_min_neighbors);
//     scene_segmentation_.setClusterParams(config.cluster_tolerance, config.cluster_min_size, config.cluster_max_size,
//             config.cluster_min_height, config.cluster_max_height, config.cluster_max_length,
//             config.cluster_min_distance_to_polygon);
//     object_height_above_workspace_ = config.object_height_above_workspace;
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "scene_segmentation_node");
//     PointcloudSegmentationROS scene_seg;
//     ros::spin();
//     return 0;
// }

