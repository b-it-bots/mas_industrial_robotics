/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MIR_OBJECT_SEGMENTATION_SCENE_SEGMENTATION_NODE_H
#define MIR_OBJECT_SEGMENTATION_SCENE_SEGMENTATION_NODE_H

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>

#include <mir_object_segmentation/scene_segmentation_ros.h>
#include <mir_object_segmentation/SceneSegmentationConfig.h>

/** \brief This node subscribes to pointcloud topic.
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
 * 
 * \author Mohammad Wasil, Santosh Thoduka
 */

namespace mpu = mir_perception_utils;
using mpu::visualization::BoundingBoxVisualizer;
using mpu::visualization::ClusteredPointCloudVisualizer;
using mpu::visualization::LabelVisualizer;
using mpu::visualization::Color;

class SceneSegmentationNode
{
    public:
        SceneSegmentationNode();
        virtual ~SceneSegmentationNode();

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_debug_;
        ros::Publisher pub_object_list_;
        ros::Publisher pub_event_out_;
        ros::Publisher pub_workspace_height_;

        ros::Subscriber sub_cloud_;
        ros::Subscriber sub_event_in_;

        dynamic_reconfigure::Server<mir_object_segmentation::SceneSegmentationConfig> server_;

        boost::shared_ptr<tf::TransformListener> tf_listener_;

        SceneSegmentationROS scene_segmentation_ros_;

        BoundingBoxVisualizer bounding_box_visualizer_;
        ClusteredPointCloudVisualizer cluster_visualizer_;
        LabelVisualizer label_visualizer_;

        // Parameters
        bool add_to_octree_;
        int object_id_;
        std::string target_frame_id_;
        std::string logdir_;

        // Dynamic reconfigurable parameters
        double octree_resolution_;
        double object_height_above_workspace_;

        //cluster
        bool center_cluster_;
        bool pad_cluster_;
        unsigned int padded_cluster_size_;

    private:
        void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg);
        void eventCallback(const std_msgs::String::ConstPtr &msg);
        void configCallback(mir_object_segmentation::SceneSegmentationConfig &config, uint32_t level);

        /** \brief Segment accumulated pointcloud, find the plane, 
         *         clusters table top objects, find object heights, and publish them.
         **/
        void segmentPointCloud();

        /** \brief Segment accumulated pointcloud, find the plane, 
         *         find the plane height, and publish it.
         **/
        void findPlane();
};

#endif  // MIR_OBJECT_SEGMENTATION_SCENE_SEGMENTATION_NODE_H
