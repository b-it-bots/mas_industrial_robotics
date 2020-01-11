/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MIR_OBJECT_RECOGNITION_POINTCLOUD_SEGMENTATION_ROS_H
#define MIR_OBJECT_RECOGNITION_POINTCLOUD_SEGMENTATION_ROS_H

#include <string>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <mas_perception_msgs/BoundingBox.h>
#include <mas_perception_msgs/BoundingBoxList.h>
#include <mas_perception_msgs/ObjectList.h>
#include <mas_perception_msgs/RecognizeObject.h>

#include <mir_object_segmentation/cloud_accumulation.h>
#include <mir_object_segmentation/scene_segmentation.h>
#include <mir_object_segmentation/SceneSegmentationConfig.h>
#include <mir_object_segmentation/bounding_box.h>
#include <mir_object_segmentation/impl/helpers.hpp>

class PointcloudSegmentationROS
{
    public:
        /** Constructor
         * \param[in] NodeHandle
         * \param[in] Transform listener
         * */
        PointcloudSegmentationROS(ros::NodeHandle nh, 
                                  boost::shared_ptr<tf::TransformListener> tf_listener=nullptr);

        virtual ~PointcloudSegmentationROS();
    
    private:
        ros::NodeHandle nh_;
        ros::ServiceClient recognize_service;

         /** Create unique pointer object of cloud_accumulation */
        CloudAccumulation::UPtr cloud_accumulation_;
        
        pcl::ModelCoefficients::Ptr model_coefficients_;
        boost::shared_ptr<tf::TransformListener> tf_listener_;
 
        bool add_to_octree_;
        int pcl_object_id_;
        double octree_resolution_;
        double workspace_height_;

    public:
        /** \brief Find plane, segment table top point cloud and cluster them
         * \param[out] Object list with unknown labels
         * \param[out] 3D table top object clusters
         * */
        void segmentCloud(mas_perception_msgs::ObjectList &obj_list,
                            std::vector<PointCloud::Ptr> &clusters);
        
        /** \brief Compute 3D bounding box parallel with the plane
         * \param[in] Pointcloud pointer
         * \param[in] Plane normal
         * \param[Out] Generated bounding box
         * \param[out] Bounding box message
         * */
        void get3DBoundingBox(const PointCloud::ConstPtr &cloud, 
                              const Eigen::Vector3f& normal, 
                              BoundingBox &bbox,  
                              mas_perception_msgs::BoundingBox &bounding_box_msg);
        
        /** \brief Estimate pose given bounding box
         * \param[in] Bounding box
         * returns PoseStamped
         * */
        geometry_msgs::PoseStamped getPose(const BoundingBox &box);

        /** \brief Transform pose
         * \param[in] target_frame id
         * \param[in] Source pose stamped
         * \param[out] Transformed pose stamped
         * */
        void transformPose(std::string &target_frame, 
                           geometry_msgs::PoseStamped &pose, 
                           geometry_msgs::PoseStamped &transformed_pose);
        /** \brief Save pointcloud
         * \param[in] logdir (default="/tmp")
         * \param[in] obj_name (default="unknown")
         * */
        void savePcd(const PointCloud::ConstPtr &pointcloud, 
                     std::string logdir="/tmp", 
                     std::string obj_name="unknown");
        /** \brief Reset accumulated cloud */
        void resetCloudAccumulation();

        /** \brief Accumulate pointcloud
         * \param[in] Pointcloud to accumulate
         * */
        void addCloudAccumulation(const PointCloud::Ptr &cloud);

        /** Returns plane normal */
        Eigen::Vector3f getPlaneNormal();

        /** Returns plane height */
        double getWorkspaceHeight();

        /** Reset 3D object id */
        void resetPclObjectId();
    
    public:
        /** Create unique pointer for object scene_segmentation */
        typedef std::unique_ptr<SceneSegmentation> SceneSegmentationUPtr;
        SceneSegmentationUPtr scene_segmentation_;

        double object_height_above_workspace_;
        std::string frame_id_;

};

#endif  // MIR_OBJECT_RECOGNITION_POINTCLOUD_SEGMENTATION_ROS_H
