/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */
#ifndef MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_H
#define MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_H

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

#include <mas_perception_libs/point_cloud_utils.h>
#include <mas_perception_msgs/ObjectList.h>
#include <mas_perception_msgs/Object.h>

class MultimodalObjectRecognitionUtils
{
    public:
        MultimodalObjectRecognitionUtils(boost::shared_ptr<tf::TransformListener> tf_listener=nullptr);
        virtual ~MultimodalObjectRecognitionUtils();

    public:
        /** \brief Estimate pose given a cluster generated from 3D proposal. Passthrough filter
         *         is also applied to remove part of the table on the cluster. Returns estimated pose.
         *         The orientation is calculated based on
         *         http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
         * \param[in] the generated cluster
         * \param[in] the label of the cluster (optional), and is used to filter round objects from passthrough
        */
        geometry_msgs::PoseStamped estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyz_input_cloud, 
                                                std::string name = "None");
        
        /** \brief Make adjustment for AXIS and Bolt (M20_100) pose. Adjust it so that the pose
         *         is close to the center of mass (the tip).
         * \param[in] object.pointcloud
         * 
        */
        void adjustAxisBoltPose(mas_perception_msgs::Object &object);

        /** \brief Adjust container pose by finding the largest cluster to find the center
         * \param[in] object.pointcloud
         * \param[in] the height adjustment of the container, default 10cm
        */
        void adjustContainerPose(mas_perception_msgs::Object &container_object, float rgb_container_height=0.1);

        /** \brief Transform pose 
         * \param[in] source_frame id
         * \param[in] target_frame id
         * \param[in] Source pose stamped
         * \param[out] Transformed pose stamped
        */
        void transformPose(std::string &source_frame, std::string &target_frame, 
                           geometry_msgs::PoseStamped &pose, 
                           geometry_msgs::PoseStamped &transformed_pose);

        /** \brief Save debug image if debug_mode is enabled
         * \param[in] image with boundix boxes of objects drawn
         * \param[in] raw_image
         * \param[in] logdir (default /tmp)
        */
        void saveDebugImage(const cv_bridge::CvImagePtr &cv_image_ptr, 
                            const sensor_msgs::ImageConstPtr &raw_image, 
                            std::string logdir="/tmp/");
    
    private:
        // transform listener
        boost::shared_ptr<tf::TransformListener> tf_listener_;


};
#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_H
