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
        geometry_msgs::PoseStamped estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyz_input_cloud, std::string name);
        
        // Update object pose for axis and bolt
        void adjustAxisBoltPose(mas_perception_msgs::Object &object);
        // fix container pose
        void adjustContainerPose(mas_perception_msgs::Object &container_object, float rgb_container_height);
        void transformPose(std::string &source_frame, std::string &target_frame, 
                            geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &transformed_pose);
        void saveDebugImage(const cv_bridge::CvImagePtr &cv_image_ptr, const sensor_msgs::ImageConstPtr &raw_image, std::string logdir="/tmp/");
    
    private:
        boost::shared_ptr<tf::TransformListener> tf_listener_;


};
#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_H
