/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */
#ifndef MIR_OBJECT_RECOGNITION_IMAGE_RECOGNITION_ROS_H
#define MIR_OBJECT_RECOGNITION_IMAGE_RECOGNITION_ROS_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

/**
 * This node subscribes to ...
 * Inputs:
 * ~event_in:
 * Outputs:
 * ~event_out:
**/

class ImageRecognitionROS
{
    public:
        struct Config
        {
            public:
                Config();
            public:
                int config_test;
        };

    public:
        ImageRecognitionROS(ros::NodeHandle nh_sub);
        virtual ~ImageRecognitionROS();

    protected:
        ros::NodeHandle nh_;

    private:
        //void imageCallback(const sensor_msgs::PointCloud2::Ptr &msg);
        //void eventCallback(const std_msgs::String::ConstPtr &msg);
        //void configCallback(mcr_scene_segmentation::SceneSegmentationConfig &config, uint32_t level);
        void recognizeImage(const sensor_msgs::ImageConstPtr &msg);
        void setConfig();
        void convertCloudToXYZImage(cv::Mat &output_xyz_image);

    public:
        // void ImageRecognitionROS::extractObjectsInRGBImage(const cv::Mat &input_image,
        //                 const cv::Mat &input_segmented_objects_image,
        //                 cv::Mat &output_debug_image,
        //                 std::vector<cv::Mat> &output_cropped_object_images,
        //                 std::vector<float> &output_object_areas,
        //                 std::vector<std::vector<cv::Point> > &output_roi_points,
        //                 std::vector<cv::Point> &output_center_of_objects);
        //TODO:
        //1. RGB region proposal to cloud
        //2. Cloud to 3Dbbox
        //3 .3Dbbox to pose
};

#endif  // MIR_OBJECT_RECOGNITION_IMAGE_RECOGNITION_ROS_H
