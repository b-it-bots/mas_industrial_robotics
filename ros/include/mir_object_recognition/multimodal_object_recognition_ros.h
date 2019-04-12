/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */
#ifndef MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H
#define MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/server.h>
#include <mir_object_recognition/SceneSegmentationConfig.h>

#include <mir_object_recognition/pointcloud_segmentation_ros.h>
#include <mir_object_recognition/image_recognition_ros.h>
#include <mcr_perception_msgs/GetSegmentedImage.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <mcr_scene_segmentation/clustered_point_cloud_visualizer.h>
#include <mcr_scene_segmentation/bounding_box_visualizer.h>
#include <mcr_scene_segmentation/label_visualizer.h>

//#include <mir_object_recognition/clustered_point_cloud_visualizer.h>
//#include <mir_object_recognition/bounding_box_visualizer.h>
//#include <mir_object_recognition/label_visualizer.h>

//#include <mas_perception_libs/bounding_box.h>

// using mcr::visualization::BoundingBoxVisualizer;
// using mcr::visualization::ClusteredPointCloudVisualizer;
// using mcr::visualization::LabelVisualizer;
// using mcr::visualization::Color;

/**
 * This node subscribes to ...
 * Inputs:
 * ~event_in:
 * Outputs:
 * ~event_out:
**/

class MultimodalObjectRecognitionROS
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
        MultimodalObjectRecognitionROS(ros::NodeHandle nh,
                                        mcr::visualization::BoundingBoxVisualizer bounding_box_visualizer,
                                        mcr::visualization::ClusteredPointCloudVisualizer cluster_visualizer_,
                                        mcr::visualization::LabelVisualizer label_visualizer_);
        virtual ~MultimodalObjectRecognitionROS();

    private:
        ros::Subscriber sub_cloud_;
        ros::Subscriber sub_event_in_;
        ros::Publisher pub_event_out_;

        tf::TransformListener transform_listener_;

        dynamic_reconfigure::Server<mir_object_recognition::SceneSegmentationConfig> server_;

        // Publisher for clouds and images recognizer
        ros::Publisher pub_cloud_to_recognizer_;
        ros::Publisher pub_image_to_recognizer_;
        // Subscriber for clouds and images recognizer
        ros::Subscriber sub_recognized_image_list_;
        ros::Subscriber sub_recognized_cloud_list_;
        // Publisher object list
        ros::Publisher pub_object_list_;
        ros::Publisher pub_workspace_height_;
        // Publisher pose array (debug_mode only)
        ros::Publisher pub_pcl_object_pose_array_;
        ros::Publisher pub_rgb_object_pose_array_;

        // Recognize Clouds and Image callback
        void recognizedImageCallback(const mcr_perception_msgs::ObjectList &msg);
        void recognizedCloudCallback(const mcr_perception_msgs::ObjectList &msg);
        
    
    protected:
        ros::NodeHandle nh_;
        std::string target_frame_id_;
        //boost::shared_ptr<SceneSegmentationROS> pointcloud_recognition_interface_; //! Linked interface.
        //typedef SceneSegmentationROS PointcloudRecognition;
        
        typedef std::auto_ptr<PointcloudSegmentationROS> PointcloudSegmentationUPtr;
        PointcloudSegmentationUPtr pointcloud_segmentation_;
        typedef std::auto_ptr<ImageRecognitionROS> ImageRecognitionUPtr;
        ImageRecognitionUPtr image_recognition_;

        sensor_msgs::PointCloud2::Ptr pointcloud_msg_;

        // Flag for pointcloud subscription
        bool pointcloud_msg_received_;
        int pointcloud_msg_received_count_;
        int rgb_object_id_;
        
        // Flags for object recognition
        bool received_recognized_image_list_flag_;
        bool received_recognized_cloud_list_flag_;

        //Recognized image list
        mcr_perception_msgs::ObjectList recognized_image_list_;
        mcr_perception_msgs::ObjectList recognized_cloud_list_;

        // Visualization
        mcr::visualization::BoundingBoxVisualizer bounding_box_visualizer_;
        mcr::visualization::ClusteredPointCloudVisualizer cluster_visualizer_;
        mcr::visualization::LabelVisualizer label_visualizer_;

        mcr::visualization::ClusteredPointCloudVisualizer cluster_visualizer_pcl_;

        // Parameters
        bool debug_mode_;

    private:
        //void setConfig();
        void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg);
        void eventCallback(const std_msgs::String::ConstPtr &msg);
        void configCallback(mir_object_recognition::SceneSegmentationConfig &config, uint32_t level);

        void segmentCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                            mcr_perception_msgs::ObjectList &object_list,
                            std::vector<PointCloud::Ptr> &clusters);

        void recognizeCloudAndImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
                                 const sensor_msgs::ImageConstPtr &msg);

        void recognizeImage(const sensor_msgs::ImageConstPtr &msg);
        void publishImagesForRecognition(const sensor_msgs::ImageConstPtr &msg);

        void preprocessCloud();

        void get3DObject(const sensor_msgs::RegionOfInterest &roi, 
                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ordered_cloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_object);

        double getMatch(const cv::Rect2d& r1, const cv::Rect2d& r2);

        void MultimodalObjectRecognitionROS::combineObjectList(const mcr_perception_msgs::ObjectList& cloud_list, 
                                                       const mcr_perception_msgs::ObjectList& image_list,
                                                       mcr_perception_msgs::ObjectList& final_list);
        
        geometry_msgs::PoseStamped estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyz_input_cloud);

    public:
        void update();
        // These should be handled in algorithm_provider_impl
        //void get3DBoundingBox();
        //void get3DPose();

    // Move this to image_recognition_ros_utils
    private:
        void segmentObjects(const sensor_msgs::ImageConstPtr &image, 
                            cv::Mat &segmented_objects_mask);

        void getCroppedImages(const cv::Mat &input_image,
                            const cv::Mat &input_segmented_objects_image,
                            cv::Mat &output_debug_image,
                            std::vector<cv::Mat> &output_cropped_object_images,
                            std::vector<float> &output_object_areas,
                            std::vector<std::vector<cv::Point> > &output_roi_points,
                            std::vector<cv::Point> &output_center_of_objects);
        void pointsOnRectangle(cv::Rect rect, std::vector<cv::Point> &output_points);
        /**
         * Object segmentation
         */
        ros::ServiceClient segmentation_service_;
        std::string segmentation_service_name_;
        cv::RNG rng;
                
};

#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H
