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

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/server.h>
#include <mir_object_recognition/SceneSegmentationConfig.h>

#include <mir_object_recognition/pointcloud_segmentation_ros.h>
#include <mas_perception_msgs/GetSegmentedImage.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <mir_object_segmentation/clustered_point_cloud_visualizer.h>
#include <mir_object_segmentation/bounding_box_visualizer.h>
#include <mir_object_segmentation/label_visualizer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mir_object_recognition/multimodal_object_recognition_utils.h>

/**
 * \brief 
 * This node subscribes to pointcloud and image_raw topics
 * Inputs:
 * ~event_in:
 *            - e_start:    - starts subscribing to pointcloud and image topics simultaneously,
 *                          - segments pointcloud, recognize the table top clusters, estimate pose and workspace height
 *                          - detects rgb object, find 3D ROI, estimate pose
 *                          - adjusts object pose and publish them
 *            - e_stop:     - stop subscribing to the input topics and clear accumulated pointcloud
 * Outputs:
 * ~event_out:
 *            - e_done:     - done recognizing pointcloud and image, done pose estimation and done publishing object_list
 *            - e_stopped:  - done unsubscribing, done clearing accumulated point clouds
**/

class MultimodalObjectRecognitionROS
{
    public:
        MultimodalObjectRecognitionROS(ros::NodeHandle nh);
        virtual ~MultimodalObjectRecognitionROS();
        void update();

    private:
        ros::Subscriber sub_event_in_;
        ros::Publisher pub_event_out_;
        ros::Subscriber sub_cloud_;

        boost::shared_ptr<tf::TransformListener> tf_listener_;
        
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

        ros::Publisher pub_pcl_debug_in_;
        ros::Publisher pub_pcl_debug_out_;

        // Synchronize callback for image and pointcloud
        message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> msgSyncPolicy;
        message_filters::Synchronizer<msgSyncPolicy> *msg_sync_;
        void synchronizeCallback(const sensor_msgs::ImageConstPtr &image, 
                                 const sensor_msgs::PointCloud2ConstPtr &cloud);

        // Recognize Clouds and Image callback
        void recognizedImageCallback(const mas_perception_msgs::ObjectList &msg);
        void recognizedCloudCallback(const mas_perception_msgs::ObjectList &msg);
    
    protected:
        ros::NodeHandle nh_;
        std::string target_frame_id_;

        // Create unique_ptr for pointcloud_segmentation and mm_utils        
        typedef std::unique_ptr<PointcloudSegmentationROS> PointcloudSegmentationUPtr;
        PointcloudSegmentationUPtr pointcloud_segmentation_;
        typedef std::unique_ptr<MultimodalObjectRecognitionUtils> MultimodalObjectRecognitionUtilsUPtr;
        MultimodalObjectRecognitionUtilsUPtr mm_object_recognition_utils_;

        // Used to store pointcloud and image received from callback
        sensor_msgs::PointCloud2ConstPtr pointcloud_msg_;
        sensor_msgs::ImageConstPtr image_msg_;
        PointCloud::Ptr cloud_;

        // Flags for pointcloud and image subscription
        int pointcloud_msg_received_count_;
        int image_msg_received_count_;
        
        // rgb_object_id used to differentiate 2D and 3D objects
        int rgb_object_id_;
        
        // Flags for object recognition
        bool received_recognized_image_list_flag_;
        bool received_recognized_cloud_list_flag_;

        //Recognized image list
        mas_perception_msgs::ObjectList recognized_image_list_;
        mas_perception_msgs::ObjectList recognized_cloud_list_;

        // Visualization
        mir::visualization::BoundingBoxVisualizer bounding_box_visualizer_pcl_;
        mir::visualization::ClusteredPointCloudVisualizer cluster_visualizer_rgb_;
        mir::visualization::ClusteredPointCloudVisualizer cluster_visualizer_pcl_;
        mir::visualization::LabelVisualizer label_visualizer_rgb_;
        mir::visualization::LabelVisualizer label_visualizer_pcl_;

        // Parameters
        bool debug_mode_;

        // Dynamic parameter
        double pcl_object_height_above_workspace_;
        double rgb_object_height_above_workspace_;
        double rgb_container_height_;
        int rgb_bbox_size_adjustment_;
        int rgb_bbox_min_diag_;
        int rgb_bbox_max_diag_;
        int rgb_cluster_filter_limit_min_;
        int rgb_cluster_filter_limit_max_;
        double rgb_base_link_to_laser_distance_;
        double rgb_max_object_pose_x_to_base_link_;
        double rgb_min_bbox_z_;

         // logdir for saving debug image
        std::string logdir_;
        bool data_collection_;

    private:
        void eventCallback(const std_msgs::String::ConstPtr &msg);
        void configCallback(mir_object_recognition::SceneSegmentationConfig &config, uint32_t level);
        
        /** \brief Transform pointcloud to the given frame id ("base_link" by default)*/
        void transformCloud();

        /** \brief Add cloud accumulation, segment accumulated pointcloud, find the plane, 
         *         clusters table top objects, find object heights.
         * \param[out] 3D object list with unknown label
         * \param[out] Table top pointcloud clusters
        */
        void segmentPointcloud(mas_perception_msgs::ObjectList &object_list, 
                               std::vector<PointCloud::Ptr> &clusters);

        /** \brief Recognize 2D and 3D objects, estimate their pose, filter them, and publish the object_list*/
        void recognizeCloudAndImage();

        /** \brief Get 3D objects given 2D ROI 
         * \param[in] Region of interest (bounding box) of 2D object
         * \param[in] Ordered and transformed pointcloud
         * \param[out] 3D pointcloud cluster (3D ROI) of the given 2D ROI 
        */
        void get3DObject(const sensor_msgs::RegionOfInterest &roi, 
                         const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ordered_cloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_object);
        
        /** \brief Adjust object pose, make it flat, adjust container, axis and bolt poses.
         * \param[in] Object_list.pose, .name,
         * 
        */
        void adjustObjectPose(mas_perception_msgs::ObjectList &object_list);

        /** \brief Publish object_list to object_list merger 
         * \param[in] Object list to publish
        */
        void publishObjectList(mas_perception_msgs::ObjectList &object_list);

        /** \brief Publish debug info such as bbox, poses, labels for both 2D and 3D objects.
         * \param[in] combined object list
         * \param[in] 3D pointcloud cluster from 3D object segmentation
         * \param[in] 3D pointcloud cluster from 2D bounding box proposal
        */
        void publishDebug(mas_perception_msgs::ObjectList &combined_object_list,
                          std::vector<PointCloud::Ptr> &clusters_3d,
                          std::vector<PointCloud::Ptr> &clusters_2d);
                
};

#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H
