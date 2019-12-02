/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <pcl/common/transforms.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/extract_clusters.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <mir_object_recognition/multimodal_object_recognition_ros.h>
#include <mas_perception_msgs/ObjectList.h>
#include <mas_perception_msgs/ImageList.h>
#include <mas_perception_msgs/BoundingBoxList.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/PoseArray.h>
#include <mir_object_recognition/SceneSegmentationConfig.h>
#include <mas_perception_libs/bounding_box.h>
#include <mas_perception_libs/point_cloud_utils.h>
#include <mas_perception_libs/sac_plane_segmenter.h>

using mcr::visualization::BoundingBoxVisualizer;
using mcr::visualization::ClusteredPointCloudVisualizer;
using mcr::visualization::LabelVisualizer;
using mas_perception_libs::Color;
using mas_perception_libs::CloudFilterParams;
using mas_perception_libs::SacPlaneSegmenterParams;

MultimodalObjectRecognitionROS::MultimodalObjectRecognitionROS(ros::NodeHandle nh):
    nh_(nh),
    pointcloud_msg_received_(false),
    pointcloud_msg_received_count_(0),
    image_msg_received_count_(0),
    received_recognized_cloud_list_flag_(false),
    received_recognized_image_list_flag_(false),
    rgb_object_id_(100),
    rgb_container_height_(0.05),
    rgb_bbox_size_adjustment_(2),
    rgb_bbox_min_diag_(21),
    rgb_bbox_max_diag_(250),
    rgb_min_bbox_z_(0.03),
    bounding_box_visualizer_pcl_("bounding_boxes", Color(Color::SALMON)),
    cluster_visualizer_rgb_("tabletop_cluster_rgb"),
    cluster_visualizer_pcl_("tabletop_cluster_pcl"),
    label_visualizer_rgb_("rgb_labels", Color(Color::SALMON)),
    label_visualizer_pcl_("pcl_labels", Color(Color::TEAL))
    
{
    pointcloud_segmentation_ = PointcloudSegmentationUPtr(new PointcloudSegmentationROS(nh_));
    image_recognition_ = ImageRecognitionUPtr(new ImageRecognitionROS(nh_));

    dynamic_reconfigure::Server<mir_object_recognition::SceneSegmentationConfig>::CallbackType f =
                            boost::bind(&MultimodalObjectRecognitionROS::configCallback, this, _1, _2);
    server_.setCallback(f);
    
    sub_event_in_ = nh_.subscribe("event_in", 1, &MultimodalObjectRecognitionROS::eventCallback, this);
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    
    // Publish cloud and images to cloud and rgb recognition topics
    pub_cloud_to_recognizer_  = nh_.advertise<mas_perception_msgs::ObjectList>(
                                "recognizer/pcl/input/object_list", 1);
    pub_image_to_recognizer_  = nh_.advertise<mas_perception_msgs::ImageList>(
                                "recognizer/rgb/input/images", 1);

    // Subscribe to cloud and rgb recognition topics
    sub_recognized_cloud_list_ = nh_.subscribe("recognizer/pcl/output/object_list", 1, 
                            &MultimodalObjectRecognitionROS::recognizedCloudCallback, this);
    sub_recognized_image_list_ = nh_.subscribe("recognizer/rgb/output/object_list", 1, 
                            &MultimodalObjectRecognitionROS::recognizedImageCallback, this);

    // Pub combined object_list to object_list merger
    pub_object_list_  = nh_.advertise<mas_perception_msgs::ObjectList>("output/object_list", 10);

    // Pub workspace height
    pub_workspace_height_ = nh_.advertise<std_msgs::Float64>("output/workspace_height", 1);

    //Segmentation topic
    nh_.param<std::string>("segmentation_service", segmentation_service_name_, "/mcr_perception/cnn_segmentation/cnn_object_segmentation");
    segmentation_service_ = nh_.serviceClient<mas_perception_msgs::GetSegmentedImage>(segmentation_service_name_);
    if (segmentation_service_.exists())
    {
        ROS_INFO_STREAM("Using segmentation "<< segmentation_service_name_);
    }
    else
    {
        ROS_WARN("Segmentation service is not available");
    }

    nh_.param<bool>("debug_mode", debug_mode_, false);
    // Pub pose array
    pub_pcl_object_pose_array_  = nh_.advertise<geometry_msgs::PoseArray>("output/pcl_object_pose_array", 10);
    pub_rgb_object_pose_array_  = nh_.advertise<geometry_msgs::PoseArray>("output/rgb_object_pose_array", 10);

    nh_.param<std::string>("target_frame_id", target_frame_id_, "base_link");
    //rgb_object_id_ = 100;
    
    // Pub pcl debug, used to compare pcl before and after filter
    pub_pcl_debug_in_ = nh_.advertise<sensor_msgs::PointCloud2>("output/rgb/debug/rgb_obj_cluster_in", 1);
    pub_pcl_debug_out_ = nh_.advertise<sensor_msgs::PointCloud2>("output/rgb/debug/rgb_obj_cluster_out", 1);

    nh_.param<std::string>("logdir", logdir_, "/tmp/");
    
}

MultimodalObjectRecognitionROS::~MultimodalObjectRecognitionROS()
{
}

void MultimodalObjectRecognitionROS::synchronizeCallback(const sensor_msgs::ImageConstPtr &image, 
                                            const sensor_msgs::PointCloud2ConstPtr &cloud)
{    
    if (pointcloud_msg_received_count_ < 2)
    {
        ROS_INFO("[multimodal_object_recognition_ros] Received 3 messages");
        /* ROS_INFO("[multimodal_object_recognition_ros] Received pointcloud message"); */
        //pointcloud_msg_ = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
        pointcloud_msg_ = cloud;
        pointcloud_msg_received_ = true;
        pointcloud_msg_received_count_ += 1;
        /* ROS_INFO("[multimodal_object_recognition_ros] Received rgb image message"); */
        //image_msg_ = sensor_msgs::Image::Ptr(new sensor_msgs::Image);
        image_msg_ = image;
        image_msg_received_ = true;
        image_msg_received_count_ += 1;
    }
}
void MultimodalObjectRecognitionROS::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    //if (!pointcloud_msg_received_ )
    if (pointcloud_msg_received_count_ < 3)
    {
        ROS_INFO("[multimodal_object_recognition_ros] Received pointcloud message");
        //pointcloud_msg_ = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
        pointcloud_msg_ = msg;
        pointcloud_msg_received_ = true;
        pointcloud_msg_received_count_ += 1;
    }
}

void MultimodalObjectRecognitionROS::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("Camera node is triggered");
    /* if (image_msg_received_count_ < 3) */
    /* //if (!image_msg_received_) */
    /* { */
    /*     ROS_INFO("[multimodal_object_recognition_ros] Received rgb image message"); */
    /*     //image_msg_ = sensor_msgs::Image::Ptr(new sensor_msgs::Image); */
    /*     image_msg_ = msg; */
    /*     image_msg_received_ = true; */
    /*     image_msg_received_count_ += 1; */
    /* } */
}

void MultimodalObjectRecognitionROS::recognizedCloudCallback(const mas_perception_msgs::ObjectList &msg)
{
    ROS_INFO("Received recognized cloud callback ");
    if (!received_recognized_cloud_list_flag_)
    {
        std::cout<<"Insde cloud cb"<<std::endl;
        std::cout<<"Insde cloud cb"<<received_recognized_cloud_list_flag_ <<std::endl;
        recognized_cloud_list_ = msg;
        received_recognized_cloud_list_flag_ = true;
    }
}

void MultimodalObjectRecognitionROS::recognizedImageCallback(const mas_perception_msgs::ObjectList &msg)
{
    ROS_INFO("Received recognized image callback ");
    if (!received_recognized_image_list_flag_)
    {
        recognized_image_list_ = msg;
        received_recognized_image_list_flag_ = true;
    }
}

void MultimodalObjectRecognitionROS::update()
{
    ROS_DEBUG_STREAM("update function called");
    if (pointcloud_msg_received_count_ > 1 && image_msg_received_count_ > 1)
    {
        ROS_DEBUG_STREAM("Received enough images and pointclouds");
        // Reset msg received flag
        pointcloud_msg_received_ = false;
        image_msg_received_ = false;
        pointcloud_msg_received_count_ = 0;
        image_msg_received_count_ = 0;
        // Shutdown subscriber
        //sub_cloud_.shutdown();
        sub_image_.shutdown();
        
        image_sub_->unsubscribe();
        cloud_sub_->unsubscribe();

        //Preprocess cloud
        double start_time = ros::Time::now().toSec();
        preprocessCloud();
        double end_time = ros::Time::now().toSec();
        std::cout<<"Time: "<<end_time - start_time<<std::endl;

        // pub e_done
        std_msgs::String event_out;
        event_out.data = "e_done";
        pub_event_out_.publish(event_out);

        // Reset received recognized cloud and image
        received_recognized_cloud_list_flag_ = false;
        received_recognized_image_list_flag_ = false;

    }
}

void MultimodalObjectRecognitionROS::preprocessCloud()
{
    std::string target_frame_id;
    nh_.param<std::string>("target_frame_id", target_frame_id, "base_link");
    sensor_msgs::PointCloud2 msg_transformed;
    msg_transformed.header.frame_id = target_frame_id;
    try
    {
        ros::Time common_time;
        transform_listener_.getLatestCommonTime(target_frame_id, pointcloud_msg_->header.frame_id, common_time, NULL);
        pointcloud_msg_->header.stamp = common_time;
        transform_listener_.waitForTransform(target_frame_id, pointcloud_msg_->header.frame_id,
                                                ros::Time::now(), ros::Duration(1.0));
        pcl_ros::transformPointCloud(target_frame_id, *pointcloud_msg_, msg_transformed, transform_listener_);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("PCL transform error: %s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    // Set frame id 
    pointcloud_segmentation_->frame_id_ = target_frame_id;
    // Get Image from ordered cloud
    pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msg_transformed, *pc2);
    pc2->header.frame_id = msg_transformed.header.frame_id;
    // pcl::PCLImage pcl_image;
    // pcl::toPCLPointCloud2(*pc2, pcl_image);
    // sensor_msgs::ImagePtr image_msg = boost::make_shared<sensor_msgs::Image>();
    // pcl_conversions::moveFromPCL(pcl_image, *image_msg);
    
    // Convert to pcl::Pointcloud for segmentation
    /* PointCloud::Ptr cloud(new PointCloud); */
    cloud_ = PointCloud::Ptr(new PointCloud); 
    pcl::fromPCLPointCloud2(*pc2, *cloud_);
    
    //recognize Cloud and image
    /* recognizeCloudAndImage(cloud, image_msg_); */
    recognizeCloudAndImage();
    // Reset cloud accumulation
    pointcloud_segmentation_->reset_cloud_accumulation();

    // Recognized image and cloud list
    if (recognized_image_list_.objects.size() > 0)
    {
        recognized_image_list_.objects.clear();
    }
    if (recognized_cloud_list_.objects.size() > 0)
    {
       recognized_cloud_list_.objects.clear(); 
    }
}

void MultimodalObjectRecognitionROS::recognizeCloudAndImage()
{
    mas_perception_msgs::ObjectList cloud_object_list;
    std::vector<PointCloud::Ptr> clusters_3d;
    
    pointcloud_segmentation_->add_cloud_accumulation(cloud_);
    pointcloud_segmentation_->segment_cloud(cloud_object_list, clusters_3d);

    std_msgs::Float64 workspace_height_msg;
    workspace_height_msg.data = pointcloud_segmentation_->getWorkspaceHeight();
    pub_workspace_height_.publish(workspace_height_msg);

    // Compute normal to generate parallel BBOX with the plane
    const Eigen::Vector3f normal = pointcloud_segmentation_->getPlaneNormal();

    //Publish cluster for recognition
    if (!cloud_object_list.objects.empty())
    {
        ROS_INFO_STREAM("Publishing clouds for recognition");
        ROS_INFO_STREAM("Using ... for recognition");
        pub_cloud_to_recognizer_.publish(cloud_object_list);
    }
    
    // Pub Image to recognizer
    mas_perception_msgs::ImageList image_list;
    image_list.images.resize(1);
    image_list.images[0] = *image_msg_;
    if (!image_list.images.empty())
    {
        ROS_INFO_STREAM("Publishing images for recognition");
        ROS_INFO_STREAM("Using ... for recognition");
        pub_image_to_recognizer_.publish(image_list);
    }
    ROS_INFO("Waiting for message from Cloud and Imahe recognizer");
    // TODO: HANDLE MALLOC ERROR WHEN SPINNING
    //loop till you received the message from the 3d and rgb recognition
    // TODO: Wait for recognize image to come
    // Why wait for image and not cloud recognition?
    // It's because rgb is slower to recognize
    int loop_rate_hz = 30;
    int timeout_wait = 2; //secs
    ros::Rate loop_rate(loop_rate_hz);
    int loop_rate_count = 0;
    if (cloud_object_list.objects.size() > 0)
    {
        while(!received_recognized_cloud_list_flag_)
        {
            ROS_INFO_STREAM("["<<loop_rate_count<<"] [PCL] Waiting message from PCL recognizer node");
            loop_rate_count += 1;
            ros::spinOnce();
            loop_rate.sleep();
            if (received_recognized_cloud_list_flag_ == true)
            {
                ROS_INFO_STREAM("[PCL] Received object_list from pcl recognizer");
                ROS_INFO_STREAM("[PCL] Received objects: "<<recognized_cloud_list_.objects.size());
            }
            if (loop_rate_count > loop_rate_hz * timeout_wait)
            {
                received_recognized_cloud_list_flag_ = false;
                ROS_ERROR("[PCL] No message received from PCL recognizer. ");
                break;
            }
        }
    }

    // Merge recognized_cloud_list and final_image_list
    mas_perception_msgs::ObjectList combined_object_list;
    if (!recognized_cloud_list_.objects.empty())
    {
        combined_object_list.objects.insert(combined_object_list.objects.end(), 
                                        recognized_cloud_list_.objects.begin(),
                                        recognized_cloud_list_.objects.end());
    }

    loop_rate_count = 0;
    timeout_wait = 3; //secs
    if (image_list.images.size() > 0)
    {
        while(!received_recognized_image_list_flag_)
        {
            ROS_INFO_STREAM("["<<loop_rate_count<<"] [RGB] Waiting message from RGB recognizer node");
            loop_rate_count += 1;
            ros::spinOnce();
            loop_rate.sleep();
            if (received_recognized_image_list_flag_ == true)
            {
                ROS_INFO_STREAM("[RGB] Received object_list from rgb recognizer");
                ROS_INFO_STREAM("[RGB] Received objects: "<<recognized_image_list_.objects.size());
            }
            if (loop_rate_count > loop_rate_hz * timeout_wait)
            {
                received_recognized_image_list_flag_ = false;
                ROS_ERROR("[RGB] No message received from RGB recognizer. ");
                break;
            }
        }
    }
    // Reset recognition callback flags
    received_recognized_cloud_list_flag_ = false;
    received_recognized_image_list_flag_ = false;

    mas_perception_msgs::ObjectList final_image_list;
    mas_perception_msgs::BoundingBoxList bounding_boxes;
    std::vector<PointCloud::Ptr> clusters_2d;
    
    bool done_recognizing_image = false;

    cv_bridge::CvImagePtr cv_image;
    if (recognized_image_list_.objects.size() > 0)
    {
        try
        {
            cv_image = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        bounding_boxes.bounding_boxes.resize(recognized_image_list_.objects.size());
        final_image_list.objects.resize(recognized_image_list_.objects.size());

        for (int i=0; i<recognized_image_list_.objects.size();i++)
        {
            mas_perception_msgs::Object object = recognized_image_list_.objects[i];
            //Get ROI
            sensor_msgs::RegionOfInterest roi_2d = object.roi;
            const cv::Rect2d rect2d(roi_2d.x_offset, roi_2d.y_offset, roi_2d.width, roi_2d.height);

            if (debug_mode_)
            {
                std::cout<<"Region of Interest: cx, cy, width, height"<<std::endl;
                std::cout<<roi_2d.x_offset<<", "<<roi_2d.y_offset<<", "<<roi_2d.width<<", "<<roi_2d.height<<std::endl;
                
                cv::Point pt1;
                cv::Point pt2;

                pt1.x = roi_2d.x_offset;
                pt1.y = roi_2d.y_offset;
                pt2.x = roi_2d.x_offset + roi_2d.width;
                pt2.y = roi_2d.y_offset + roi_2d.height;

                // draw bbox
                cv::rectangle(cv_image->image, pt1, pt2, cv::Scalar(0,255,0), 1, 8, 0);
                // add label
                cv::putText(cv_image->image, object.name, cv::Point(pt1.x, pt2.y), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,255,0), 1);
            }
            // TODO: Create Get pose in _utils.cpp
            // NOTE: Remove large 2d misdetected bbox (misdetection)
            // Solution find diagional, and make a threshold
            double len_diag = sqrt(powf(((roi_2d.width + roi_2d.width) >> 1), 2));
            if (len_diag > rgb_bbox_min_diag_ && len_diag < rgb_bbox_max_diag_)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_object_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                get3DObject(roi_2d, cloud_, pcl_object_cluster);
                pcl_object_cluster->header.frame_id = cloud_->header.frame_id;

                // Filter big objects from 3d proposal, if the height is less than 3 mm
                pcl::PointXYZRGB min_pt;
                pcl::PointXYZRGB max_pt;
                pcl::getMinMax3D(*pcl_object_cluster, min_pt, max_pt);

                std::cout<<"obj name: "<<recognized_image_list_.objects[i].name<<std::endl;
                std::cout<<"min z: "<<min_pt.z<<std::endl;
                std::cout<<"max z: "<<max_pt.z<<std::endl;

                float obj_height = max_pt.z - pointcloud_segmentation_->getWorkspaceHeight();
                std::cout<<"obj height: "<<obj_height<<std::endl;
                
                sensor_msgs::PointCloud2 ros_pc2;
                pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
                pcl::toPCLPointCloud2(*pcl_object_cluster, *pc2);
                pcl_conversions::fromPCL(*pc2, ros_pc2);
                ros_pc2.header.frame_id = target_frame_id_;
                ros_pc2.header.stamp = ros::Time::now();
                final_image_list.objects[i].pointcloud = ros_pc2;

                clusters_2d.push_back(pcl_object_cluster);
                // Get pose
                geometry_msgs::PoseStamped pose = estimatePose(pcl_object_cluster, recognized_image_list_.objects[i].name);
                // Transform pose
                std::string frame_id = cloud_->header.frame_id;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = frame_id;
                std::string target_frame_id;
                geometry_msgs::PoseStamped pose_transformed;
                if (nh_.hasParam("target_frame_id"))
                {
                    nh_.param("target_frame_id", target_frame_id, frame_id);
                    if (target_frame_id != frame_id)
                    {
                        try
                        {
                            ros::Time common_time;
                            transform_listener_.getLatestCommonTime(frame_id, target_frame_id, common_time, NULL);
                            pose.header.stamp = common_time;
                            transform_listener_.waitForTransform(target_frame_id, frame_id, common_time, ros::Duration(0.1));
                            transform_listener_.transformPose(target_frame_id, pose, pose_transformed);
                            final_image_list.objects[i].pose = pose_transformed;
                        }
                        catch(tf::LookupException& ex)
                        {
                            ROS_WARN("Failed to transform pose: (%s)", ex.what());
                            pose.header.stamp = ros::Time::now();
                            final_image_list.objects[i].pose = pose;
                        }
                    }
                    else
                    {
                        final_image_list.objects[i].pose = pose;
                    }
                }             
                else
                {
                    final_image_list.objects[i].pose = pose;
                }
                final_image_list.objects[i].probability = recognized_image_list_.objects[i].probability;
                final_image_list.objects[i].database_id = rgb_object_id_;
                final_image_list.objects[i].name = recognized_image_list_.objects[i].name;
            }
            else
            {
                std::cout<<"[RGB] DECOY"<<std::endl;
                final_image_list.objects[i].name = "DECOY";
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = cloud_->header.frame_id;
                pose.pose.position.x = 10.0;
                pose.pose.position.y = 10.0;
                pose.pose.position.z = 10.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                final_image_list.objects[i].pose = pose;
                final_image_list.objects[i].probability = 0.0;
                final_image_list.objects[i].database_id = rgb_object_id_;
            }
            rgb_object_id_++;
        }
        combined_object_list.objects.insert(combined_object_list.objects.end(), 
                                        final_image_list.objects.begin(),
                                        final_image_list.objects.end());
    }

    if (!combined_object_list.objects.empty())
    {
        for (int i=0; i<combined_object_list.objects.size(); i++)
        {
            double current_object_x_pose = combined_object_list.objects[i].pose.pose.position.x;
            if (current_object_x_pose < rgb_base_link_to_laser_distance_ ||
                current_object_x_pose > rgb_max_object_pose_x_to_base_link_)
                /* combined_object_list.objects[i].pose.pose.position.z < pointcloud_segmentation_ */
                /* ->object_height_above_workspace_ - 0.05) */
            {
                std::cout<<"#########Filtering: "<<combined_object_list.objects[i].name<<std::endl;
                std::cout<<"base link to laser: "<<rgb_base_link_to_laser_distance_<<std::endl;
                std::cout<<"rgb max object pose to baselink: "<<rgb_max_object_pose_x_to_base_link_<<std::endl;
                std::cout<<"Pose:  "<<current_object_x_pose<<std::endl;
                
                combined_object_list.objects[i].name = "DECOY";
                
            }
            std::cout<<"[RGB] Object: "<<combined_object_list.objects[i].name<<std::endl;
            std::cout<<"Pose:  "<<current_object_x_pose<<std::endl;
            
            std::cout<<"Plane height: "<<pointcloud_segmentation_->object_height_above_workspace_<<std::endl;
            std::cout<<"Object height: "<<combined_object_list.objects[i].pose.pose.position.z<<std::endl;
        }
        // Adjust RPY and update container pose
        adjustObjectPose(combined_object_list);
        // update AXIS, BOLT
        updateObjectPose(combined_object_list);
        publishObjectList(combined_object_list);
    }
    else
    {
        ROS_WARN("No objects to publish");
        return;
    }
    
    if (debug_mode_)
    {
        ROS_INFO_STREAM("Debug mode: publishing object information");
        std::cout<<"Cloud list: "<< recognized_cloud_list_.objects.size()<<std::endl;
        std::cout<<"Image list: "<< recognized_image_list_.objects.size()<<std::endl;

        ROS_INFO_STREAM("Combined object list: "<< combined_object_list.objects.size());
        // Bounding boxes
        if (recognized_cloud_list_.objects.size() > 0)
        {
            if (clusters_3d.size() > 0)
            {
                mas_perception_msgs::BoundingBoxList bounding_boxes;
                cluster_visualizer_pcl_.publish<PointT>(clusters_3d, target_frame_id_);
                bounding_boxes.bounding_boxes.resize(clusters_3d.size());
                for (int i=0; i<clusters_3d.size(); i++)
                {
                    mas_perception_libs::BoundingBox bbox;
                    pointcloud_segmentation_->get3DBoundingBox(clusters_3d[i], normal, bbox, bounding_boxes.bounding_boxes[i]);
                }
                if (bounding_boxes.bounding_boxes.size() > 0)
                {
                    bounding_box_visualizer_pcl_.publish(bounding_boxes.bounding_boxes, target_frame_id_);
                }
            }
            // PCL Pose array for debug mode only
            geometry_msgs::PoseArray pcl_object_pose_array;
            pcl_object_pose_array.header.frame_id = target_frame_id_;
            pcl_object_pose_array.header.stamp = ros::Time::now();
            pcl_object_pose_array.poses.resize(recognized_cloud_list_.objects.size());
            std::vector<std::string> pcl_labels;
            int pcl_count = 0;
            for (int i=0; i<combined_object_list.objects.size(); i++)
            {
                if (combined_object_list.objects[i].database_id < 99)
                {
                    std::cout<<"PCL Class: "<<combined_object_list.objects[i].name<<std::endl;
                    pcl_object_pose_array.poses[pcl_count] = combined_object_list.objects[i].pose.pose;
                    pcl_labels.push_back(combined_object_list.objects[i].name);
                    pcl_count ++;
                }
            }
            if (pcl_object_pose_array.poses.size() > 0)
            {
                pub_pcl_object_pose_array_.publish(pcl_object_pose_array);
            }
            if ((pcl_labels.size() == pcl_object_pose_array.poses.size()) &&
                    (pcl_labels.size() > 0) && (pcl_object_pose_array.poses.size() > 0))
            {
                label_visualizer_pcl_.publish(pcl_labels, pcl_object_pose_array);
            }
        }
        if (clusters_2d.size() > 0)
        {
            std::cout<<"RGB FINAL: "<<final_image_list.objects.size()<<std::endl;
            std::cout<<"OBJECT COMBINED: "<<recognized_image_list_.objects.size()<<std::endl;
            cluster_visualizer_rgb_.publish<PointT>(clusters_2d, target_frame_id_);
            // RGB Pose array for debug mode only
            geometry_msgs::PoseArray rgb_object_pose_array;
            rgb_object_pose_array.header.frame_id = target_frame_id_;
            rgb_object_pose_array.header.stamp = ros::Time::now();
            rgb_object_pose_array.poses.resize(final_image_list.objects.size());
            std::vector<std::string> rgb_labels;
            int rgb_count = 0;
            for (int i=0; i<combined_object_list.objects.size(); i++)
            {
                if (combined_object_list.objects[i].database_id > 99)
                {
                    std::cout<<"[RGB] Class: "<<combined_object_list.objects[i].name<<std::endl;
                    rgb_object_pose_array.poses[rgb_count] = combined_object_list.objects[i].pose.pose;
                    rgb_labels.push_back(combined_object_list.objects[i].name);
                    std::cout<<"[RGB] Pose: "<<combined_object_list.objects[i].pose.pose.position.x<<", "<<combined_object_list.objects[i].pose.pose.position.y<<", "<<combined_object_list.objects[i].pose.pose.position.z<<std::endl;
                    rgb_count ++;
                }
            }

            if (rgb_object_pose_array.poses.size() > 0)
            {
                std::cout<<"[RGB] pose array size: "<<rgb_object_pose_array.poses.size()<<std::endl;
                pub_rgb_object_pose_array_.publish(rgb_object_pose_array);
            }
            if ((rgb_labels.size() == rgb_object_pose_array.poses.size()) && 
                    (rgb_labels.size() > 0) && (rgb_object_pose_array.poses.size() > 0))
            {
                label_visualizer_rgb_.publish(rgb_labels, rgb_object_pose_array);
            }
        }
        /* if (!image_list.images.empty()) */
        /* { */
        /*     saveDebugImage(cv_image); */
        /* } */
    }
}

void MultimodalObjectRecognitionROS::publishObjectList(mas_perception_msgs::ObjectList &object_list)
{
    for (int i=0; i<object_list.objects.size(); i++)
    {
        // Empty cloud
        sensor_msgs::PointCloud2 empty_ros_cloud;
        object_list.objects[i].pointcloud = empty_ros_cloud;
        // Rename container to match refbox naming
        if (object_list.objects[i].name == "BLUE_CONTAINER")
        {
            object_list.objects[i].name = "CONTAINER_BOX_BLUE";
        }
        else if (object_list.objects[i].name == "RED_CONTAINER")
        {
            object_list.objects[i].name = "CONTAINER_BOX_RED";
        }
    }
    // Publish object list to object list merger
    pub_object_list_.publish(object_list);
}

void MultimodalObjectRecognitionROS::updateObjectPose(mas_perception_msgs::ObjectList &combined_object_list)
{
    for (int i=0; i<combined_object_list.objects.size(); i++)
    {
        if (combined_object_list.objects[i].name == "M20_100" || combined_object_list.objects[i].name == "AXIS")
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(combined_object_list.objects[i].pointcloud, *xyz_cloud);
            int pcl_point_size = combined_object_list.objects[i].pointcloud.height * combined_object_list.objects[i].pointcloud.width;
            pcl::PointXYZ min_pt;
            pcl::PointXYZ max_pt;

            pcl::getMinMax3D(*xyz_cloud, min_pt, max_pt);
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_at_z(new pcl::PointCloud<pcl::PointXYZ>);

            Eigen::Vector4f centroid;
            for (size_t i=0; i<pcl_point_size; i++ )
            {
                if (xyz_cloud->points[i].z == max_pt.z)
                {
                    point_at_z->points.push_back(xyz_cloud->points[i]);
                }
            }
            unsigned int valid_points = pcl::compute3DCentroid(*point_at_z, centroid);
            if (combined_object_list.objects[i].name == "M20_100")
            {
                ROS_INFO_STREAM("Updating M20_100 pose");
                float midpoint_x = (combined_object_list.objects[i].pose.pose.position.x + centroid[0])/2;
                float midpoint_y = (combined_object_list.objects[i].pose.pose.position.y + centroid[1])/2;
                combined_object_list.objects[i].pose.pose.position.x = midpoint_x;
                combined_object_list.objects[i].pose.pose.position.y = midpoint_y;
            }
            else if (combined_object_list.objects[i].name == "AXIS")
            { 
                ROS_INFO_STREAM("Updating axis pose");
                combined_object_list.objects[i].pose.pose.position.x = centroid[0];
                combined_object_list.objects[i].pose.pose.position.y = centroid[1]; 
            }
        }
    }
}

void MultimodalObjectRecognitionROS::get3DObject(const sensor_msgs::RegionOfInterest &roi, 
                                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ordered_cloud,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_object)
{
    /* int pixel_loc_tolerance = 2; */
    int min_x = roi.x_offset;
    int min_y = roi.y_offset;
    int max_x = roi.x_offset + roi.width;
    int max_y = roi.y_offset + roi.height;
    // Add BBox tolerance
    if (roi.x_offset > rgb_bbox_size_adjustment_) min_x = min_x - rgb_bbox_size_adjustment_;
    if (roi.y_offset > rgb_bbox_size_adjustment_) min_y = min_y - rgb_bbox_size_adjustment_;
    if (roi.width+rgb_bbox_size_adjustment_ < ordered_cloud->width) min_x = min_x + rgb_bbox_size_adjustment_;
    if (roi.height+rgb_bbox_size_adjustment_ < ordered_cloud->height) min_y = min_y + rgb_bbox_size_adjustment_;
    
    std::vector<cv::Point> pixel_loc;

    for (int i=min_x; i<max_x; i++)
    {
        for (int j=min_y; j<max_y; j++)
        {
            cv::Point loc;
            loc.x =i;
            loc.y =j;
            pixel_loc.push_back(loc);
        }
    }
    for (size_t i=0; i<pixel_loc.size(); i++)
    {
        pcl::PointXYZRGB pcl_point = ordered_cloud->at(pixel_loc[i].x, pixel_loc[i].y);
        if ((!pcl_isnan(pcl_point.x)) && (!pcl_isnan(pcl_point.y)) && (!pcl_isnan(pcl_point.z)) &&
            (!pcl_isnan(pcl_point.r)) && (!pcl_isnan(pcl_point.g)) && (!pcl_isnan(pcl_point.b)))
        {
            pcl_object->points.push_back(pcl_point);
        }
    }
    pcl_object->header = ordered_cloud->header;
    if (pcl_object->points.size() > 0)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(pcl_object);
        sor.setMeanK(50);
        sor.setStddevMulThresh(3.0);
        sor.filter(*pcl_object);
    }
}

geometry_msgs::PoseStamped MultimodalObjectRecognitionROS::estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyz_input_cloud, std::string name)
{
    // If is not object name is not m20, m30, bearing, distance tube,
    // do passtrhough to filter
    // TODO: Pose zero after filter
    bool use_filter = true;
    pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
    // With filter, remove points belonging to plane for long objects
    if (name == "M30" || 
        name == "M20" ||
        name == "DISTANCE_TUBE" ||
        name == "BEARING") 
    {
        filtered_cloud = *xyz_input_cloud;
    }
    else
    {
        /**
        sensor_msgs::PointCloud2 ros_pc2_in;
        pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*xyz_input_cloud, *pc2);
        pcl_conversions::fromPCL(*pc2, ros_pc2_in);
        ros_pc2_in.header.frame_id = target_frame_id_;
        ros_pc2_in.header.stamp = ros::Time::now();
        pub_pcl_debug_in_.publish(ros_pc2_in);
        **/
        pcl::PassThrough<PointT> pass_through;
        pass_through.setFilterFieldName("z");
        pcl::PointXYZRGB min_pt;
        pcl::PointXYZRGB max_pt;
        pcl::getMinMax3D(*xyz_input_cloud, min_pt, max_pt);
        ROS_INFO_STREAM("Min and max z before filter: "<<min_pt.z<<", "<<max_pt.z);
        double limit_min = min_pt.z + 0.0060;
        double limit_max = max_pt.z;
        pass_through.setFilterLimits(limit_min, limit_max);
        pass_through.setInputCloud(xyz_input_cloud);
        pass_through.filter(filtered_cloud);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(filtered_cloud, centroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(filtered_cloud, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

    // swap largest and second largest eigenvector so that y-axis aligns with largest eigenvector and z with the second largest
    eigen_vectors.col(0).swap(eigen_vectors.col(2));
    eigen_vectors.col(1) = eigen_vectors.col(2).cross(eigen_vectors.col(0));

    Eigen::Matrix4f eigen_vector_transform(Eigen::Matrix4f::Identity());
    eigen_vector_transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
    eigen_vector_transform.block<3, 1>(0, 3) = -(eigen_vector_transform.block<3, 3>(0, 0) * centroid.head<3>());

    // transform cloud to eigenvector space
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::transformPointCloud(filtered_cloud, transformed_cloud, eigen_vector_transform);

    // find mean diagonal
    pcl::PointXYZRGB min_point, max_point;
    pcl::getMinMax3D(transformed_cloud, min_point, max_point);
    Eigen::Vector3f mean_diag = (max_point.getVector3fMap() + min_point.getVector3fMap()) / 2.0;

    // orientation and position of bounding box of cloud
    Eigen::Quaternionf orientation(eigen_vectors);
    Eigen::Vector3f position = eigen_vectors * mean_diag + centroid.head<3>();

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = position(0);
    pose_stamped.pose.position.y = position(1);
    pose_stamped.pose.position.z = position(2);
    /* pose_stamped.pose.position.z = pointcloud_segmentation_->getWorkspaceHeight(); */
    pose_stamped.pose.orientation.w = orientation.w();
    pose_stamped.pose.orientation.x = orientation.x();
    pose_stamped.pose.orientation.y = orientation.y();
    pose_stamped.pose.orientation.z = orientation.z();
    pose_stamped.header = pointcloud_msg_->header;

    tf::Quaternion temp;
    tf::quaternionMsgToTF(pose_stamped.pose.orientation, temp);
    tf::Matrix3x3 m(temp);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);

    return pose_stamped;
}

MultimodalObjectRecognitionROS::adjustObjectPose(mas_perception_msgs::ObjectList &object_list)
{
    for (int i=0; i<object_list.objects.size(); i++)
    {    
        tf::Quaternion q(
                object_list.objects[i].pose.pose.orientation.x,
                object_list.objects[i].pose.pose.orientation.y,
                object_list.objects[i].pose.pose.orientation.z,
                object_list.objects[i].pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double change_in_pitch = 0.0;
        if (object_list.objects[i].name == "M30" || 
                object_list.objects[i].name == "M20" ||
                object_list.objects[i].name == "DISTANCE_TUBE"  ||
                object_list.objects[i].name == "RED_CONTAINER"  ||
                object_list.objects[i].name == "BLUE_CONTAINER"  ||
                object_list.objects[i].name == "BEARING") 
        {
            yaw = 0.0;
        }

        if (object_list.objects[i].name == "RED_CONTAINER" || object_list.objects[i].name == "BLUE_CONTAINER")
        {
            object_list.objects[i].pose.pose.position.z = pointcloud_segmentation_->getWorkspaceHeight() + 0.05 ;
            /* change_in_pitch = -M_PI / 6.0; */
        }
        tf::Quaternion q2 = tf::createQuaternionFromRPY(0.0, change_in_pitch , yaw);
        object_list.objects[i].pose.pose.orientation.x = q2.x();
        object_list.objects[i].pose.pose.orientation.y = q2.y();
        object_list.objects[i].pose.pose.orientation.z = q2.z();
        object_list.objects[i].pose.pose.orientation.w = q2.w();

        if (pointcloud_segmentation_->getWorkspaceHeight() != -1000.0)
        {
            object_list.objects[i].pose.pose.position.z = pointcloud_segmentation_->getWorkspaceHeight() + 
                                                    pointcloud_segmentation_->object_height_above_workspace_;
        }
        if (object_list.objects[i].name == "RED_CONTAINER" || object_list.objects[i].name == "BLUE_CONTAINER")
        {
            object_list.objects[i].pose.pose.position.z = pointcloud_segmentation_->getWorkspaceHeight() + 0.08 ;
            if (object_list.objects[i].database_id > 100)
            {
                std::cout<<"Updating contained pose "<<std::endl;
                updateContainerPose(object_list.objects[i]);
                /* for (int j=0; j<object_list.objects.size(); j++) */
                /* { */
                /*     if (object_list.objects[i].name == object_list.objects[j].name) */
                /*     { */
                /*         object_list.objects[j].pose = object_list.objects[i].pose; */
                /*     } */
                /* } */
            }
        }
    }
}

void MultimodalObjectRecognitionROS::updateContainerPose(mas_perception_msgs::Object &container_object)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(container_object.pointcloud, *cloud);
    //find min and max z
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    ROS_INFO_STREAM("Min and max z "<<min_pt.z<<", "<<max_pt.z); 
    
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min_pt.z, (min_pt.z + (min_pt.z + max_pt.z/2))/2 );
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (300);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (200);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    pcl::PointIndices::Ptr filtered_cluster (new pcl::PointIndices() );
    int largest_index, largest_cluster;
    if (clusters.size() == 0)
    {
        return;
    }
    for(size_t i=0; i<clusters.size(); i++)
    {
        if(i == 0)
        {
            largest_index = 0;
            largest_cluster = clusters[0].indices.size();
        }
        else
        {
            if (largest_cluster <= clusters[i].indices.size())
            {
                largest_index = i;
                largest_cluster = clusters[i].indices.size();	
            }
        }
    }
    int counter = 0;
    while (counter < clusters[largest_index].indices.size ())
    {
        filtered_cluster->indices.push_back(clusters[largest_index].indices[counter]); 
        counter++;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    //point indices to cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> c_filter (true);
    c_filter.setInputCloud (cloud);
    c_filter.setIndices (filtered_cluster);
    c_filter.filter (*cloud_filtered); 
    
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    sensor_msgs::PointCloud2 ros_pointcloud;
    pcl::PCLPointCloud2::Ptr cloud_cl(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud_filtered, *cloud_cl);
    pcl_conversions::fromPCL(*cloud_cl, ros_pointcloud);
    ros_pointcloud.header.frame_id = target_frame_id_;
    
    //pub_pcl_cluster_.publish(ros_pointcloud);

    Eigen::Vector4f centroid;
    unsigned int valid_points = pcl::compute3DCentroid(*cloud_filtered, centroid);
    //std::cout<<"Valid: "<<valid_points<<std::endl;
    //std::cout<<"Centroid: "<<centroid<<std::endl;

    container_object.pose.pose.position.x = centroid[0];
    container_object.pose.pose.position.y = centroid[1];
    container_object.pose.pose.position.z = max_pt.z + rgb_container_height_;
}

void MultimodalObjectRecognitionROS::segmentObjects(const sensor_msgs::ImageConstPtr &image, 
                                                    cv::Mat &segmented_objects_mask)
{
    cv_bridge::CvImagePtr cv_image;
    try
    {
        cv_image = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    if (segmentation_service_.exists())
    {
        mas_perception_msgs::GetSegmentedImage srv;
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", 
                                       cv_image->image).toImageMsg();
        
        srv.request.image = *image_msg;
        if (segmentation_service_.call(srv))
        {
            cv_bridge::CvImagePtr cv_seg_image;
            cv_seg_image = cv_bridge::toCvCopy(srv.response.segmented_image, sensor_msgs::image_encodings::MONO8);
            segmented_objects_mask = cv_seg_image->image;
        }
    }

}

void MultimodalObjectRecognitionROS::getCroppedImages(const cv::Mat &input_image,
                        const cv::Mat &input_segmented_objects_image,
                        cv::Mat &output_debug_image,
                        std::vector<cv::Mat> &output_cropped_object_images,
                        std::vector<float> &output_object_areas,
                        std::vector<std::vector<cv::Point> > &output_roi_points,
                        std::vector<cv::Point> &output_center_of_objects)
{
    /// Find contours
    cv::Mat clone_mask = input_segmented_objects_image.clone();
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    //Get only the external contour with hierachry level 1
    findContours( clone_mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    std::vector<cv::Rect> roi_rectangle( contours.size() );
    std::vector<cv::RotatedRect> rotated_roi( contours.size() );

    for( int i = 0; i< contours.size(); i++ )
    {
        //Bounding box
        roi_rectangle[i] = boundingRect( cv::Mat(contours[i]) );

        // Find the minimum area enclosing bounding box
        rotated_roi[i] = minAreaRect( cv::Mat(contours[i]) );

        // expand rectangle a bit
        // (move top left by 5x5 pixels, and increase size by 10 x 10)
        roi_rectangle[i] -= cv::Point(15, 15);
        roi_rectangle[i] += cv::Size(30, 30);
        cv::Rect image_rect(0, 0, output_debug_image.cols, output_debug_image.rows);

        // check if roi is contained within image
        if (!((roi_rectangle[i] & image_rect) == roi_rectangle[i]))
        {
            if (roi_rectangle[i].x < 0)
            {
                roi_rectangle[i].x = 0;
            }
            if (roi_rectangle[i].y < 0)
            {
                roi_rectangle[i].y = 0;
            }
            if (roi_rectangle[i].x + roi_rectangle[i].width >= output_debug_image.cols)
            {
                roi_rectangle[i].width = output_debug_image.cols - roi_rectangle[i].x - 1;
            }
            if (roi_rectangle[i].y + roi_rectangle[i].height >= output_debug_image.rows)
            {
                roi_rectangle[i].height = output_debug_image.cols - roi_rectangle[i].y - 1;
            }
        }
 
        // Drawing the ROI and the contours and the center of the contours
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::rectangle(output_debug_image, roi_rectangle[i].tl(), roi_rectangle[i].br(), color, 1, 8, 0);
        cv::drawContours( output_debug_image, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        cv::Point tl = roi_rectangle[i].tl();
        int height = roi_rectangle[i].height;
        int width = roi_rectangle[i].width;
        cv::Point center = cv::Point(tl.x + width/2.0, tl.y + height/2.0);
        cv::circle(output_debug_image, center, 2.0, color);
       
        // Getting points on the bounding box 
        std::vector<cv::Point>  bounding_box_points;
        pointsOnRectangle( roi_rectangle[i], bounding_box_points );

        //extracting the roi images 
        cv::Rect rect_intersection = image_rect & roi_rectangle[i] ;
        cv::Mat cropped_image = input_image(rect_intersection); 

        //creating output messages to be passed back
        output_cropped_object_images.push_back( cropped_image );
        output_object_areas.push_back( rotated_roi[i].size.height * rotated_roi[i].size.width );
        output_roi_points.push_back( bounding_box_points );
        output_center_of_objects.push_back( center );
    }

}

void MultimodalObjectRecognitionROS::pointsOnRectangle(cv::Rect rect, std::vector<cv::Point> &output_points)
{
    int num_of_points = 10;
    double delta_width = rect.size().width / num_of_points;
    double delta_height = rect.size().height / num_of_points;

    for (int i=0; i<num_of_points; i++)
    {
        //Top line
        output_points.push_back(cv::Point (rect.tl().x + i*delta_width ,rect.tl().y));
    }
    for (int i=0; i<num_of_points; i++)
    {
        //right line
        output_points.push_back(cv::Point (rect.br().x ,rect.br().y - i*delta_height));
    }
    for (int i=0; i<num_of_points; i++)
    {
        //Bottom line
        output_points.push_back(cv::Point (rect.br().x - i*delta_width ,rect.br().y));
    }
    for (int i=0; i<num_of_points; i++)
    {
        // Left line
        output_points.push_back(cv::Point (rect.tl().x ,rect.tl().y + i*delta_height));
    }


}

void MultimodalObjectRecognitionROS::eventCallback(const std_msgs::String::ConstPtr &msg)
{
    std_msgs::String event_out;
    if (msg->data == "e_start")
    {
        /* sub_cloud_ = nh_.subscribe("input_cloud_topic", 1, &MultimodalObjectRecognitionROS::pointcloudCallback, this); */
        /* sub_image_ = nh_.subscribe("input_image_topic", 1, &MultimodalObjectRecognitionROS::imageCallback, this); */

        // Synchronize callback
        image_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh_, "input_image_topic", 1);
        cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "input_cloud_topic", 1);
        msg_sync_ = new message_filters::Synchronizer<msgSyncPolicy> (msgSyncPolicy(10), *image_sub_, *cloud_sub_);
        msg_sync_->registerCallback(boost::bind(&MultimodalObjectRecognitionROS::synchronizeCallback, this, _1, _2));
    }
    else if (msg->data == "e_stop")
    {
        sub_cloud_.shutdown();
        sub_image_.shutdown();
        pointcloud_segmentation_->reset_cloud_accumulation();
        event_out.data = "e_stopped";
        pub_event_out_.publish(event_out);
    }
    else
    {
        return;
    }
}

void MultimodalObjectRecognitionROS::saveDebugImage(const cv_bridge::CvImagePtr &cv_image_ptr)
{
    std::stringstream filename; // stringstream used for the conversion
    ros::Time time_now = ros::Time::now();

    // save image
    filename.str("");
    filename << logdir_ << time_now << "_bbox_rgb" <<".jpg";
    cv::imwrite(filename.str(), cv_image_ptr->image);

    cv_bridge::CvImagePtr raw_cv_image;
    try
    {
        raw_cv_image = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    filename.str("");
    filename << logdir_ << time_now << "_raw_rgb" <<".jpg";
    cv::imwrite(filename.str(), raw_cv_image->image);

}

void MultimodalObjectRecognitionROS::configCallback(mir_object_recognition::SceneSegmentationConfig &config, uint32_t level)
{
    CloudFilterParams cloudFilterParams;
    
    cloudFilterParams.mPassThroughLimitMinX = static_cast<float>(config.passthrough_limit_min_x);
    cloudFilterParams.mPassThroughLimitMaxX = static_cast<float>(config.passthrough_limit_max_x);
    
    cloudFilterParams.mPassThroughLimitMinY = static_cast<float>(config.passthrough_limit_min_y);
    cloudFilterParams.mPassThroughLimitMaxY = static_cast<float>(config.passthrough_limit_max_y);
        
    cloudFilterParams.mVoxelLimitMinZ = static_cast<float>(config.voxel_limit_min_z);
    cloudFilterParams.mVoxelLimitMaxZ = static_cast<float>(config.voxel_limit_max_z);
    cloudFilterParams.mVoxelLeafSize = static_cast<float>(config.voxel_leaf_size);
    pointcloud_segmentation_->scene_segmentation_.setCloudFilterParams(cloudFilterParams);

    SacPlaneSegmenterParams planeFitParams;
    planeFitParams.mNormalRadiusSearch = config.normal_radius_search;
    planeFitParams.mSacMaxIterations = config.sac_max_iterations;
    planeFitParams.mSacDistThreshold = config.sac_distance_threshold;
    planeFitParams.mSacOptimizeCoeffs = config.sac_optimize_coefficients;
    planeFitParams.mSacEpsAngle = config.sac_eps_angle;
    planeFitParams.mSacNormalDistWeight = config.sac_normal_distance_weight;
    pointcloud_segmentation_->scene_segmentation_.setPlaneSegmenterParams(planeFitParams);

    pointcloud_segmentation_->scene_segmentation_.setPrismParams(config.prism_min_height, config.prism_max_height);
    pointcloud_segmentation_->scene_segmentation_.setOutlierParams(config.outlier_radius_search, config.outlier_min_neighbors);
    pointcloud_segmentation_->scene_segmentation_.setClusterParams(config.cluster_tolerance, config.cluster_min_size, config.cluster_max_size,
            config.cluster_min_height, config.cluster_max_height, config.cluster_max_length,
            config.cluster_min_distance_to_polygon);

    pointcloud_segmentation_->object_height_above_workspace_ = config.object_height_above_workspace;
    rgb_container_height_ = config.rgb_container_height;
    rgb_bbox_size_adjustment_ = config.rgb_bbox_size_adjustment;
    rgb_bbox_min_diag_ = config.rgb_bbox_min_diag;
    rgb_bbox_max_diag_ = config.rgb_bbox_max_diag;
    rgb_cluster_filter_limit_min_ = config.rgb_cluster_filter_limit_min;
    rgb_cluster_filter_limit_max_ = config.rgb_cluster_filter_limit_max;
    rgb_base_link_to_laser_distance_ = config.rgb_base_link_to_laser_distance;
    rgb_max_object_pose_x_to_base_link_= config.rgb_max_object_pose_x_to_base_link;
    rgb_min_bbox_z_ = config.rgb_min_bbox_z;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "multimodal_object_recognition");
    ros::NodeHandle nh("~");

    int frame_rate = 30;
    nh.param<int>("frame_rate", frame_rate, 30);
    ROS_INFO_STREAM("[multimodal_object_recognition] node started");

    MultimodalObjectRecognitionROS object_recognition(nh);
    //boost::shared_ptr<MultimodalObjectRecognitionROS> object_recognition = boost::make_shared<MultimodalObjectRecognitionROS>(nh);

    ros::Rate loop_rate(frame_rate);
    while (ros::ok())
    {
        object_recognition.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
