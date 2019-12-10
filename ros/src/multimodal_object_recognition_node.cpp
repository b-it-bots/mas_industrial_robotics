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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/PoseArray.h>

#include <mas_perception_msgs/ObjectList.h>
#include <mas_perception_msgs/ImageList.h>
#include <mas_perception_msgs/BoundingBoxList.h>

#include <mas_perception_libs/bounding_box.h>
#include <mas_perception_libs/point_cloud_utils.h>
#include <mas_perception_libs/sac_plane_segmenter.h>

#include <mir_object_recognition/multimodal_object_recognition_node.h>
#include <mir_object_recognition/SceneSegmentationConfig.h>

using mcr::visualization::BoundingBoxVisualizer;
using mcr::visualization::ClusteredPointCloudVisualizer;
using mcr::visualization::LabelVisualizer;
using mas_perception_libs::Color;
using mas_perception_libs::CloudFilterParams;
using mas_perception_libs::SacPlaneSegmenterParams;

MultimodalObjectRecognitionROS::MultimodalObjectRecognitionROS(ros::NodeHandle nh):
    nh_(nh),
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
    tf_listener_.reset(new tf::TransformListener);
    pointcloud_segmentation_ = PointcloudSegmentationUPtr(new PointcloudSegmentationROS(nh_, tf_listener_));
    mm_object_recognition_utils_ = MultimodalObjectRecognitionUtilsUPtr(new MultimodalObjectRecognitionUtils(tf_listener_));
    
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

    nh_.param<bool>("debug_mode", debug_mode_, false);
    ROS_WARN_STREAM("[multimodal_object_recognition] Debug mode: " <<debug_mode_);
    // Pub pose array
    pub_pcl_object_pose_array_  = nh_.advertise<geometry_msgs::PoseArray>("output/pcl_object_pose_array", 10);
    pub_rgb_object_pose_array_  = nh_.advertise<geometry_msgs::PoseArray>("output/rgb_object_pose_array", 10);

    nh_.param<std::string>("target_frame_id", target_frame_id_, "base_link");
    ROS_WARN_STREAM("[multimodal_object_recognition] target frame: " <<target_frame_id_);
    
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
    if (pointcloud_msg_received_count_ < 1)
    {
        ROS_INFO("[multimodal_object_recognition_ros] Received enough messages");
        pointcloud_msg_ = cloud;
        pointcloud_msg_received_count_ += 1;

        image_msg_ = image;
        image_msg_received_count_ += 1;
    }
}

void MultimodalObjectRecognitionROS::recognizedCloudCallback(const mas_perception_msgs::ObjectList &msg)
{
    ROS_INFO("Received recognized cloud callback ");
    if (!received_recognized_cloud_list_flag_)
    {
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
    if (pointcloud_msg_received_count_ > 0 && image_msg_received_count_ > 0)
    {
        ROS_WARN("Received %d images and pointclouds", pointcloud_msg_received_count_);
        // Reset msg received flag
        pointcloud_msg_received_count_ = 0;
        image_msg_received_count_ = 0;
        
        image_sub_->unsubscribe();
        cloud_sub_->unsubscribe();

        ROS_WARN_STREAM("Starting multimodal object recognition");
        double start_time = ros::Time::now().toSec();
        recognizeCloudAndImage();
        double end_time = ros::Time::now().toSec();
        ROS_INFO_STREAM("Total processing time: "<<end_time - start_time);

        // Reset received recognized cloud and image
        received_recognized_cloud_list_flag_ = false;
        received_recognized_image_list_flag_ = false;

        // reset object id
        rgb_object_id_ = 100;
        pointcloud_segmentation_->resetPclObjectId();

        // clear recognized image and cloud list
        recognized_image_list_.objects.clear();
        recognized_cloud_list_.objects.clear(); 
        
        pointcloud_segmentation_->reset_cloud_accumulation();
        // pub e_done
        std_msgs::String event_out;
        event_out.data = "e_done";
        pub_event_out_.publish(event_out);
    }
}

void MultimodalObjectRecognitionROS::transformCloud()
{
    std::string target_frame_id;
    nh_.param<std::string>("target_frame_id", target_frame_id, "base_link");
    sensor_msgs::PointCloud2 msg_transformed;
    msg_transformed.header.frame_id = target_frame_id;
    try
    {
        ros::Time common_time;
        tf_listener_->getLatestCommonTime(target_frame_id, pointcloud_msg_->header.frame_id, common_time, NULL);
        pointcloud_msg_->header.stamp = common_time;
        tf_listener_->waitForTransform(target_frame_id, pointcloud_msg_->header.frame_id,
                                                ros::Time::now(), ros::Duration(1.0));
        pcl_ros::transformPointCloud(target_frame_id, *pointcloud_msg_, msg_transformed, *tf_listener_);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("PCL transform error: %s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    // Set frame id 
    pointcloud_segmentation_->frame_id_ = target_frame_id;
    pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msg_transformed, *pc2);
    pc2->header.frame_id = msg_transformed.header.frame_id;
    
    // Convert to pcl::Pointcloud for segmentation
    cloud_ = PointCloud::Ptr(new PointCloud); 
    pcl::fromPCLPointCloud2(*pc2, *cloud_);
}


void MultimodalObjectRecognitionROS::segmentPointcloud(mas_perception_msgs::ObjectList &object_list, 
                                                       std::vector<PointCloud::Ptr> &clusters)
{
    pointcloud_segmentation_->add_cloud_accumulation(cloud_);
    pointcloud_segmentation_->segment_cloud(object_list, clusters);

    std_msgs::Float64 workspace_height_msg;
    workspace_height_msg.data = pointcloud_segmentation_->getWorkspaceHeight();
    pub_workspace_height_.publish(workspace_height_msg);

}

void MultimodalObjectRecognitionROS::recognizeCloudAndImage()
{
    // transform pointcloud to the given frame_id
    transformCloud();

    mas_perception_msgs::ObjectList cloud_object_list;
    std::vector<PointCloud::Ptr> clusters_3d;

    segmentPointcloud(cloud_object_list, clusters_3d);

    //Publish cluster for recognition
    if (!cloud_object_list.objects.empty())
    {
        ROS_INFO_STREAM("Publishing clouds for recognition");
        pub_cloud_to_recognizer_.publish(cloud_object_list);
    }
    
    // Pub Image to recognizer
    mas_perception_msgs::ImageList image_list;
    image_list.images.resize(1);
    image_list.images[0] = *image_msg_;
    if (!image_list.images.empty())
    {
        ROS_INFO_STREAM("Publishing images for recognition");
        pub_image_to_recognizer_.publish(image_list);
    }
    ROS_INFO_STREAM("Waiting for message from Cloud and Image recognizer");
    //loop till you received the message from the 3d and rgb recognition
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
                ROS_WARN("[PCL] Received %d objects from pcl recognizer", recognized_cloud_list_.objects.size());
            }
            if (loop_rate_count > loop_rate_hz * timeout_wait)
            {
                received_recognized_cloud_list_flag_ = false;
                ROS_ERROR("[PCL] No message received from PCL recognizer. ");
                break;
            }
        }
    }

    // Merge recognized_cloud_list and rgb_object_list
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
                ROS_WARN("[RGB] Received %d objects from rgb recognizer", recognized_image_list_.objects.size());
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

    mas_perception_msgs::ObjectList rgb_object_list;
    mas_perception_msgs::BoundingBoxList bounding_boxes;
    std::vector<PointCloud::Ptr> clusters_2d;
    
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
        rgb_object_list.objects.resize(recognized_image_list_.objects.size());

        for (int i=0; i<recognized_image_list_.objects.size();i++)
        {
            mas_perception_msgs::Object object = recognized_image_list_.objects[i];
            //Get ROI
            sensor_msgs::RegionOfInterest roi_2d = object.roi;
            const cv::Rect2d rect2d(roi_2d.x_offset, roi_2d.y_offset, roi_2d.width, roi_2d.height);

            if (debug_mode_)
            {
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
            // Remove large 2d misdetected bbox (misdetection)
            double len_diag = sqrt(powf(((roi_2d.width + roi_2d.width) >> 1), 2));
            if (len_diag > rgb_bbox_min_diag_ && len_diag < rgb_bbox_max_diag_)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_object_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                get3DObject(roi_2d, cloud_, pcl_object_cluster);
                pcl_object_cluster->header.frame_id = cloud_->header.frame_id;

                // Filter big objects from 2d proposal, if the height is less than 3 mm
                //pcl::PointXYZRGB min_pt;
                //pcl::PointXYZRGB max_pt;
                //pcl::getMinMax3D(*pcl_object_cluster, min_pt, max_pt);
                //float obj_height = max_pt.z - pointcloud_segmentation_->getWorkspaceHeight();

                sensor_msgs::PointCloud2 ros_pc2;
                pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
                pcl::toPCLPointCloud2(*pcl_object_cluster, *pc2);
                pcl_conversions::fromPCL(*pc2, ros_pc2);
                ros_pc2.header.frame_id = target_frame_id_;
                ros_pc2.header.stamp = ros::Time::now();
                rgb_object_list.objects[i].pointcloud = ros_pc2;

                clusters_2d.push_back(pcl_object_cluster);
                // Get pose
                geometry_msgs::PoseStamped pose = mm_object_recognition_utils_->estimatePose(pcl_object_cluster, 
                                                                    recognized_image_list_.objects[i].name);

                // Transform pose
                std::string frame_id = cloud_->header.frame_id;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = frame_id;
                if (frame_id != target_frame_id_)
                {   
                    mm_object_recognition_utils_->transformPose(frame_id, target_frame_id_, pose, rgb_object_list.objects[i].pose);
                }
                else
                {
                    rgb_object_list.objects[i].pose = pose;
                }
                rgb_object_list.objects[i].probability = recognized_image_list_.objects[i].probability;
                rgb_object_list.objects[i].database_id = rgb_object_id_;
                rgb_object_list.objects[i].name = recognized_image_list_.objects[i].name;
            }
            else
            {
                std::cout<<"[RGB] DECOY"<<std::endl;
                rgb_object_list.objects[i].name = "DECOY";
                rgb_object_list.objects[i].database_id = rgb_object_id_;
            }
            rgb_object_id_++;
        }
        combined_object_list.objects.insert(combined_object_list.objects.end(), 
                                        rgb_object_list.objects.begin(),
                                        rgb_object_list.objects.end());
    }

    if (!combined_object_list.objects.empty())
    {
        for (int i=0; i<combined_object_list.objects.size(); i++)
        {
            double current_object_pose_x = combined_object_list.objects[i].pose.pose.position.x;
            if (current_object_pose_x < rgb_base_link_to_laser_distance_ ||
                current_object_pose_x > rgb_max_object_pose_x_to_base_link_)
                /* combined_object_list.objects[i].pose.pose.position.z < pointcloud_segmentation_ */
                /* ->object_height_above_workspace_ - 0.05) */
            {
                ROS_WARN_STREAM("I got you filtered: "<<combined_object_list.objects[i].name<<" SORRY!!!");
                ROS_WARN_STREAM("You are a DECOY now");
                std::cout<<"#########Filtering: "<<combined_object_list.objects[i].name<<std::endl;
                std::cout<<"base link to laser: "<<rgb_base_link_to_laser_distance_<<std::endl;
                std::cout<<"rgb max object pose to baselink: "<<rgb_max_object_pose_x_to_base_link_<<std::endl;
                std::cout<<"Pose X:  "<<current_object_pose_x<<std::endl;                
                combined_object_list.objects[i].name = "DECOY";
            }
        }
        // Adjust RPY to make pose flat, adjust container pose 
        // Adjust Axis and Bolt pose
        adjustObjectPose(combined_object_list);
        //Publish object to object list merger
        publishObjectList(combined_object_list);
    }
    else
    {
        ROS_WARN("No objects to publish");
        return;
    }
    
    if (debug_mode_)
    {
        ROS_WARN_STREAM("Debug mode: publishing object information");
        publishDebug(combined_object_list, clusters_3d, clusters_2d);
        mm_object_recognition_utils_->saveDebugImage(cv_image, image_msg_, logdir_); 
    }
}

void MultimodalObjectRecognitionROS::publishDebug(mas_perception_msgs::ObjectList &combined_object_list,
                                                  std::vector<PointCloud::Ptr> &clusters_3d,
                                                  std::vector<PointCloud::Ptr> &clusters_2d)
{
    std::cout<<"Cloud list: "<< recognized_cloud_list_.objects.size()<<std::endl;
    std::cout<<"RGB list: "<< recognized_image_list_.objects.size()<<std::endl;

    ROS_INFO_STREAM("Combined object list: "<< combined_object_list.objects.size());
    // Compute normal to generate parallel BBOX to the plane
    const Eigen::Vector3f normal = pointcloud_segmentation_->getPlaneNormal();
    
    if (recognized_cloud_list_.objects.size() > 0)
    {
        // Bounding boxes
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
                std::cout<<"[PCL] Class: "<<combined_object_list.objects[i].name<<std::endl;
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
        cluster_visualizer_rgb_.publish<PointT>(clusters_2d, target_frame_id_);
        // RGB Pose array for debug mode only
        geometry_msgs::PoseArray rgb_object_pose_array;
        rgb_object_pose_array.header.frame_id = target_frame_id_;
        rgb_object_pose_array.header.stamp = ros::Time::now();
        rgb_object_pose_array.poses.resize(recognized_image_list_.objects.size());
        std::vector<std::string> rgb_labels;
        int rgb_count = 0;
        for (int i=0; i<combined_object_list.objects.size(); i++)
        {
            if (combined_object_list.objects[i].database_id > 99)
            {
                std::cout<<"[RGB] Class: "<<combined_object_list.objects[i].name<<std::endl;
                rgb_object_pose_array.poses[rgb_count] = combined_object_list.objects[i].pose.pose;
                rgb_labels.push_back(combined_object_list.objects[i].name);
                rgb_count ++;
            }
        }

        if (rgb_object_pose_array.poses.size() > 0)
        {
            pub_rgb_object_pose_array_.publish(rgb_object_pose_array);
        }
        if ((rgb_labels.size() == rgb_object_pose_array.poses.size()) && 
                (rgb_labels.size() > 0) && (rgb_object_pose_array.poses.size() > 0))
        {
            label_visualizer_rgb_.publish(rgb_labels, rgb_object_pose_array);
        }
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

void MultimodalObjectRecognitionROS::adjustObjectPose(mas_perception_msgs::ObjectList &object_list)
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
            //object_list.objects[i].pose.pose.position.z = pointcloud_segmentation_->getWorkspaceHeight() + 0.05 ;
            /* change_in_pitch = -M_PI / 6.0; */
            object_list.objects[i].pose.pose.position.z = pointcloud_segmentation_->getWorkspaceHeight() + 0.08 ;
            if (object_list.objects[i].database_id > 100)
            {
                ROS_INFO_STREAM("Updating container pose");
                mm_object_recognition_utils_->adjustContainerPose(object_list.objects[i], rgb_container_height_);
            }
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

        if (object_list.objects[i].name == "M20_100" || object_list.objects[i].name == "AXIS")
        {
            mm_object_recognition_utils_->adjustAxisBoltPose(object_list.objects[i]);
        }
    }
}

void MultimodalObjectRecognitionROS::eventCallback(const std_msgs::String::ConstPtr &msg)
{
    std_msgs::String event_out;
    if (msg->data == "e_start")
    {
        // Synchronize callback
        image_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh_, "input_image_topic", 1);
        cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "input_cloud_topic", 1);
        msg_sync_ = new message_filters::Synchronizer<msgSyncPolicy> (msgSyncPolicy(10), *image_sub_, *cloud_sub_);
        msg_sync_->registerCallback(boost::bind(&MultimodalObjectRecognitionROS::synchronizeCallback, this, _1, _2));
    }
    else if (msg->data == "e_stop")
    {
        pointcloud_segmentation_->reset_cloud_accumulation();
        event_out.data = "e_stopped";
        pub_event_out_.publish(event_out);
    }
    else
    {
        return;
    }
}

/* void MultimodalObjectRecognitionROS::saveDebugImage(const cv_bridge::CvImagePtr &cv_image_ptr)
{
    std::stringstream filename; 
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

} */

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
    ros::Rate loop_rate(frame_rate);

    ROS_WARN_STREAM("[multimodal_object_recognition] node started with rate "<<frame_rate);

    MultimodalObjectRecognitionROS mm_object_recognition(nh);
    
    while (ros::ok())
    {
        mm_object_recognition.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
