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

#include <mir_object_recognition/multimodal_object_recognition_ros.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/ImageList.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/PoseArray.h>


MultimodalObjectRecognitionROS::MultimodalObjectRecognitionROS(ros::NodeHandle nh, 
                                 mcr::visualization::BoundingBoxVisualizer bounding_box_visualizer,
                                 mcr::visualization::ClusteredPointCloudVisualizer cluster_visualizer,
                                 mcr::visualization::LabelVisualizer label_visualizer):
    nh_(nh),
    pointcloud_msg_received_(false),
    pointcloud_msg_received_count_(0),
    received_recognized_cloud_list_flag_(false),
    received_recognized_image_list_flag_(false),
    bounding_box_visualizer_(bounding_box_visualizer),
    cluster_visualizer_(cluster_visualizer),
    label_visualizer_(label_visualizer),
    cluster_visualizer_pcl_("tabletop_cluster_pcl")
{
    pointcloud_segmentation_ = PointcloudSegmentationUPtr(new PointcloudSegmentationROS(nh_));
    image_recognition_ = ImageRecognitionUPtr(new ImageRecognitionROS(nh_));

    dynamic_reconfigure::Server<mir_object_recognition::SceneSegmentationConfig>::CallbackType f =
                            boost::bind(&MultimodalObjectRecognitionROS::configCallback, this, _1, _2);
    server_.setCallback(f);
    
    sub_event_in_ = nh_.subscribe("event_in", 1, &MultimodalObjectRecognitionROS::eventCallback, this);
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    
    // Publish cloud and images to cloud and rgb recognition topics
    pub_cloud_to_recognizer_  = nh_.advertise<mcr_perception_msgs::ObjectList>(
                                "recognizer/pcl/input/object_list", 10);
    pub_image_to_recognizer_  = nh_.advertise<mcr_perception_msgs::ImageList>(
                                "recognizer/rgb/input/images", 10);

    // Subscribe to cloud and rgb recognition topics
    sub_recognized_cloud_list_ = nh_.subscribe("recognizer/pcl/output/object_list", 1, 
                            &MultimodalObjectRecognitionROS::recognizedCloudCallback, this);
    sub_recognized_image_list_ = nh_.subscribe("recognizer/rgb/output/object_list", 1, 
                            &MultimodalObjectRecognitionROS::recognizedImageCallback, this);

    // Pub combined object_list to object_list merger
    pub_object_list_  = nh_.advertise<mcr_perception_msgs::ObjectList>("output/object_list", 10);

    // Pub workspace height
    pub_workspace_height_ = nh_.advertise<std_msgs::Float64>("output/workspace_height", 1);

    //Segmentation topic
    nh_.param<std::string>("segmentation_service", segmentation_service_name_, "/mcr_perception/cnn_segmentation/cnn_object_segmentation");
    segmentation_service_ = nh_.serviceClient<mcr_perception_msgs::GetSegmentedImage>(segmentation_service_name_);
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
    rgb_object_id_ = 100;
    
}

MultimodalObjectRecognitionROS::~MultimodalObjectRecognitionROS()
{

}

//void setConfig();
void MultimodalObjectRecognitionROS::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    if (! pointcloud_msg_received_ )
    {
        ROS_INFO("[multimodal_object_recognition_ros] Received pointcloud message");
        pointcloud_msg_ = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
        pointcloud_msg_ = msg;
        //pointcloud_msg_received_ = true;
        pointcloud_msg_received_count_ += 1;
    }
}

void MultimodalObjectRecognitionROS::recognizedCloudCallback(const mcr_perception_msgs::ObjectList &msg)
{
    ROS_INFO("Received recognized cloud callback ");
    recognized_cloud_list_ = msg;
    received_recognized_cloud_list_flag_ = true;
}

void MultimodalObjectRecognitionROS::recognizedImageCallback(const mcr_perception_msgs::ObjectList &msg)
{
    ROS_INFO("Received recognized image callback ");
    recognized_image_list_ = msg;
    received_recognized_image_list_flag_ = true;
}

void MultimodalObjectRecognitionROS::update()
{
    if (pointcloud_msg_received_count_ > 0)
    {
        pointcloud_msg_received_ = false;
        pointcloud_msg_received_count_ = 0;
        sub_cloud_.shutdown();
        double start_time = ros::Time::now().toSec();
        preprocessCloud();
        double end_time = ros::Time::now().toSec();
        std::cout<<"Time: "<<end_time - start_time<<std::endl;
        recognized_image_list_.objects.clear();
        recognized_cloud_list_.objects.clear(); 
    }
}

// Splitter
void MultimodalObjectRecognitionROS::preprocessCloud()
{
    std::cout<<"Preprocessing cloud"<<std::endl;
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
    pcl::PCLImage pcl_image;
    pcl::toPCLPointCloud2(*pc2, pcl_image);
    sensor_msgs::ImagePtr image_msg = boost::make_shared<sensor_msgs::Image>();
    pcl_conversions::moveFromPCL(pcl_image, *image_msg);
    
    // Convert to pcl::Pointcloud for segmentation
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromPCLPointCloud2(*pc2, *cloud);
    
    //recognize Cloud and image
    recognizeCloudAndImage(cloud, image_msg);
}


void MultimodalObjectRecognitionROS::segmentCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                                mcr_perception_msgs::ObjectList &object_list,
                                                std::vector<PointCloud::Ptr> &clusters)
{
    pointcloud_segmentation_->add_cloud_accumulation(cloud);
    pointcloud_segmentation_->segment_cloud(object_list, clusters);
}

void MultimodalObjectRecognitionROS::recognizeCloudAndImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
                                 const sensor_msgs::ImageConstPtr &image)
{
    mcr_perception_msgs::ObjectList cloud_object_list;
    std::vector<PointCloud::Ptr> clusters;
    segmentCloud(cloud, cloud_object_list, clusters);
    cluster_visualizer_pcl_.publish<PointT>(clusters, target_frame_id_);
    // Publish pose
    geometry_msgs::PoseArray pcl_object_poses;
    pcl_object_poses.poses.resize(cloud_object_list.objects.size());
    pcl_object_poses.header.frame_id = target_frame_id_;
    pcl_object_poses.header.stamp = ros::Time::now();
    for (int i=0; i<cloud_object_list.objects.size(); i++)
    {
        pcl_object_poses.poses[i] = cloud_object_list.objects[i].pose.pose;
    }
    pub_pcl_object_pose_array_.publish(pcl_object_poses);

    // Compute normal to generate parallel BBOX with the plane
    const Eigen::Vector3f normal(pointcloud_segmentation_->scene_segmentation_.coefficients_->values[0], 
                                 pointcloud_segmentation_->scene_segmentation_.coefficients_->values[1], 
                                 pointcloud_segmentation_->scene_segmentation_.coefficients_->values[2]);
    //Publish cluster for recognition
    ROS_INFO_STREAM("Publishing clouds for recognition");
    ROS_INFO_STREAM("Using ... for recognition");
    pub_cloud_to_recognizer_.publish(cloud_object_list);
    //publish rgb for recognition
    ROS_INFO_STREAM("Publishing images for recognition");
    ROS_INFO_STREAM("Using ... for recognition");

    // Use semantic_segmentation
    bool use_semantic_segmentation = false;
    if (use_semantic_segmentation)
    {   
        cv::Mat segmented_objects_mask;
        segmentObjects(image, segmented_objects_mask);
        
        // Detect objects based on the segmented mask
        cv::Mat debug_image;
        std::vector <cv::Mat>  object_images;
        std::vector <float>  object_areas;
        std::vector<std::vector<cv::Point> > object_roi_points;
        std::vector<cv::Point> object_center_points;
        ROS_INFO_STREAM("Starting object extraction");

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

        getCroppedImages(cv_image->image, segmented_objects_mask,debug_image,
                        object_images, object_areas, object_roi_points, object_center_points);
        ROS_INFO_STREAM("DONE OBJECT EXTRACTION");

    }
    mcr_perception_msgs::ImageList image_list;
    image_list.images.resize(1);
    image_list.images[0] = *image;
    pub_image_to_recognizer_.publish(image_list);
    ROS_INFO("Waiting for message from SSD MobileNet recognition");
    //loop till you received the message from the 3d and rgb recognition
    int loop_rate_hz = 30;
    int timeout_wait = 8; //secs
    ros::Rate loop_rate(loop_rate_hz);
    int loop_rate_count = 0;

    // TODO: CHeck whether the pcl clusters and rgb image exist

    //bool received_cloud_image_list = false;
    bool received_cloud_list = false;
    bool received_image_list = false;
    //while((!received_image_list) && (!received_cloud_list))
    while(!received_image_list)
    {
        loop_rate.sleep();
        loop_rate_count += 1;
        ros::spinOnce();
        if (received_recognized_image_list_flag_ == true)
        {
            received_image_list = true;
            ROS_INFO_STREAM("[RGB] Received object_list from rgb recognizer");
            ROS_INFO_STREAM("[RGB] Received objects: "<<recognized_image_list_.objects.size());
        }
        if (received_recognized_cloud_list_flag_ == true)
        {
            received_cloud_list = true;
            ROS_INFO_STREAM("[PCL] Received object_list from pcl recognizer");
            ROS_INFO_STREAM("[PCL] Received objects: "<<recognized_cloud_list_.objects.size());
        }
        if (loop_rate_count > loop_rate_hz * timeout_wait)
        {
            ROS_ERROR("No message received from both RGB and PCL recognizer. ");
            return; //If not recevied in timeout secs return
        }
        std::cout<<"[RGB]: "<<received_recognized_image_list_flag_<<std::endl;
        std::cout<<"[PCL]: "<<received_recognized_cloud_list_flag_<<std::endl;
    }
    std::cout<<"Cloud list: "<< recognized_cloud_list_.objects.size()<<std::endl;
    std::cout<<"Image list: "<< recognized_image_list_.objects.size()<<std::endl;
    std::cout<<"===================="<<std::endl;
    //received_recognized_image_list_flag_ = false;
    //received_recognized_cloud_list_flag_ = false;
    // TODO: Handle empty objects here
    // Postprocess RGB List
    // 1. Generate 3d object
    mcr_perception_msgs::ObjectList final_image_list;
    if (received_image_list == true)
    {
        std::cout<<"Processing 3D Bbox"<<std::endl;
        cv_bridge::CvImagePtr cv_image;
        try
        {
            cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Pose array for debug mode only
        geometry_msgs::PoseArray rgb_object_pose_array;
        std::vector<geometry_msgs::Pose> rgb_object_poses;
        
        mcr_perception_msgs::BoundingBoxList bounding_boxes;
        std::vector<PointCloud::Ptr> clusters_2d;
        bounding_boxes.bounding_boxes.resize(recognized_image_list_.objects.size());
        final_image_list.objects.resize(recognized_image_list_.objects.size());

        std::cout<<"Image size: "<<cv_image->image.rows<<", "<<cv_image->image.cols<<std::endl;
        for (int i=0; i<recognized_image_list_.objects.size();i++)
        {
            std::cout<<"Class: "<<recognized_image_list_.objects[i].name<<std::endl;
            mcr_perception_msgs::Object object = recognized_image_list_.objects[i];
            //Get ROI
            sensor_msgs::RegionOfInterest roi_2d = object.roi;
            const cv::Rect2d rect2d(roi_2d.x_offset, roi_2d.y_offset, roi_2d.width, roi_2d.height);

            std::cout<<"Region of Interest: cx, cy, width, height"<<std::endl;
            std::cout<<roi_2d.x_offset<<", "<<roi_2d.y_offset<<", "<<roi_2d.width<<", "<<roi_2d.height<<std::endl;
            
            // TODO: Create Get pose in _utils.cpp
            // NOTE: Remove large 2d misdetected bbox (misdetection)
            // Solution find diagional, and make a threshold
            // Cannot compute 3d bbox for small outlier bbox
            double len_diag = sqrt(powf(((roi_2d.width + roi_2d.width) >> 1), 2));
            std::cout<<"Diagonal: "<<len_diag<<std::endl;
            std::cout<<"Normal: "<<normal<<std::endl;
            if (len_diag > 21 && len_diag < 200)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_object_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                std::cout<<"Converting 2D region proposal to 3D"<<std::endl;
                get3DObject(roi_2d, cloud, pcl_object_cluster);
                std::cout<<"PCL size: "<<pcl_object_cluster->height<<", "<<pcl_object_cluster->width<<std::endl;
                pcl::PointXYZRGB minPt_2d, maxPt_2d;
                pcl::getMinMax3D(*pcl_object_cluster, minPt_2d, maxPt_2d);
                std::cout<<"Min Max 2dcloud: "<<minPt_2d.z<<", "<<maxPt_2d.z<<std::endl;

                clusters_2d.push_back(pcl_object_cluster);

                mas_perception_libs::BoundingBox bbox;
                std::cout<<"Calculating 3D BBox given 3D cloud cluster"<<std::endl;
                pointcloud_segmentation_->get3DBoundingBox(pcl_object_cluster, normal, bbox, bounding_boxes.bounding_boxes[i]);
                std::cout<<"Calculating pose given 3D BBox"<<std::endl;
                geometry_msgs::PoseStamped pose = pointcloud_segmentation_->getPose(bbox);

                // Transform pose
                std::string frame_id = cloud->header.frame_id;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = frame_id;
                std::string target_frame_id;
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
                            geometry_msgs::PoseStamped pose_transformed;
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

                //object_list.objects[i].pointcloud = ros_cloud;
                final_image_list.objects[i].name = recognized_image_list_.objects[i].name;
                final_image_list.objects[i].probability = recognized_image_list_.objects[i].probability;
                final_image_list.objects[i].database_id = rgb_object_id_;
                rgb_object_id_++;

                // debug_mode only
                // if (debug_mode_)
                // {
                    std::cout<<"DEGUB rgb"<<std::endl;
                    rgb_object_poses.push_back(pose.pose);
                //}


                //TODO: find max intersection between rgb and 3d bboxes (IOU)
                // for (int j=0; j<clusters.size(); j++)
                // {
                //     pcl::PointXYZRGB minPt, maxPt;
                //     pcl::getMinMax3D(*clusters[i], minPt, maxPt);
                //     int roi_3d_x_offset = minPt.x;
                //     int roi_3d_y_offset = minPt.y;
                //     int roi_3d_width = maxPt.x - minPt.x;
                //     int roi_3d_height = maxPt.y - minPt.y;
                //     const cv::Rect2d rect3d(roi_3d_x_offset, roi_3d_y_offset, roi_3d_width, roi_3d_height);
                //     std::cout<<"3D ROI"<<std::endl;
                //     std::cout<<roi_3d_x_offset<<", "<<roi_3d_y_offset<<", "<<roi_3d_width<<", "<<roi_3d_height<<std::endl;
                //     double area = getMatch(rect2d, rect3d);
                //     std::cout<<"Overlapping area: "<<area<<std::endl;

                // }
            }
            else
            {
                std::cout<<"DECOY"<<std::endl;
                final_image_list.objects[i].name = "DECOY";
            }
            //pose.header.stamp = now;
            // TODO: Transform pose to base_link, see scene_segmentation
        }
        // if (debug_mode_)
        // {
            //bounding_box_visualizer_.publish(bounding_boxes.bounding_boxes, target_frame_id_);
            cluster_visualizer_.publish<PointT>(clusters_2d, target_frame_id_);
            rgb_object_pose_array.header.frame_id = target_frame_id_;
            rgb_object_pose_array.header.stamp = ros::Time::now();
            rgb_object_pose_array.poses.resize(rgb_object_poses.size());
            //rgb_object_pose_array.poses.insert(rgb_object_pose_array.poses.end(), rgb_object_poses.begin(), rgb_object_poses.end());
            for (int i=0; i<rgb_object_pose_array.poses.size(); i++)
            {
                rgb_object_pose_array.poses[i] = rgb_object_poses[i];
            }
            pub_rgb_object_pose_array_.publish(rgb_object_pose_array);
        //}
    }
    std::cout<<"Cloud list: "<< cloud_object_list.objects.size()<<std::endl;
    std::cout<<"Image list: "<< final_image_list.objects.size()<<std::endl;

    // TODO: handle EXCEPTION here (REMEMBER IT WILL KILL THIS NODE)
    // Merge cloud_list and image_ist
    mcr_perception_msgs::ObjectList final_object_list;
    final_object_list.objects.resize(recognized_cloud_list_.objects.size() + final_image_list.objects.size());
    for (int i=0; i<recognized_cloud_list_.objects.size();i++)
    {
        final_object_list.objects[i] = recognized_cloud_list_.objects[i];
        std::cout<<"PCL Label: "<<final_object_list.objects[i].name<<std::endl;
    }
    for (int i=0; i<final_image_list.objects.size();i++)
    {
        final_object_list.objects[i] = final_image_list.objects[i];
        std::cout<<"RGB Label: "<<final_object_list.objects[i].name<<std::endl;
    }
    //final_object_list.objects = cloud_object_list.objects;
    //combineObjectList(cloud_object_list, final_image_list, final_object_list);
    // if (received_image_list)
    // {
    //     cloud_object_list.objects.insert(cloud_object_list.objects.end(), final_image_list.objects.begin(), final_image_list.objects.end());
    // }
    std::cout<<"Final list: "<< final_object_list.objects.size()<<std::endl;


    // TODO: remove cloud before publish
    // TODO: Edit object_list merger to receive a single list containing 3d and rgb
    pub_object_list_.publish(final_object_list);
    //pub_object_list_.publish(final_image_list);
    //Resetting the flag for next message
    received_recognized_image_list_flag_ = false;
    received_recognized_cloud_list_flag_ = false;
    sub_cloud_.shutdown();
}

void MultimodalObjectRecognitionROS::combineObjectList(const mcr_perception_msgs::ObjectList& cloud_list, 
                                                       const mcr_perception_msgs::ObjectList& image_list,
                                                       mcr_perception_msgs::ObjectList& final_object_list)
{
    
    //final_object_list.objects = cloud_list.objects.merge(image_list.objects);
}

// Intel ros object analytics
// BBox merger
double MultimodalObjectRecognitionROS::getMatch(const cv::Rect2d& r1, const cv::Rect2d& r2)
{
  cv::Rect2i ir1(r1), ir2(r2);
  /* calculate center of rectangle #1*/
  cv::Point2i c1(ir1.x + (ir1.width >> 1), ir1.y + (ir1.height >> 1));
  /* calculate center of rectangle #2*/
  cv::Point2i c2(ir2.x + (ir2.width >> 1), ir2.y + (ir2.height >> 1));

  double a1 = ir1.area(), a2 = ir2.area(), a0 = (ir1 & ir2).area();
  /* calculate the overlap rate*/
  double overlap = a0 / (a1 + a2 - a0);
  /* calculate the deviation between centers #1 and #2*/
  double deviate = sqrt(powf((c1.x - c2.x), 2) + powf((c1.y - c2.y), 2));
  /* calculate the length of diagonal for the rectangle in average size*/
  double len_diag = sqrt(powf(((ir1.width + ir2.width) >> 1), 2) + powf(((ir1.height + ir2.height) >> 1), 2));

  /* calculate the match rate. The more overlap, the more matching. Contrary, the more deviation, the less matching*/
  return overlap * len_diag / deviate;
}


void MultimodalObjectRecognitionROS::get3DObject(const sensor_msgs::RegionOfInterest &roi, 
                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ordered_cloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_object)
{
    int pixel_loc_tolerance = 3;
    int min_x = roi.x_offset;
    int min_y = roi.y_offset;
    int max_x = roi.x_offset + roi.width;
    int max_y = roi.y_offset + roi.height;

    // Add BBox tolerance
    if (roi.x_offset > pixel_loc_tolerance) min_x = min_x - pixel_loc_tolerance;
    if (roi.y_offset > pixel_loc_tolerance) min_y = min_y - pixel_loc_tolerance;
    if (roi.width+pixel_loc_tolerance < ordered_cloud->width) min_x = min_x + pixel_loc_tolerance;
    if (roi.height+pixel_loc_tolerance < ordered_cloud->height) min_y = min_y + pixel_loc_tolerance;
    
    std::cout<<"PCL size: "<<ordered_cloud->height<<", "<<ordered_cloud->width<<std::endl;
    std::cout<<min_x<<", "<<min_y<<", "<<max_x<<", "<<max_y<<std::endl;
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
        //pcl_objects.push_back(pcl_object);
    }
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
        mcr_perception_msgs::GetSegmentedImage srv;
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

    //input_image.copyTo(output_debug_image) ;

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
        //add_to_octree_ = true;
        sub_cloud_ = nh_.subscribe("input", 1, &MultimodalObjectRecognitionROS::pointcloudCallback, this);
        event_out.data = "e_started";
    }
    else if (msg->data == "e_stop")
    {
        sub_cloud_.shutdown();
        pointcloud_segmentation_->reset_cloud_accumulation();
        event_out.data = "e_stopped";
    }
    else
    {
        return;
    }
    pub_event_out_.publish(event_out);
}

void MultimodalObjectRecognitionROS::configCallback(mir_object_recognition::SceneSegmentationConfig &config, uint32_t level)
{
    pointcloud_segmentation_->scene_segmentation_.setVoxelGridParams(config.voxel_leaf_size, config.voxel_filter_field_name,
            config.voxel_filter_limit_min, config.voxel_filter_limit_max);
    pointcloud_segmentation_->scene_segmentation_.setPassthroughParams(config.passthrough_filter_field_name,
            config.passthrough_filter_limit_min,
            config.passthrough_filter_limit_max);
    pointcloud_segmentation_->scene_segmentation_.setNormalParams(config.normal_radius_search);
    pointcloud_segmentation_->scene_segmentation_.setSACParams(config.sac_max_iterations, config.sac_distance_threshold,
            config.sac_optimize_coefficients, config.sac_eps_angle,
            config.sac_normal_distance_weight);
    pointcloud_segmentation_->scene_segmentation_.setPrismParams(config.prism_min_height, config.prism_max_height);
    pointcloud_segmentation_->scene_segmentation_.setOutlierParams(config.outlier_radius_search, config.outlier_min_neighbors);
    pointcloud_segmentation_->scene_segmentation_.setClusterParams(config.cluster_tolerance, config.cluster_min_size, config.cluster_max_size,
            config.cluster_min_height, config.cluster_max_height, config.cluster_max_length,
            config.cluster_min_distance_to_polygon);
    std::cout<<"============================"<<std::endl;
    std::cout<<"Normal radiuse "<<config.normal_radius_search<<std::endl;
    std::cout<<"Voxel leaf "<<config.voxel_leaf_size<<std::endl;
    pointcloud_segmentation_->object_height_above_workspace_ = config.object_height_above_workspace;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "multimodal_object_recognition");
    ros::NodeHandle nh("~");

    using mcr::visualization::BoundingBoxVisualizer;
    using mcr::visualization::ClusteredPointCloudVisualizer;
    using mcr::visualization::LabelVisualizer;
    using mcr::visualization::Color;
    BoundingBoxVisualizer bounding_box_visualizer("bounding_boxes", Color(Color::SEA_GREEN));
    ClusteredPointCloudVisualizer cluster_visualizer("tabletop_clusters");
    LabelVisualizer label_visualizer("labels", Color(Color::TEAL));

    int frame_rate = 30;
    nh.param<int>("frame_rate", frame_rate, 30);
    ROS_INFO_STREAM("[multimodal_object_recognition] node started");

    MultimodalObjectRecognitionROS object_recognition(nh, bounding_box_visualizer, cluster_visualizer, label_visualizer);
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