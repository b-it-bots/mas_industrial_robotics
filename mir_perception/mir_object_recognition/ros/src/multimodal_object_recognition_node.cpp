/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>

#include <mas_perception_msgs/ImageList.h>
#include <mas_perception_msgs/BoundingBoxList.h>

#include <mir_perception_utils/clustered_point_cloud_visualizer.h>
#include <mir_perception_utils/bounding_box_visualizer.h>
#include <mir_perception_utils/label_visualizer.h>
#include <mir_perception_utils/bounding_box.h>

#include <mir_object_recognition/multimodal_object_recognition_node.h>

MultimodalObjectRecognitionROS::MultimodalObjectRecognitionROS(ros::NodeHandle nh):
  nh_(nh),
  pointcloud_msg_received_count_(0),
  image_msg_received_count_(0),
  received_recognized_cloud_list_flag_(false),
  received_recognized_image_list_flag_(false),
  rgb_object_id_(100),
  rgb_container_height_(0.05),
  rgb_roi_adjustment_(2),
  rgb_bbox_min_diag_(21),
  rgb_bbox_max_diag_(250),
  roi_min_bbox_z_(0.03),
  bounding_box_visualizer_pc_("output/bounding_boxes", Color(Color::IVORY)),
  cluster_visualizer_rgb_("output/tabletop_cluster_rgb"),
  cluster_visualizer_pc_("output/tabletop_cluster_pc"),
  label_visualizer_rgb_("output/rgb_labels", Color(Color::SEA_GREEN)),
  label_visualizer_pc_("output/pc_labels", Color(Color::IVORY)),
  data_collection_(false)
{
  tf_listener_.reset(new tf::TransformListener);
  scene_segmentation_ros_ = SceneSegmentationROSSPtr(new SceneSegmentationROS());
  mm_object_recognition_utils_ = MultimodalObjectRecognitionUtilsSPtr(new MultimodalObjectRecognitionUtils());

  dynamic_reconfigure::Server<mir_object_recognition::SceneSegmentationConfig>::CallbackType f =
              boost::bind(&MultimodalObjectRecognitionROS::configCallback, this, _1, _2);
  server_.setCallback(f);

  sub_event_in_ = nh_.subscribe("event_in", 1, &MultimodalObjectRecognitionROS::eventCallback, this);
  pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);

  // Publish cloud and images to cloud and rgb recognition topics
  pub_cloud_to_recognizer_  = nh_.advertise<mas_perception_msgs::ObjectList>(
                "recognizer/pc/input/object_list", 1);
  pub_image_to_recognizer_  = nh_.advertise<mas_perception_msgs::ImageList>(
                "recognizer/rgb/input/images", 1);

  // Subscribe to cloud and rgb recognition topics
  sub_recognized_cloud_list_ = nh_.subscribe("recognizer/pc/output/object_list", 1,
              &MultimodalObjectRecognitionROS::recognizedCloudCallback, this);
  sub_recognized_image_list_ = nh_.subscribe("recognizer/rgb/output/object_list", 1,
              &MultimodalObjectRecognitionROS::recognizedImageCallback, this);

  // Pub combined object_list to object_list merger
  pub_object_list_  = nh_.advertise<mas_perception_msgs::ObjectList>("output/object_list", 10);

  // Pub workspace height
  pub_workspace_height_ = nh_.advertise<std_msgs::Float64>("output/workspace_height", 1);

  // debug topics
  pub_debug_cloud_plane_ = nh_.advertise<sensor_msgs::PointCloud2>("output/debug_cloud_plane", 1);

  nh_.param<bool>("debug_mode", debug_mode_, false);
  ROS_WARN_STREAM("[multimodal_object_recognition] Debug mode: " <<debug_mode_);
  // Pub pose array
  pub_pc_object_pose_array_  = nh_.advertise<geometry_msgs::PoseArray>("output/pc_object_pose_array", 10);
  pub_rgb_object_pose_array_  = nh_.advertise<geometry_msgs::PoseArray>("output/rgb_object_pose_array", 10);

  nh_.param<std::string>("target_frame_id", target_frame_id_, "base_link");
  ROS_WARN_STREAM("[multimodal_object_recognition] target frame: " <<target_frame_id_);

  nh_.param<std::string>("logdir", logdir_, "/tmp/");
  nh_.param<std::string>("object_info", object_info_path_, "None");
  loadObjectInfo(object_info_path_);
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
    // transform pointcloud to the given frame_id
    preprocessPointCloud(pointcloud_msg_);
    scene_segmentation_ros_->addCloudAccumulation(cloud_);  // Extend this to work with multiple viewpoints
    recognizeCloudAndImage();
    double end_time = ros::Time::now().toSec();
    ROS_INFO_STREAM("Total processing time: "<< end_time - start_time);

    // Reset received recognized cloud and image
    received_recognized_cloud_list_flag_ = false;
    received_recognized_image_list_flag_ = false;

    // reset object id
    rgb_object_id_ = 100;
    scene_segmentation_ros_->resetPclObjectId();

    // clear recognized image and cloud list
    recognized_image_list_.objects.clear();
    recognized_cloud_list_.objects.clear();

    scene_segmentation_ros_->resetCloudAccumulation();
    // pub e_done
    std_msgs::String event_out;
    event_out.data = "e_done";
    pub_event_out_.publish(event_out);
  }
}

void MultimodalObjectRecognitionROS::preprocessPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  sensor_msgs::PointCloud2 msg_transformed;
  msg_transformed.header.frame_id = target_frame_id_;
  if (!mpu::pointcloud::transformPointCloudMsg(tf_listener_, target_frame_id_, *cloud_msg, msg_transformed))
    return;

  pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(msg_transformed, *pc2);
  pc2->header.frame_id = msg_transformed.header.frame_id;

  cloud_ = PointCloud::Ptr(new PointCloud);
  pcl::fromPCLPointCloud2(*pc2, *cloud_);
}

void MultimodalObjectRecognitionROS::segmentPointCloud(mas_perception_msgs::ObjectList &object_list,
                             std::vector<PointCloud::Ptr> &clusters,
                             std::vector<mpu::object::BoundingBox> boxes)
{
  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id = target_frame_id_;
  scene_segmentation_ros_->getCloudAccumulation(cloud);

  // if the cluster is centered,it looses the correct location of the object
  scene_segmentation_ros_->segmentCloud(cloud, object_list, clusters, boxes,
                      center_cluster_ = false, pad_cluster_, padded_cluster_size_);

  // get workspace height
  std_msgs::Float64 workspace_height_msg;
  workspace_height_msg.data = scene_segmentation_ros_->getWorkspaceHeight();
  pub_workspace_height_.publish(workspace_height_msg);

  if (debug_mode_)
  {
    PointCloud::Ptr cloud_debug(new PointCloud);
    cloud_debug = scene_segmentation_ros_->getCloudDebug();
    sensor_msgs::PointCloud2 ros_pc2;
    pcl::toROSMsg(*cloud_debug, ros_pc2);
    ros_pc2.header.frame_id = target_frame_id_;
    pub_debug_cloud_plane_.publish(ros_pc2);
  }
}

void MultimodalObjectRecognitionROS::recognizeCloudAndImage()
{
  mas_perception_msgs::ObjectList cloud_object_list;
  std::vector<PointCloud::Ptr> clusters_3d;
  std::vector<mpu::object::BoundingBox> boxes;

  segmentPointCloud(cloud_object_list, clusters_3d, boxes);

  if (data_collection_)
  {
    std::string filename;
    for (auto& cluster : clusters_3d)
    {
      filename = "";
      filename.append("pcd_cluster_");
      filename.append(std::to_string(ros::Time::now().toSec()));
      mpu::object::savePcd(cluster, logdir_, filename);
      ROS_INFO_STREAM("\033[1;35mSaving point cloud to \033[0m" << logdir_);
    }
    return;
  }

  // Publish 3D object cluster for recognition
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
  // loop till it received the message from the 3d and rgb recognition
  int loop_rate_hz = 30;
  int timeout_wait = 2;  // secs
  ros::Rate loop_rate(loop_rate_hz);
  int loop_rate_count = 0;
  if (cloud_object_list.objects.size() > 0)
  {
    while (!received_recognized_cloud_list_flag_)
    {
      ROS_INFO_STREAM("[" << loop_rate_count << "] [Cloud] Waiting message from PCL recognizer node");
      loop_rate_count += 1;
      ros::spinOnce();
      loop_rate.sleep();
      if (received_recognized_cloud_list_flag_ == true)
      {
        ROS_WARN("[Cloud] Received %d objects from pcl recognizer", recognized_cloud_list_.objects.size());
      }
      if (loop_rate_count > loop_rate_hz * timeout_wait)
      {
        received_recognized_cloud_list_flag_ = false;
        ROS_ERROR("[Cloud] No message received from PCL recognizer. ");
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
  timeout_wait = 3;  //  secs
  if (image_list.images.size() > 0)
  {
    while (!received_recognized_image_list_flag_)
    {
      ROS_INFO_STREAM("[" << loop_rate_count << "] [RGB] Waiting message from RGB recognizer node");
      loop_rate_count += 1;
      ros::spinOnce();
      loop_rate.sleep();
      if (received_recognized_image_list_flag_ == true)
      {
        ROS_WARN("[RGB] Received %d objects from rgb recognizer", (int)(recognized_image_list_.objects.size()));
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

    for (int i = 0; i < recognized_image_list_.objects.size(); i++)
    {
      mas_perception_msgs::Object object = recognized_image_list_.objects[i];
      // Check qualitative info of the object
      if (round_objects_.count(recognized_image_list_.objects[i].name))
      {
        object.shape.shape = object.shape.SPHERE;
      }
      else
      {
        object.shape.shape = object.shape.OTHER;
      }
      // Get ROI
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
        cv::rectangle(cv_image->image, pt1, pt2, cv::Scalar(0, 255, 0), 1, 8, 0);
        // add label
        cv::putText(cv_image->image, object.name, cv::Point(pt1.x, pt2.y),
              cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
      }
      // Remove large 2d misdetected bbox (misdetection)
      double len_diag = sqrt(powf(((roi_2d.width + roi_2d.width) >> 1), 2));
      if (len_diag > rgb_bbox_min_diag_ && len_diag < rgb_bbox_max_diag_)
      {
        PointCloud::Ptr cloud_roi(new PointCloud);
        // get3DObject(roi_2d, cloud_, pcl_object_cluster);
        mpu::pointcloud::getPointCloudROI(roi_2d, cloud_, cloud_roi, rgb_roi_adjustment_, true);
        // ToDo: Filter big objects from 2d proposal, if the height is less than 3 mm
        // pcl::PointXYZRGB min_pt;
        // pcl::PointXYZRGB max_pt;
        // pcl::getMinMax3D(*cloud_roi, min_pt, max_pt);
        // float obj_height = max_pt.z - scene_segmentation_ros_->getWorkspaceHeight();

        sensor_msgs::PointCloud2 ros_pc2;
        pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cloud_roi, *pc2);
        pcl_conversions::fromPCL(*pc2, ros_pc2);
        ros_pc2.header.frame_id = target_frame_id_;
        ros_pc2.header.stamp = ros::Time::now();

        rgb_object_list.objects[i].views.resize(1);
        rgb_object_list.objects[i].views[0].point_cloud = ros_pc2;

        clusters_2d.push_back(cloud_roi);
        // Get pose
        geometry_msgs::PoseStamped pose;
        mpu::object::estimatePose(cloud_roi, pose, object.shape.shape,
                      rgb_cluster_filter_limit_min_, rgb_cluster_filter_limit_max_);

        // Transform pose
        std::string frame_id = cloud_->header.frame_id;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;
        if (frame_id != target_frame_id_)
        {
          mpu::object::transformPose(tf_listener_, target_frame_id_,
                         pose, rgb_object_list.objects[i].pose);
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
        ROS_INFO("[RGB] DECOY");
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
    for (int i = 0; i < combined_object_list.objects.size(); i++)
    {
      double current_object_pose_x = combined_object_list.objects[i].pose.pose.position.x;
      if (current_object_pose_x < roi_base_link_to_laser_distance_ ||
          current_object_pose_x > roi_max_object_pose_x_to_base_link_)
        /* combined_object_list.objects[i].pose.pose.position.z < scene_segmentation_ros_ */
        /* ->object_height_above_workspace_ - 0.05) */
      {
        ROS_WARN_STREAM("This object " << combined_object_list.objects[i].name << " out of RoI");
        combined_object_list.objects[i].name = "DECOY";
      }
    }
    // Adjust RPY to make pose flat, adjust container pose
    // Adjust Axis and Bolt pose
    adjustObjectPose(combined_object_list);
    // Publish object to object list merger
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

    ros::Time time_now = ros::Time::now();

    // Save debug image
    if(recognized_image_list_.objects.size() > 0)
    {
      std::string filename = "";
      filename.append("rgb_debug_");
      filename.append(std::to_string(time_now.toSec()));
      mpu::object::saveCVImage(cv_image, logdir_, filename);
      ROS_INFO_STREAM("Image:" << filename << " saved to " << logdir_);
    }
    else
    {
      ROS_WARN_STREAM("No Objects found. Cannot save debug image...");
    }
    // Save raw image
    cv_bridge::CvImagePtr raw_cv_image;
    if (mpu::object::getCVImage(image_msg_, raw_cv_image))
    {
      std::string filename = "";
      filename = "";
      filename.append("rgb_raw_");
      filename.append(std::to_string(time_now.toSec()));
      mpu::object::saveCVImage(raw_cv_image, logdir_, filename);
      ROS_INFO_STREAM("Image:" << filename << " saved to " << logdir_);
    }
    else
    {
      ROS_ERROR("Cannot generate cv image...");
    }

    // Save pointcloud debug
    for (auto& cluster : clusters_3d)
    {
      std::string filename = "";
      filename = "";
      filename.append("pcd_cluster_");
      filename.append(std::to_string(time_now.toSec()));
      mpu::object::savePcd(cluster, logdir_, filename);
      ROS_INFO_STREAM("Point cloud:" << filename << " saved to " << logdir_);
    }
  }
}

void MultimodalObjectRecognitionROS::publishDebug(mas_perception_msgs::ObjectList &combined_object_list,
                          std::vector<PointCloud::Ptr> &clusters_3d,
                          std::vector<PointCloud::Ptr> &clusters_2d)
{
  ROS_INFO_STREAM("Cloud list: " << recognized_cloud_list_.objects.size());
  ROS_INFO_STREAM("RGB list: " << recognized_image_list_.objects.size());
  ROS_INFO_STREAM("Combined object list: "<< combined_object_list.objects.size());
  // Compute normal to generate parallel BBOX to the plane
  const Eigen::Vector3f normal = scene_segmentation_ros_->getPlaneNormal();

  if (recognized_cloud_list_.objects.size() > 0)
  {
    // Bounding boxes
    if (clusters_3d.size() > 0)
    {
      mas_perception_msgs::BoundingBoxList bounding_boxes;
      cluster_visualizer_pc_.publish<PointT>(clusters_3d, target_frame_id_);
      bounding_boxes.bounding_boxes.resize(clusters_3d.size());
      for (int i=0; i < clusters_3d.size(); i++)
      {
        mpu::object::BoundingBox bbox;
        mpu::object::get3DBoundingBox(clusters_3d[i], normal, bbox, bounding_boxes.bounding_boxes[i]);
      }
      if (bounding_boxes.bounding_boxes.size() > 0)
      {
        bounding_box_visualizer_pc_.publish(bounding_boxes.bounding_boxes, target_frame_id_);
      }
    }
    // PCL Pose array for debug mode only
    geometry_msgs::PoseArray pcl_object_pose_array;
    pcl_object_pose_array.header.frame_id = target_frame_id_;
    pcl_object_pose_array.header.stamp = ros::Time::now();
    pcl_object_pose_array.poses.resize(recognized_cloud_list_.objects.size());
    std::vector<std::string> pcl_labels;
    int pcl_count = 0;
    for (int i=0; i < combined_object_list.objects.size(); i++)
    {
      if (combined_object_list.objects[i].database_id < 99)
      {
        ROS_INFO_STREAM("[Cloud] Objects: " << combined_object_list.objects[i].name);
        pcl_object_pose_array.poses[pcl_count] = combined_object_list.objects[i].pose.pose;
        pcl_labels.push_back(combined_object_list.objects[i].name);
        pcl_count++;
      }
    }
    // Publish pose array
    if (pcl_object_pose_array.poses.size() > 0)
    {
      pub_pc_object_pose_array_.publish(pcl_object_pose_array);
    }
    // Publish label visualizer
    if ((pcl_labels.size() == pcl_object_pose_array.poses.size()) &&
        (pcl_labels.size() > 0) && (pcl_object_pose_array.poses.size() > 0))
    {
      label_visualizer_pc_.publish(pcl_labels, pcl_object_pose_array);
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
    for (int i = 0; i < combined_object_list.objects.size(); i++)
    {
      if (combined_object_list.objects[i].database_id > 99)
      {
        ROS_INFO_STREAM("[RGB] Objects: "<< combined_object_list.objects[i].name);
        rgb_object_pose_array.poses[rgb_count] = combined_object_list.objects[i].pose.pose;
        rgb_labels.push_back(combined_object_list.objects[i].name);
        rgb_count++;
      }
    }
    // Publish pose array
    if (rgb_object_pose_array.poses.size() > 0)
    {
      pub_rgb_object_pose_array_.publish(rgb_object_pose_array);
    }
    // Publish label visualizer
    if ((rgb_labels.size() == rgb_object_pose_array.poses.size()) &&
        (rgb_labels.size() > 0) && (rgb_object_pose_array.poses.size() > 0))
    {
      label_visualizer_rgb_.publish(rgb_labels, rgb_object_pose_array);
    }
  }
}

void MultimodalObjectRecognitionROS::publishObjectList(mas_perception_msgs::ObjectList &object_list)
{
  for (int i = 0; i < object_list.objects.size(); i++)
  {
    // Empty cloud
    sensor_msgs::PointCloud2 empty_ros_cloud;
    object_list.objects[i].views[0].point_cloud = empty_ros_cloud;
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

void MultimodalObjectRecognitionROS::adjustObjectPose(mas_perception_msgs::ObjectList &object_list)
{
  for (int i = 0; i < object_list.objects.size(); i++)
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
    if (round_objects_.count(object_list.objects[i].name))
    {
      yaw = 0.0;
    }
    // Update container pose
    if (object_list.objects[i].name == "CONTAINER_BOX_RED" ||
      object_list.objects[i].name == "CONTAINER_BOX_BLUE")
    {
      if (object_list.objects[i].database_id > 100)
      {
        ROS_INFO_STREAM("Updating container pose");
        mm_object_recognition_utils_->adjustContainerPose(object_list.objects[i], rgb_container_height_);
      }
    }
    // Make pose flat
    tf::Quaternion q2 = tf::createQuaternionFromRPY(0.0, change_in_pitch , yaw);
    object_list.objects[i].pose.pose.orientation.x = q2.x();
    object_list.objects[i].pose.pose.orientation.y = q2.y();
    object_list.objects[i].pose.pose.orientation.z = q2.z();
    object_list.objects[i].pose.pose.orientation.w = q2.w();

    // Update workspace height
    if (scene_segmentation_ros_->getWorkspaceHeight() != -1000.0)
    {
      object_list.objects[i].pose.pose.position.z = scene_segmentation_ros_->getWorkspaceHeight() +
                              object_height_above_workspace_;
    }
    // Update axis or bolt pose
    if (object_list.objects[i].name == "M20_100" || object_list.objects[i].name == "AXIS")
    {
      mm_object_recognition_utils_->adjustAxisBoltPose(object_list.objects[i]);
    }
  }
}

void MultimodalObjectRecognitionROS::loadObjectInfo(const std::string &filename)
{
  if (boost::filesystem::is_regular_file(filename))
  {
    using boost::property_tree::ptree;
    mas_perception_msgs::Object object;
    ptree pt;
    read_xml(filename, pt);

    BOOST_FOREACH(ptree::value_type const& v, pt.get_child("object_info"))
    {
      if (v.first == "object") 
      {
        Object f;
        f.name = v.second.get<std::string>("name");
        f.shape = v.second.get<std::string>("shape");
        f.color = v.second.get<std::string>("color");
        if (f.shape == object.shape.SPHERE)
        {
          round_objects_.insert(f.name);
        }
        object_info_.push_back(f);
      }
    }
    ROS_INFO("Object info is loaded!");
  }
  else
  {
    ROS_WARN("No object info is provided!");
    return;
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
    scene_segmentation_ros_->resetCloudAccumulation();
    event_out.data = "e_stopped";
    pub_event_out_.publish(event_out);
  }
  else if (msg->data == "e_data_collection")
  {
    data_collection_ = true;
    event_out.data = "e_data_collection_started";
    pub_event_out_.publish(event_out);
    ROS_INFO_STREAM("\033[1;35mData collection enabled\033[0m");
  }
  else if (msg->data == "e_stop_data_collection")
  {
    data_collection_ = false;
    scene_segmentation_ros_->resetCloudAccumulation();
    event_out.data = "e_data_collection_stopped";
    pub_event_out_.publish(event_out);
    ROS_WARN_STREAM("\033[1;35mData collection disabled\033[0m");
  }
  else
  {
    return;
  }
}

void MultimodalObjectRecognitionROS::configCallback(mir_object_recognition::SceneSegmentationConfig &config, uint32_t level)
{
  scene_segmentation_ros_->setVoxelGridParams(config.voxel_leaf_size, config.voxel_filter_field_name,
      config.voxel_filter_limit_min, config.voxel_filter_limit_max);
  scene_segmentation_ros_->setPassthroughParams(config.enable_passthrough_filter,
      config.passthrough_filter_field_name,
      config.passthrough_filter_limit_min,
      config.passthrough_filter_limit_max);
  scene_segmentation_ros_->setNormalParams(config.normal_radius_search, config.use_omp, config.num_cores);
  Eigen::Vector3f axis(config.sac_x_axis, config.sac_y_axis, config.sac_z_axis);
  scene_segmentation_ros_->setSACParams(config.sac_max_iterations, config.sac_distance_threshold,
      config.sac_optimize_coefficients, axis, config.sac_eps_angle,
      config.sac_normal_distance_weight);
  scene_segmentation_ros_->setPrismParams(config.prism_min_height, config.prism_max_height);
  scene_segmentation_ros_->setOutlierParams(config.outlier_radius_search, config.outlier_min_neighbors);
  scene_segmentation_ros_->setClusterParams(config.cluster_tolerance, config.cluster_min_size, config.cluster_max_size,
      config.cluster_min_height, config.cluster_max_height, config.cluster_max_length,
      config.cluster_min_distance_to_polygon);
  // Cluster param
  center_cluster_ = config.center_cluster;
  pad_cluster_ = config.pad_cluster;
  padded_cluster_size_ = config.padded_cluster_size;
  // Workspace and object height
  object_height_above_workspace_ = config.object_height_above_workspace;
  rgb_container_height_ = config.rgb_container_height;
  // RGB proposal params
  rgb_roi_adjustment_ = config.rgb_roi_adjustment;
  rgb_bbox_min_diag_ = config.rgb_bbox_min_diag;
  rgb_bbox_max_diag_ = config.rgb_bbox_max_diag;
  rgb_cluster_filter_limit_min_ = config.rgb_cluster_filter_limit_min;
  rgb_cluster_filter_limit_max_ = config.rgb_cluster_filter_limit_max;
  // ROI params
  roi_base_link_to_laser_distance_ = config.roi_base_link_to_laser_distance;
  roi_max_object_pose_x_to_base_link_ = config.roi_max_object_pose_x_to_base_link;
  roi_min_bbox_z_ = config.roi_min_bbox_z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multimodal_object_recognition");
  ros::NodeHandle nh("~");
  // Initialize frame rate
  int frame_rate = 30;
  nh.param<int>("frame_rate", frame_rate, 30);
  ros::Rate loop_rate(frame_rate);
  // Create an object of multimodal object recognition
  MultimodalObjectRecognitionROS mm_object_recognition(nh);
  ROS_INFO_STREAM("\033[1;32m [multimodal_object_recognition] node started with rate "
                  << frame_rate << " \033[0m\n");
  // Run mm object recognition
  while (ros::ok())
  {
    mm_object_recognition.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
