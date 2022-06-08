/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.hpp>
#include <pcl_ros/transforms.hpp>

#include <std_msgs/msg/float64.hpp>

#include <mas_perception_msgs/msg/bounding_box.hpp>
#include <mas_perception_msgs/msg/bounding_box_list.hpp>
#include <mas_perception_msgs/msg/object_list.hpp>
#include <mas_perception_msgs/srv/recognize_object.hpp>

#include <mir_perception_utils/bounding_box.hpp>
#include <mir_perception_utils/bounding_box_visualizer.hpp>
#include <mir_perception_utils/clustered_point_cloud_visualizer.hpp>
#include <mir_perception_utils/label_visualizer.hpp>
#include <mir_perception_utils/pointcloud_utils_ros.hpp>

#include <mir_object_segmentation/scene_segmentation_node.hpp>

SceneSegmentationNode::SceneSegmentationNode()
    : nh_("~"),
      bounding_box_visualizer_("output/bounding_boxes", Color(Color::SEA_GREEN)),
      cluster_visualizer_("output/tabletop_clusters"),
      label_visualizer_("output/labels", Color(Color::TEAL)),
      add_to_octree_(false),
      object_id_(0),
      scene_segmentation_ros_(0.0025)
{
  sub_event_in_ = nh_.subscribe("event_in", 1, &SceneSegmentationNode::eventCallback, this);
  pub_event_out_ = nh_.advertise<std_msgs::msg::String>("event_out", 1);
  pub_object_list_ = nh_.advertise<mas_perception_msgs::msg::ObjectList>("output/object_list", 1);
  pub_workspace_height_ = nh_.advertise<std_msgs::msg::Float64>("output/workspace_height", 1);
  pub_debug_ = nh_.advertise<sensor_msgs::msg::PointCloud2>("output/debug_cloud", 1);

  dynamic_reconfigure::Server<mir_object_segmentation::SceneSegmentationConfig>::CallbackType f =
      boost::bind(&SceneSegmentationNode::configCallback, this, _1, _2);
  server_.setCallback(f);

  tf_listener_.reset(new tf::TransformListener);

  nh_.param<std::string>("logdir", logdir_, "/tmp/");
  nh_.param<std::string>("target_frame_id", target_frame_id_, "base_link");
}

SceneSegmentationNode::~SceneSegmentationNode() {}
void SceneSegmentationNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::Ptr &msg)
{
  if (add_to_octree_) {
    sensor_msgs::msg::PointCloud2 msg_transformed;
    if (!mpu::pointcloud::transformPointCloudMsg(tf_listener_, target_frame_id_, *msg,
                                                 msg_transformed))
      return;

    PointCloud::Ptr cloud = boost::make_shared<PointCloud>();
    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL(msg_transformed, pc2);
    pcl::fromPCLPointCloud2(pc2, *cloud);

    scene_segmentation_ros_.addCloudAccumulation(cloud);

    std_msgs::msg::String event_out;
    add_to_octree_ = false;
    event_out.data = "e_add_cloud_stopped";
    pub_event_out_.publish(event_out);
  }
}

void SceneSegmentationNode::segmentPointCloud()
{
  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id = target_frame_id_;
  scene_segmentation_ros_.getCloudAccumulation(cloud);

  std::vector<PointCloud::Ptr> clusters;
  mas_perception_msgs::msg::ObjectList object_list;
  std::vector<BoundingBox> boxes;
  scene_segmentation_ros_.segmentCloud(cloud, object_list, clusters, boxes, center_cluster_,
                                       pad_cluster_, padded_cluster_size_);

  mas_perception_msgs::msg::BoundingBoxList bounding_boxes;
  bounding_boxes.bounding_boxes.resize(clusters.size());

  geometry_msgs::msg::PoseArray poses;
  poses.header.stamp = rclcpp::Clock().now();
  poses.header.frame_id = target_frame_id_;
  std::vector<std::string> labels;

  for (int i = 0; i < object_list.objects.size(); i++) {
    mpu::object::convertBboxToMsg(boxes[i], bounding_boxes.bounding_boxes[i]);
    labels.push_back(object_list.objects[i].name);
    // make the object height dynamically reconfigurable
    object_list.objects[i].pose.pose.position.z += object_height_above_workspace_;
    poses.poses.push_back(object_list.objects[i].pose.pose);
  }
  ROS_INFO_STREAM("Publishing object list and workspace height");
  pub_object_list_.publish(object_list);
  bounding_box_visualizer_.publish(bounding_boxes.bounding_boxes, target_frame_id_);
  cluster_visualizer_.publish<PointT>(clusters, target_frame_id_);
  label_visualizer_.publish(labels, poses);

  std_msgs::msg::Float64 workspace_height_msg;
  workspace_height_msg.data = scene_segmentation_ros_.getWorkspaceHeight();
  pub_workspace_height_.publish(workspace_height_msg);
}

void SceneSegmentationNode::findPlane()
{
  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id = target_frame_id_;
  scene_segmentation_ros_.getCloudAccumulation(cloud);

  PointCloud::Ptr cloud_debug(new PointCloud);
  scene_segmentation_ros_.findPlane(cloud, cloud_debug);
  cloud_debug->header.frame_id = cloud->header.frame_id;
  ROS_INFO_STREAM("Got plane and publishing workspace height");
  std_msgs::msg::Float64 workspace_height_msg;
  workspace_height_msg.data = scene_segmentation_ros_.getWorkspaceHeight();
  pub_workspace_height_.publish(workspace_height_msg);
  pub_debug_.publish(*cloud_debug);
}

void SceneSegmentationNode::eventCallback(const std_msgs::msg::String::ConstPtr &msg)
{
  std_msgs::msg::String event_out;
  if (msg->data == "e_start") {
    sub_cloud_ = nh_.subscribe("input", 1, &SceneSegmentationNode::pointcloudCallback, this);
    event_out.data = "e_started";
  } else if (msg->data == "e_add_cloud_start") {
    add_to_octree_ = true;
    return;
  } else if (msg->data == "e_add_cloud_stop") {
    add_to_octree_ = false;
    event_out.data = "e_add_cloud_stopped";
  } else if (msg->data == "e_find_plane") {
    findPlane();
    scene_segmentation_ros_.resetCloudAccumulation();
    event_out.data = "e_done";
  } else if (msg->data == "e_segment") {
    segmentPointCloud();
    scene_segmentation_ros_.resetCloudAccumulation();
    event_out.data = "e_done";
  } else if (msg->data == "e_reset") {
    scene_segmentation_ros_.resetCloudAccumulation();
    event_out.data = "e_reset";
  } else if (msg->data == "e_stop") {
    sub_cloud_.shutdown();
    scene_segmentation_ros_.resetCloudAccumulation();
    event_out.data = "e_stopped";
  } else {
    return;
  }
  pub_event_out_.publish(event_out);
}

void SceneSegmentationNode::configCallback(mir_object_segmentation::SceneSegmentationConfig &config,
                                           uint32_t level)
{
  scene_segmentation_ros_.setVoxelGridParams(config.voxel_leaf_size, config.voxel_filter_field_name,
                                             config.voxel_filter_limit_min,
                                             config.voxel_filter_limit_max);
  scene_segmentation_ros_.setPassthroughParams(
      config.enable_passthrough_filter, config.passthrough_filter_field_name,
      config.passthrough_filter_limit_min, config.passthrough_filter_limit_max);
  scene_segmentation_ros_.setNormalParams(config.normal_radius_search, config.use_omp,
                                          config.num_cores);
  Eigen::Vector3f axis(config.sac_x_axis, config.sac_y_axis, config.sac_z_axis);
  scene_segmentation_ros_.setSACParams(config.sac_max_iterations, config.sac_distance_threshold,
                                       config.sac_optimize_coefficients, axis, config.sac_eps_angle,
                                       config.sac_normal_distance_weight);
  scene_segmentation_ros_.setPrismParams(config.prism_min_height, config.prism_max_height);
  scene_segmentation_ros_.setOutlierParams(config.outlier_radius_search,
                                           config.outlier_min_neighbors);
  scene_segmentation_ros_.setClusterParams(config.cluster_tolerance, config.cluster_min_size,
                                           config.cluster_max_size, config.cluster_min_height,
                                           config.cluster_max_height, config.cluster_max_length,
                                           config.cluster_min_distance_to_polygon);

  center_cluster_ = config.center_cluster;
  pad_cluster_ = config.pad_cluster;
  padded_cluster_size_ = config.padded_cluster_size;

  octree_resolution_ = config.octree_resolution;
  object_height_above_workspace_ = config.object_height_above_workspace;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, "scene_segmentation_node");
  SceneSegmentationNode scene_seg;
  ROS_INFO_STREAM("\033[1;32m[scene_segmentation_node] node started \033[0m\n");
  rclcpp::spin();
  return 0;
}
