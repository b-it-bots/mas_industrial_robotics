#ifndef EMPTY_SPACE_DETECTOR_H
#define EMPTY_SPACE_DETECTOR_H

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <mir_object_segmentation/cloud_accumulation.h>
#include <mir_object_segmentation/scene_segmentation.h>
#include <mir_perception_utils/pointcloud_utils_ros.h>

namespace mpu = mir_perception_utils;

class EmptySpaceDetector {
 public:
  EmptySpaceDetector();
  virtual ~EmptySpaceDetector();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  ros::Subscriber event_in_sub_;
  ros::Publisher pc_pub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher event_out_pub_;

  std::string output_frame_;
  bool enable_debug_pc_pub_;
  bool add_to_octree_;
  float empty_space_radius_;
  float expected_num_of_points_;
  float empty_space_pnt_cnt_perc_thresh_;
  bool find_empty_spaces_;
  /* int retry_attempts_; */
  /* int num_of_retries_; */
  boost::shared_ptr<tf::TransformListener> tf_listener_;

  typedef std::shared_ptr<SceneSegmentation> SceneSegmentationSPtr;
  SceneSegmentationSPtr scene_segmentation_;
  CloudAccumulation::UPtr cloud_accumulation_;

  pcl::KdTreeFLANN<PointT> kdtree_;
  pcl::ExtractIndices<PointT> extract_indices_;

  void pcCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void eventInCallback(const std_msgs::String::ConstPtr &msg);
  void loadParams();
  bool findEmptySpaces();
  bool findPlane(PointCloud::Ptr plane);
  void findEmptySpacesOnPlane(const PointCloud::Ptr &plane,
                              geometry_msgs::PoseArray &empty_space_poses);
};
#endif
