/*
 * Copyright 2020 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */
#include <mir_perception_utils/pointcloud_utils_ros.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>

using namespace mir_perception_utils;

bool pointcloud::transformPointCloudMsg(const boost::shared_ptr<tf::TransformListener> &tf_listener,
                                        const std::string &target_frame,
                                        const sensor_msgs::PointCloud2 &cloud_in,
                                        sensor_msgs::PointCloud2 &cloud_out)
{
  if (tf_listener) {
    try {
      ros::Time common_time;
      tf_listener->getLatestCommonTime(target_frame, cloud_in.header.frame_id, common_time, NULL);
      cloud_in.header.stamp = common_time;
      tf_listener->waitForTransform(target_frame, cloud_in.header.frame_id, ros::Time::now(),
                                    ros::Duration(1.0));
      pcl_ros::transformPointCloud(target_frame, cloud_in, cloud_out, *tf_listener);
      cloud_out.header.frame_id = target_frame;
    } catch (tf::TransformException &ex) {
      ROS_ERROR("PCL transform error: %s", ex.what());
      return (false);
    }
  } else {
    ROS_ERROR_THROTTLE(2.0, "TF listener not initialized.");
    return (false);
  }
  return (true);
}

bool pointcloud::transformPointCloud(const boost::shared_ptr<tf::TransformListener> &tf_listener,
                                     const std::string &target_frame, const PointCloud &cloud_in,
                                     PointCloud &cloud_out)
{
  if (tf_listener) {
    try {
      ros::Time common_time;
      tf_listener->getLatestCommonTime(target_frame, cloud_in.header.frame_id, common_time, NULL);
      pcl::PCLHeader pc_header;
      pc_header.frame_id = cloud_in.header.frame_id;
      pc_header.seq = cloud_in.header.seq;
      pcl_conversions::toPCL(common_time, pc_header.stamp);
      cloud_in.header = pc_header;
      tf_listener->waitForTransform(target_frame, cloud_in.header.frame_id, ros::Time::now(),
                                    ros::Duration(1.0));
      pcl_ros::transformPointCloud(target_frame, cloud_in, cloud_out, *tf_listener);
      cloud_out.header.frame_id = target_frame;
    } catch (tf::TransformException &ex) {
      ROS_ERROR("PCL transform error: %s", ex.what());
      return (false);
    }
  } else {
    ROS_ERROR_THROTTLE(2.0, "TF listener not initialized.");
    return (false);
  }
  return (true);
}

bool pointcloud::transformPointCloud2(const boost::shared_ptr<tf::TransformListener> &tf_listener,
                                      const std::string &target_frame,
                                      const pcl::PCLPointCloud2 &cloud_in_pc2,
                                      pcl::PCLPointCloud2 &cloud_out_pc2)
{
  if (tf_listener) {
    try {
      PointCloud cloud_in;
      pcl::fromPCLPointCloud2(cloud_in_pc2, cloud_in);
      PointCloud cloud_out;

      ros::Time common_time;
      tf_listener->getLatestCommonTime(target_frame, cloud_in.header.frame_id, common_time, NULL);
      pcl::PCLHeader pc_header;
      pc_header.frame_id = cloud_in.header.frame_id;
      pc_header.seq = cloud_in.header.seq;
      pcl_conversions::toPCL(common_time, pc_header.stamp);
      cloud_in.header = pc_header;
      tf_listener->waitForTransform(target_frame, cloud_in.header.frame_id, ros::Time::now(),
                                    ros::Duration(1.0));
      pcl_ros::transformPointCloud(target_frame, cloud_in, cloud_out, *tf_listener);

      cloud_out.header.frame_id = target_frame;
      pcl::toPCLPointCloud2(cloud_out, cloud_out_pc2);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("PCL transform error: %s", ex.what());
      return (false);
    }
  } else {
    ROS_ERROR_THROTTLE(2.0, "TF listener not initialized.");
    return (false);
  }
  return (true);
}

bool pointcloud::getPointCloudROI(const sensor_msgs::RegionOfInterest &roi,
                                  const PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_roi,
                                  float roi_size_adjustment, bool remove_outliers)
{
  if (cloud_in->height <= 1 || cloud_in->width <= 1) {
    ROS_ERROR("Pointcloud input height is %d and width is %d",cloud_in->height, cloud_in->width );
    return (false);
  }
  int min_x = roi.x_offset;
  int min_y = roi.y_offset;
  int max_x = roi.x_offset + roi.width;
  int max_y = roi.y_offset + roi.height;
  // Adjust roi
  if (roi.x_offset > roi_size_adjustment) min_x = min_x - roi_size_adjustment;
  if (roi.y_offset > roi_size_adjustment) min_y = min_y - roi_size_adjustment;
  if (roi.width + roi_size_adjustment < cloud_in->width) min_x = min_x + roi_size_adjustment;
  if (roi.height + roi_size_adjustment < cloud_in->height) min_y = min_y + roi_size_adjustment;

  std::vector<cv::Point> pixel_loc;

  for (int i = min_x; i < max_x; i++) {
    for (int j = min_y; j < max_y; j++) {
      cv::Point loc;
      loc.x = i;
      loc.y = j;
      pixel_loc.push_back(loc);
    }
  }
  for (size_t i = 0; i < pixel_loc.size(); i++) {
    PointT pcl_point = cloud_in->at(pixel_loc[i].x, pixel_loc[i].y);
    if ((!pcl_isnan(pcl_point.x)) && (!pcl_isnan(pcl_point.y)) && (!pcl_isnan(pcl_point.z)) &&
        (!pcl_isnan(pcl_point.r)) && (!pcl_isnan(pcl_point.g)) && (!pcl_isnan(pcl_point.b))) {
      cloud_roi->points.push_back(pcl_point);
    }
  }
  cloud_roi->header = cloud_in->header;
  if (remove_outliers) {
    if (cloud_roi->points.size() > 0) {
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(cloud_roi);
      sor.setMeanK(50);
      sor.setStddevMulThresh(3.0);
      sor.filter(*cloud_roi);
    }
  }
  return (true);
}
