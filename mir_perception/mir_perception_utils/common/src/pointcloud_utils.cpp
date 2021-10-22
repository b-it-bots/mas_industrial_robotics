/*
 * Copyright 2020 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */
#include <random>
#include <mir_perception_utils/pointcloud_utils.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>

using namespace mir_perception_utils;

unsigned int pointcloud::centerPointCloud(const PointCloud &cloud_in, PointCloud &centered_cloud)
{
  if (cloud_in.empty()) return (0);

  pcl::copyPointCloud(cloud_in, centered_cloud);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(centered_cloud, centroid);

  unsigned int point_count;
  // Check if the data is dense, means there may be NaN or Inf values
  if (centered_cloud.is_dense) {
    point_count = static_cast<unsigned int>(centered_cloud.points.size());
    // For each point in the cloud
    for (auto &point : centered_cloud) {
      point.x = point.x - centroid[0];
      point.y = point.y - centroid[1];
      point.z = point.z - centroid[2];
    }
  }
  // Check NaN or Inf values, that could exist
  else {
    point_count = 0;
    for (auto &point : centered_cloud) {
      // Check if the point is invalid
      if (!isFinite(point)) continue;

      point.x = point.x - centroid[0];
      point.y = point.y - centroid[1];
      point.z = point.z - centroid[2];
      ++point_count;
    }
  }
  return (point_count);
}

unsigned int pointcloud::padPointCloud(PointCloud::Ptr &cloud_in, int num_points)
{
  if (cloud_in->empty()) return (0);

  unsigned int point_count;
  point_count = static_cast<unsigned int>(cloud_in->size());

  std::random_device rd;   // get ran
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<> distr(0, point_count);

  if (point_count > num_points) {
    unsigned int point_diff = point_count - num_points;

    pcl::PointIndices::Ptr random_indices(new pcl::PointIndices());

    for (int n = 0; n < num_points; ++n) random_indices->indices.push_back(distr(eng));

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(random_indices);
    extract.setNegative(false);
    extract.filter(*cloud_in);
    cloud_in->width = cloud_in->points.size();
  } else if (point_count < num_points) {
    int additional_points = num_points - point_count;
    for (int i = 0; i < additional_points; i++) {
      int random_index = distr(eng);
      cloud_in->points.push_back(cloud_in->points[random_index]);
    }
    point_count += additional_points;
    cloud_in->width = cloud_in->points.size();
  }
  return (point_count);
}
