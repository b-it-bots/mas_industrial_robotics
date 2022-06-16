/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

#include "mir_object_recognition/multimodal_object_recognition_utils.hpp"

MultimodalObjectRecognitionUtils::MultimodalObjectRecognitionUtils() {}
MultimodalObjectRecognitionUtils::~MultimodalObjectRecognitionUtils() {}
void MultimodalObjectRecognitionUtils::adjustContainerPose(mas_perception_msgs::msg::Object &container_object,
                               float container_height)
{
  // PointCloud::Ptr cloud(new PointCloud);
  PointCloudBSPtr cloud(new PointCloud);
  pcl::fromROSMsg(container_object.views[0].point_cloud, *cloud);
  // find min and max z
  pcl::PointXYZRGB min_pt;
  pcl::PointXYZRGB max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  RCLCPP_INFO(rclcpp::get_logger("mmor_utils_logger"), "Min and max z %f, %f", min_pt.z, max_pt.z);
  // ROS_INFO_STREAM("Min and max z " << min_pt.z << ", " << max_pt.z);
  // estimate normal
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);
  // filter points by height
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_pt.z, (min_pt.z + (min_pt.z + max_pt.z/2))/2);
  pass.filter(*indices);
  // apply segmentation with region growing
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize(300);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(200);
  reg.setInputCloud(cloud);
  // reg.setIndices (indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);
  std::vector <pcl::PointIndices> clusters;
  reg.extract(clusters);
  // find the biggest cluster
  pcl::PointIndices::Ptr filtered_cluster(new pcl::PointIndices() );
  int largest_index, largest_cluster;
  if (clusters.size() == 0)
  {
    return;
  }
  for (size_t i = 0; i < clusters.size(); i++)
  {
    if (i == 0)
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
  PointCloud::Ptr cloud_filtered (new PointCloud);
  // point indices to cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> c_filter(true);
  c_filter.setInputCloud(cloud);
  c_filter.setIndices(filtered_cluster);
  c_filter.filter(*cloud_filtered);
  // publish selected cluster
  // PointCloud::Ptr colored_cloud = reg.getColoredCloud();
  // sensor_msgs::PointCloud2 ros_pointcloud;
  // pcl::PCLPointCloud2::Ptr cloud_cl(new pcl::PCLPointCloud2);
  // pcl::toPCLPointCloud2(*cloud_filtered, *cloud_cl);
  // pcl_conversions::fromPCL(*cloud_cl, ros_pointcloud);
  // ros_pointcloud.header.frame_id = target_frame_id_;
  // pub_pc_cluster_.publish(ros_pointcloud);
  // find centroid
  Eigen::Vector4f centroid;
  unsigned int valid_points = pcl::compute3DCentroid(*cloud_filtered, centroid);
  // Change the center of object
  container_object.pose.pose.position.x = centroid[0];
  container_object.pose.pose.position.y = centroid[1];
  container_object.pose.pose.position.z = max_pt.z + container_height;
}
void MultimodalObjectRecognitionUtils::adjustAxisBoltPose(mas_perception_msgs::msg::Object &object)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(object.views[0].point_cloud, *xyz_cloud);
  int pcl_point_size = object.views[0].point_cloud.height * object.views[0].point_cloud.width;
  pcl::PointXYZ min_pt;
  pcl::PointXYZ max_pt;

  pcl::getMinMax3D(*xyz_cloud, min_pt, max_pt);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_at_z(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Vector4f centroid;
  for (size_t i = 0; i < pcl_point_size; i++ )
  {
    if (xyz_cloud->points[i].z == max_pt.z)
    {
      point_at_z->points.push_back(xyz_cloud->points[i]);
    }
  }
  unsigned int valid_points = pcl::compute3DCentroid(*point_at_z, centroid);
  if (object.name == "M20_100")
  {
    // ROS_INFO_STREAM("Updating M20_100 pose from object id: " << object.database_id);
    RCLCPP_INFO(rclcpp::get_logger("mmor_utils_logger"), "Updating M20_100 pose from object id: %d", object.database_id);
    float midpoint_x = (object.pose.pose.position.x + centroid[0])/2;
    float midpoint_y = (object.pose.pose.position.y + centroid[1])/2;
    object.pose.pose.position.x = midpoint_x;
    object.pose.pose.position.y = midpoint_y;
  }
  else if (object.name == "AXIS")
  {
    RCLCPP_INFO(rclcpp::get_logger("mmor_utils_logger"), "Updating AXIS pose from object id: %d", object.database_id);
    // ROS_INFO_STREAM("Updating AXIS pose from object id: " << object.database_id);
    object.pose.pose.position.x = centroid[0];
    object.pose.pose.position.y = centroid[1];
  }
}
