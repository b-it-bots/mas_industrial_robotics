/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#include <iostream>
#include <string>
#include <vector>

#include <mir_object_segmentation/scene_segmentation.h>

SceneSegmentation::SceneSegmentation() : use_omp_(false)
{
  cluster_extraction_.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT>>());
  normal_estimation_.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT>>());
  normal_estimation_omp_.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT>>());
};
SceneSegmentation::~SceneSegmentation(){

};

PointCloud::Ptr SceneSegmentation::segmentScene(const PointCloud::ConstPtr &cloud,
                                                std::vector<PointCloud::Ptr> &clusters,
                                                std::vector<BoundingBox> &boxes,
                                                pcl::ModelCoefficients::Ptr &coefficients,
                                                double &workspace_height)
{
  PointCloud::Ptr filtered(new PointCloud);
  PointCloud::Ptr plane(new PointCloud);
  PointCloud::Ptr hull(new PointCloud);
  pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);
  std::vector<pcl::PointIndices> clusters_indices;

  filtered = findPlane(cloud, hull, plane, coefficients, workspace_height);

  if (coefficients->values.size() == 0) {
    return filtered;
  }

  extract_polygonal_prism_.setInputPlanarHull(hull);
  extract_polygonal_prism_.setInputCloud(cloud);
  extract_polygonal_prism_.setViewPoint(0.0, 0.0, 2.0);
  extract_polygonal_prism_.segment(*segmented_cloud_inliers);

  cluster_extraction_.setInputCloud(cloud);
  cluster_extraction_.setIndices(segmented_cloud_inliers);
  cluster_extraction_.extract(clusters_indices);

  const Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1],
                               coefficients->values[2]);

  for (size_t i = 0; i < clusters_indices.size(); i++) {
    const pcl::PointIndices &cluster_indices = clusters_indices[i];
    PointCloud::Ptr cluster(new PointCloud);
    pcl::copyPointCloud(*cloud, cluster_indices, *cluster);
    clusters.push_back(cluster);
    BoundingBox box = BoundingBox::create(cluster->points, normal);
    boxes.push_back(box);
  }
  return filtered;
}

PointCloud::Ptr SceneSegmentation::findPlane(const PointCloud::ConstPtr &cloud,
                                             PointCloud::Ptr &hull, PointCloud::Ptr &plane,
                                             pcl::ModelCoefficients::Ptr &coefficients,
                                             double &workspace_height)
{
  PointCloud::Ptr filtered(new PointCloud);
  pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);

  PointCloudN::Ptr normals(new PointCloudN);

  voxel_grid_.setInputCloud(cloud);
  voxel_grid_.filter(*filtered);

  if (enable_passthrough_filter_) {
    pass_through_.setInputCloud(filtered);
    pass_through_.filter(*filtered);
  }

  if (use_omp_) {
    normal_estimation_omp_.setInputCloud(filtered);
    normal_estimation_omp_.compute(*normals);
  } else {
    normal_estimation_.setInputCloud(filtered);
    normal_estimation_.compute(*normals);
  }

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  sac_.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
  sac_.setMethodType(pcl::SAC_RANSAC);

  sac_.setInputCloud(filtered);
  sac_.setInputNormals(normals);
  sac_.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    std::cout << "No plane inliers found " << std::endl;
    return filtered;
  }

  project_inliers_.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
  project_inliers_.setInputCloud(filtered);
  project_inliers_.setModelCoefficients(coefficients);
  project_inliers_.setIndices(inliers);
  project_inliers_.setCopyAllData(false);
  project_inliers_.filter(*plane);

  convex_hull_.setInputCloud(plane);
  convex_hull_.reconstruct(*hull);

  // not sure if this is necessary
  hull->points.push_back(hull->points.front());
  hull->width += 1;
  double z = 0.0;
  for (int i = 0; i < hull->points.size(); i++) {
    z += hull->points[i].z;
  }
  if (hull->points.size() > 0) {
    z /= hull->points.size();
  }
  workspace_height = z;

  return filtered;
}

void SceneSegmentation::setVoxelGridParams(double leaf_size, const std::string &filter_field,
                                           double limit_min, double limit_max)
{
  voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid_.setFilterFieldName(filter_field);
  voxel_grid_.setFilterLimits(limit_min, limit_max);
}

void SceneSegmentation::setPassthroughParams(bool enable_passthrough_filter,
                                             const std::string &field_name, double limit_min,
                                             double limit_max)
{
  enable_passthrough_filter_ = enable_passthrough_filter;
  pass_through_.setFilterFieldName(field_name);
  pass_through_.setFilterLimits(limit_min, limit_max);
}

void SceneSegmentation::setNormalParams(double radius_search, bool use_omp, int num_cores)
{
  use_omp_ = use_omp;
  if (use_omp_) {
    normal_estimation_omp_.setRadiusSearch(radius_search);
    normal_estimation_omp_.setNumberOfThreads(num_cores);
  } else {
    normal_estimation_.setRadiusSearch(radius_search);
  }
}
void SceneSegmentation::setSACParams(int max_iterations, double distance_threshold,
                                     bool optimize_coefficients, Eigen::Vector3f axis,
                                     double eps_angle, double normal_distance_weight)
{
  sac_.setMaxIterations(max_iterations);
  sac_.setDistanceThreshold(distance_threshold);
  sac_.setAxis(axis);
  sac_.setEpsAngle(eps_angle);
  sac_.setOptimizeCoefficients(optimize_coefficients);
  sac_.setNormalDistanceWeight(normal_distance_weight);
}
void SceneSegmentation::setPrismParams(double min_height, double max_height)
{
  extract_polygonal_prism_.setHeightLimits(min_height, max_height);
}

void SceneSegmentation::setOutlierParams(double radius_search, int min_neighbors)
{
  radius_outlier_.setRadiusSearch(radius_search);
  radius_outlier_.setMinNeighborsInRadius(min_neighbors);
}
void SceneSegmentation::setClusterParams(double cluster_tolerance, int cluster_min_size,
                                         int cluster_max_size, double cluster_min_height,
                                         double cluster_max_height, double max_length,
                                         double cluster_min_distance_to_polygon)
{
  cluster_extraction_.setClusterTolerance(cluster_tolerance);
  cluster_extraction_.setMinClusterSize(cluster_min_size);
  cluster_extraction_.setMaxClusterSize(cluster_max_size);
}
