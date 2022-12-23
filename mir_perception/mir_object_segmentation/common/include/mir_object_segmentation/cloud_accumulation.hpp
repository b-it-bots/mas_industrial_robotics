/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Maintainer: Mohammad Wasil
 * Author: Sergey Alexandrov
 * ROS2 contributor: Hamsa Datta Perur
 *
 */

#ifndef MIR_OBJECT_SEGMENTATION_CLOUD_ACCUMULATION_HPP
#define MIR_OBJECT_SEGMENTATION_CLOUD_ACCUMULATION_HPP

#include <mir_perception_utils/aliases.hpp>
#include <mir_perception_utils/octree_pointcloud_occupancy_colored.hpp>
// #include <pcl/octree/octree2buf_base.h>
#include <memory>

/** This class accumulates input point clouds in the occupancy octree with a
  * given spatial resolution. */
class CloudAccumulation
{
 public:
  typedef std::unique_ptr<CloudAccumulation> UPtr;

  /** \brief Constructor
   * \param[in] Octree resolution
   * */
  explicit CloudAccumulation(double resolution = 0.0025);

  /** \brief Add point cloud to octree
   * \param[in] Point cloud
   * */
  void addCloud(const PointCloudConstBSPtr &cloud);
  /** \brief Get accumulated cloud
   * \param[out] Accumulated point cloud
   * */
  void getAccumulatedCloud(PointCloud &cloud);
  /** \brief Return cloud count */
  int getCloudCount() const { return cloud_count_; }
  /** \brief Reset octree and cloud count */
  void reset();

 private:
  typedef OctreePointCloudOccupancyColored<PointT> Octree;
  typedef std::unique_ptr<Octree> OctreeUPtr;

  OctreeUPtr octree_;

  int cloud_count_;
  double resolution_;
};

#endif  // MIR_OBJECT_SEGMENTATION_CLOUD_ACCUMULATION_HPP
