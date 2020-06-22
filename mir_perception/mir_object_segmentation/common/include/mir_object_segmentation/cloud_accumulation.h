/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Maintainer: Mohammad Wasil
 * Author: Sergey Alexandrov
 *
 */
#ifndef MIR_OBJECT_SEGMENTATION_CLOUD_ACCUMULATION_H
#define MIR_OBJECT_SEGMENTATION_CLOUD_ACCUMULATION_H

#include <mir_perception_utils/aliases.h>
#include <mir_perception_utils/octree_pointcloud_occupancy_colored.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <memory>

/** This class accumulates input point clouds in the occupancy octree with a
  * given spatial resolution. */
class CloudAccumulation {
 public:
  typedef std::unique_ptr<CloudAccumulation> UPtr;

  /** \brief Constructor
   * \param[in] Octree resolution
   * */
  explicit CloudAccumulation(double resolution = 0.0025);

  /** \brief Add point cloud to octree
   * \param[in] Point cloud
   * */
  void addCloud(const PointCloud::ConstPtr &cloud);
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

#endif  // MIR_OBJECT_SEGMENTATION_CLOUD_ACCUMULATION_H
