/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 *
 */

#include <pcl/octree/octree_impl.h>

#include <mir_object_segmentation/cloud_accumulation.hpp>

CloudAccumulation::CloudAccumulation(double resolution) : resolution_(resolution) { reset(); }
void CloudAccumulation::addCloud(const PointCloudConstBSPtr &cloud)
{
  octree_->setOccupiedVoxelsAtPointsFromCloud(cloud);
  cloud_count_++;
}

void CloudAccumulation::getAccumulatedCloud(PointCloud &cloud)
{
  octree_->getOccupiedVoxelCentersWithColor(cloud.points);
  cloud.width = static_cast<uint32_t>(cloud.points.size());
  cloud.height = 1;
}

void CloudAccumulation::reset()
{
  octree_ = OctreeUPtr(new Octree(resolution_));
  cloud_count_ = 0;
}
