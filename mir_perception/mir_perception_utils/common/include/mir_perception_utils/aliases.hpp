/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 * ROS2 contributors: Hamsa Datta Perur, Vamsi Kalagaturu.
 *
 */

#ifndef MIR_PERCEPTION_UTILS_ALIASES_HPP
#define MIR_PERCEPTION_UTILS_ALIASES_HPP

#include <chrono>
#include <vector>
#include <memory>

#include "pcl/geometry/planar_polygon.h"
#include "pcl/point_cloud.h"
#include <pcl/PCLPointCloud2.h>
#include "pcl/point_types.h"
#include "pcl/segmentation/planar_region.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::Normal PointNT;
typedef pcl::Label PointLT;

typedef pcl::PointCloud<PointT> PointCloud;
typedef std::shared_ptr<PointCloud> PointCloudSPtr;
typedef std::shared_ptr<const PointCloud> PointCloudConstSPtr;
typedef PointCloud::Ptr PointCloudBSPtr;
typedef PointCloud::ConstPtr PointCloudConstBSPtr;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<PointLT> PointCloudL;
typedef pcl::PCLPointCloud2::Ptr PCLPointCloud2BSPtr;

typedef pcl::PlanarPolygon<PointT> PlanarPolygon;
typedef std::vector<PlanarPolygon, Eigen::aligned_allocator<PlanarPolygon>> PlanarPolygonVector;
typedef std::shared_ptr<PlanarPolygon> PlanarPolygonPtr;
typedef std::shared_ptr<const PlanarPolygon> PlanarPolygonConstPtr;

typedef pcl::PlanarRegion<PointT> PlanarRegion;
typedef std::vector<PlanarRegion, Eigen::aligned_allocator<PlanarRegion>> PlanarRegionVector;

#endif  // MIR_PERCEPTION_UTILS_ALIASES_HPP