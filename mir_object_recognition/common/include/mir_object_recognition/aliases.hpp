#ifndef MIR_PERCEPTION_UTILS_ALIASES_H
#define MIR_PERCEPTION_UTILS_ALIASES_H

#include <vector>

#include "pcl/geometry/planar_polygon.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/planar_region.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::Normal PointNT;
typedef pcl::Label PointLT;

typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<PointLT> PointCloudL;

typedef pcl::PlanarPolygon<PointT> PlanarPolygon;
typedef std::vector<PlanarPolygon, Eigen::aligned_allocator<PlanarPolygon>> PlanarPolygonVector;
typedef boost::shared_ptr<PlanarPolygon> PlanarPolygonPtr;
typedef boost::shared_ptr<const PlanarPolygon> PlanarPolygonConstPtr;

typedef pcl::PlanarRegion<PointT> PlanarRegion;
typedef std::vector<PlanarRegion, Eigen::aligned_allocator<PlanarRegion>> PlanarRegionVector;

#endif  // MIR_PERCEPTION_UTILS_ALIASES_H