/*
 * Copyright 2020 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */

#ifndef MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_H
#define MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_H

#include <mir_perception_utils/aliases.h>

namespace mir_perception_utils
{
namespace pointcloud
{
/** \brief Center PointCloud, adapted from pcl library group_common
 * https://pointcloudlibrary.github.io/documentation/group__common.html
 * \param[in] PointCloud input
 * \param[out] Centered PointCloud output
 * \return The number of valid points, if the cloud is dense, it's the same
 * as the number of input points
 */
unsigned int centerPointCloud(const PointCloud &cloud_in, PointCloud &centered_cloud);
/** \brief Pad point cloud
  * \param[in] Normalized PointCloud input
  * \param[in] Number of points
  * \return The number of padded points
  */
unsigned int padPointCloud(PointCloud::Ptr &cloud_in, int num_points);
}
};

#endif  // MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_H
