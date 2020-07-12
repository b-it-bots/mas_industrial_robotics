/*
 * Copyright 2020 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */

#ifndef MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_ROS_H
#define MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_ROS_H

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

#include <mir_perception_utils/aliases.h>
#include <sensor_msgs/RegionOfInterest.h>

namespace mir_perception_utils
{
namespace pointcloud
{
/** \brief Transform sensor_msgs PointCloud2
* \param[in] Transform listener
* \param[in] Target frame id
* \param[in] sensor_msgs PointCloud2 input
* \param[in] sensor_msgs PointCloud2 output
*/
bool transformPointCloudMsg(const boost::shared_ptr<tf::TransformListener> &tf_listener,
                            const std::string &target_frame,
                            const sensor_msgs::PointCloud2 &cloud_in,
                            sensor_msgs::PointCloud2 &cloud_out);

/** \brief Transform pcl PointCloud
 * \param[in] Transform listener
 * \param[in] Target frame id
 * \param[in] pcl PointCloud input
 * \param[in] pcl PointCloud output
*/
bool transformPointCloud(const boost::shared_ptr<tf::TransformListener> &tf_listener,
                         const std::string &target_frame, const PointCloud &cloud_in,
                         PointCloud &cloud_out);

/** \brief Transform pcl PointCloud2
 * \param[in] Transform listener
 * \param[in] Target frame id
 * \param[in] pcl PointCloud2 input
 * \param[in] pcl PointCloud2 output
*/
bool transformPointCloud2(const boost::shared_ptr<tf::TransformListener> &tf_listener,
                          const std::string &target_frame, const pcl::PCLPointCloud2 &cloud_in_pc2,
                          pcl::PCLPointCloud2 &cloud_out_pc2);

/** \brief Get 3D ROI of point cloud given 2D ROI
 * \param[in] Region of interest (bounding box) of 2D object
 * \param[in] Organized pointcloud input
 * \param[out] 3D pointcloud cluster (3D ROI) of the given 2D ROI
 * \param[in] Adjust the rgb roi proposal (in pixel)
 * \param[in] Remove 3D ROI outliers
*/
void getPointCloudROI(const sensor_msgs::RegionOfInterest &roi, const PointCloud::Ptr &cloud_id,
                      PointCloud::Ptr &cloud_roi, float roi_size_adjustment, bool remove_outliers);
}
};

#endif  // MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_ROS_H
