#ifndef MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_ROS_HPP
#define MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_ROS_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// pcl_ros is not converted to ros2 yet, hence commented out
// #include <pcl_ros/point_cloud.hpp>

#include <pcl_ros/transforms.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include "mir_perception_utils/aliases.hpp"
#include <sensor_msgs/msg/region_of_interest.hpp>

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
        bool transformPointCloudMsg(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                                   const std::string &target_frame,
                                   const sensor_msgs::msg::PointCloud2 &cloud_in,
                                   sensor_msgs::msg::PointCloud2 &cloud_out);

        /** \brief Transform pcl PointCloud
         * \param[in] Transform listener
         * \param[in] Target frame id
         * \param[in] pcl PointCloud input
         * \param[in] pcl PointCloud output
        */
       bool transformPointCloud(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                                const std::string &target_frame,
                                const PointCloud &cloud_in,
                                PointCloud &cloud_out);

        /** \brief Transform pcl PointCloud2
         * \param[in] Transform listener
         * \param[in] Target frame id
         * \param[in] pcl PointCloud2 input
         * \param[in] pcl PointCloud2 output
        */
       bool transformPointCloud2(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                                 const std::string &target_frame,
                                 const pcl::PCLPointCloud2 &cloud_in_pc2,
                                 pcl::PCLPointCloud2 &cloud_out_pc2);

        /** \brief Get 3D ROI of point cloud given 2D ROI
         * \param[in] Region of interest (bounding box) of 2D object
         * \param[in] Organized pointcloud input
         * \param[out] 3D pointcloud cluster (3D ROI) of the given 2D ROI
         * \param[in] Adjust the rgb roi proposal (in pixel)
         * \param[in] Remove 3D ROI outliers
        */
       bool getPointCloudROI(const sensor_msgs::msg::RegionOfInterest &roi,
                             const PointCloudBSPtr &cloud_in,
                             PointCloudBSPtr &cloud_roi,
                             float roi_size_adjustment, bool remove_outliers);

    }
}
#endif // MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_ROS_HPP