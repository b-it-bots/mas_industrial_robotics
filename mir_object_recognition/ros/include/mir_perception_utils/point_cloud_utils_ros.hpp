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
       bool transformPointCloudMsg(const tf2_ros::Buffer &tf_buffer,
                                   const std::string &target_frame,
                                   const sensor_msgs::msg::PointCloud2 &cloud_in,
                                   sensor_msgs::msg::PointCloud2 &cloud_out);

        
    }
}
#endif // MIR_PERCCEPTION_UTILS_POINTCLOUD_UTILS_ROS_HPP