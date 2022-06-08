#ifndef MIR_PERCCEPTION_UTILS_OBJECT_UTILS_HPP
#define MIR_PERCCEPTION_UTILS_OBJECT_UTILS_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cv_bridge/cv_bridge.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


#include "mas_perception_msgs/msg/bounding_box.hpp"
#include "mas_perception_msgs/msg/object_list.hpp"
#include "mir_perception_utils/bounding_box.hpp"

namespace mir_perception_utils
{
    namespace object
    {
        /** \brief Compute 3D bounding box parallel with the plane
         * \param[in] Pointcloud pointer
         * \param[in] Plane normal
         * \param[Out] Generated bounding box
         * \param[out] Bounding box message
         * */
        void get3DBoundingBox(const PointCloud::ConstPtr &cloud,
                              const Eigen::Vector3f &normal,
                              BoundingBox &bbox,
                              mas_perception_msgs::msg::BoundingBox &bounding_box_msg);

        /** \brief Estimate pose given bounding box
         * \param[in] Bounding box
         * \param[out] Estimated pose
         * */
        void estimatePose(const BoundingBox &box, geometry_msgs::msg::PoseStamped &pose);

        /** \brief Estimate pose given a cluster generated from 3D proposal. Passthrough
         * filter
         *         is also applied to remove part of the table on the cluster. Returns
         * estimated pose.
         *         The orientation is calculated based on
         *         http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
         * \param[in] Point cloud cluster
         * \param[out] Estimated pose
         * \param[in] The label of the cluster (optional), and is used to filter round
         * objects
         * \param[in] Minimum value of the field z allowed
         * \param[in] Maximum value of the field z allowed
         */
        void estimatePose(const PointCloud::ConstPtr &xyz_input_cloud,
                          geometry_msgs::msg::PoseStamped &pose,
                          const std::string shape = "None",
                          const float passthrough_lim_min = 0.0060,
                          const float passthrough_lim_max = 0.0);

        /** \brief Transform pose
         * \param[in] Transform listener
         * \param[in] Target frame id
         * \param[in] Source pose stamped
         * \param[out] Transformed pose stamped
         * */
        void transformPose(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                           const std::string &target_frame,
                           const geometry_msgs::msg::PoseStamped &pose,
                           geometry_msgs::msg::PoseStamped &transformed_pose);

        /** \brief Convert boundingbox to bounding box ros msg
         * \param[in] BoundingBox bbox
         * \param[out] Bounding box message
         * */
        void convertBboxToMsg(const BoundingBox &bbox,
                              mas_perception_msgs::msg::BoundingBox &bounding_box_msg);

        bool getCVImage(const std::shared_ptr<sensor_msgs::msg::Image> &image,
                        cv_bridge::CvImagePtr &cv_image);
    }
}

#endif // MIR_PERCCEPTION_UTILS_OBJECT_UTILS_HPP