/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MIR_PERCCEPTION_UTILS_OBJECT_UTILS_H
#define MIR_PERCCEPTION_UTILS_OBJECT_UTILS_H

#include <string>
#include <Eigen/Dense>
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <mas_perception_msgs/BoundingBox.h>
#include <mas_perception_msgs/ObjectList.h>
#include <mir_perception_utils/bounding_box.h>

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
                            const Eigen::Vector3f& normal,
                            BoundingBox &bbox,
                            mas_perception_msgs::BoundingBox &bounding_box_msg);

    /** \brief Estimate pose given bounding box
     * \param[in] Bounding box
     * \param[out] Estimated pose
     * */
    void estimatePose(const BoundingBox &box,
                        geometry_msgs::PoseStamped &pose);

    /** \brief Estimate pose given a cluster generated from 3D proposal. Passthrough filter
     *         is also applied to remove part of the table on the cluster. Returns estimated pose.
     *         The orientation is calculated based on
     *         http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
     * \param[in] Point cloud cluster
     * \param[out] Estimated pose
     * \param[in] The label of the cluster (optional), and is used to filter round objects
     * \param[in] Minimum value of the field z allowed
     * \param[in] Maximum value of the field z allowed
    */
    void estimatePose(const PointCloud::Ptr &xyz_input_cloud,
                        geometry_msgs::PoseStamped &pose,
                        std::string shape = "None",
                        float passthrough_lim_min=0.0060,
                        float passthrough_lim_max=0.0);

    /** \brief Transform pose
     * \param[in] Transform listener
     * \param[in] Target frame id
     * \param[in] Source pose stamped
     * \param[out] Transformed pose stamped
     * */
    void transformPose(const boost::shared_ptr<tf::TransformListener> tf_listener,
                        const std::string &target_frame,
                        const geometry_msgs::PoseStamped &pose,
                        geometry_msgs::PoseStamped &transformed_pose);

    /** \brief Convert boundingbox to bounding box ros msg
     * \param[in] BoundingBox bbox
     * \param[out] Bounding box message
     * */
    void convertBboxToMsg(const BoundingBox &bbox,
                            mas_perception_msgs::BoundingBox& bounding_box_msg);

    /** \brief Save pointcloud
     * \param[in] logdir (default="/tmp")
     * \param[in] obj_name (default="unknown")
     * */
    void savePcd(const PointCloud::ConstPtr &pointcloud,
                 std::string log_dir="/tmp/",
                 std::string obj_name="unknown");

    /** \brief Save debug image if debug_mode is enabled
     * \param[in] image with boundix boxes of objects drawn
     * \param[in] raw_image
     * \param[in] logdir (default /tmp)
    */
    void saveCVImage(const cv_bridge::CvImagePtr &cv_image,
                     std::string log_dir="/tmp/",
                     std::string obj_name="unknown");

    /** \brief Convert sensor_msgs/Image to cv_image
     * \param[in] Sensor_msg/Image
     * \param[out] cv image output
    */
    bool getCVImage(const sensor_msgs::ImageConstPtr &image,
                    cv_bridge::CvImagePtr &cv_image);

}
}

#endif  // MIR_PERCCEPTION_UTILS_OBJECT_UTILS_H
