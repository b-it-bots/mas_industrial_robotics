/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 * ROS2 contributors: Shubham Shinde.
 *
 */

#ifndef MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_HPP
#define MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_HPP

#include <cv_bridge/cv_bridge.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include "mir_perception_utils/aliases.hpp"
#include "mas_perception_msgs/msg/object.hpp"

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

class MultimodalObjectRecognitionUtils
{
  public:
    /** Constructor
     * */
    MultimodalObjectRecognitionUtils();
    virtual ~MultimodalObjectRecognitionUtils();
    // explicit MultimodalObjectRecognitionUtils(const rclcpp::NodeOptions& options);

    /** \brief Make adjustment for AXIS and Bolt (M20_100) pose. Adjust it so that the pose
     *     is around to the center of mass (the tip).
     * \param[in] object
     * 
    */
    void adjustAxisBoltPose(mas_perception_msgs::msg::Object &object);

    /** \brief Adjust container pose by finding the largest cluster to find the center
     * \param[in] object.views[0].point_cloud
     * \param[in] height adjustment of the container, default 10cm
    */
    void adjustContainerPose(mas_perception_msgs::msg::Object &container_object, float container_height=0.1);
};
#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_HPP
