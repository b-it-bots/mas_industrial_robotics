/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */
#ifndef MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_HPP
#define MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_HPP

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include "mir_perception_utils/aliases.hpp"
#include "mas_perception_msgs/msg/object.hpp"

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
     * \param[in] the height adjustment of the container, default 10cm
    */
    void adjustContainerPose(mas_perception_msgs::msg::Object &container_object, float container_height=0.1);
};
#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_HPP
