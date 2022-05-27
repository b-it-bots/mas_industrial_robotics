/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 *
 */
#ifndef MIR_PERCEPTION_UTILS_BOUNDING_BOX_VISUALIZER_H
#define MIR_PERCEPTION_UTILS_BOUNDING_BOX_VISUALIZER_H

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mas_perception_msgs/msg/bounding_box.hpp>
#include <mir_perception_utils/color.h>
#include <visualization_msgs/msg/marker.hpp>

using mir_perception_utils::visualization::Color;

namespace mir_perception_utils
{
namespace visualization
{
class BoundingBoxVisualizer
{
 public:
  // BoundingBoxVisualizer(ros::NodeHandle *nh, const std::string &topic_name, Color color,
  //                       bool check_subscribers = true);

  BoundingBoxVisualizer(const std::string &topic_name, Color color, bool check_subscribers = true);

  void publish(const mas_perception_msgs::msg::BoundingBox &box, const std::string &frame_id);

  void publish(const std::vector<mas_perception_msgs::msg::BoundingBox> &boxes,
               const std::string &frame_id);

  int getNumSubscribers();

 private:
  // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  const Color color_;
  bool check_subscribers_;
};

}  // namespace visualization

}  // namespace mir
#include "impl/bounding_box_visualizer.hpp"

#endif  // MIR_PERCEPTION_UTILS_BOUNDING_BOX_VISUALIZER_H
