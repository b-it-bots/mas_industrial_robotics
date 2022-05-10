/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka, Sergey Alexandrov
 *
 */
#ifndef MIR_PERCEPTION_UTILS_LABEL_VISUALIZER_H
#define MIR_PERCEPTION_UTILS_LABEL_VISUALIZER_H

#include <string>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

#include <mir_perception_utils/color.h>

using mir_perception_utils::visualization::Color;

namespace mir_perception_utils
{
namespace visualization
{
class LabelVisualizer
{
 public:
  LabelVisualizer(const ros::NodeHandle &nh, const std::string &topic_name, Color color,
                  bool check_subscribers = true);

  LabelVisualizer(const std::string &topic_name, Color color, bool check_subscribers = true);

  void publish(const std::vector<std::string> &labels, const geometry_msgs::PoseArray &poses);

  int getNumSubscribers();

 private:
  ros::Publisher marker_publisher_;

  const Color color_;
  bool check_subscribers_;
};

}  // namespace visualization

}  // namespace mir
#include "impl/label_visualizer.hpp"

#endif  // MIR_PERCEPTION_UTILS_LABEL_VISUALIZER_H
