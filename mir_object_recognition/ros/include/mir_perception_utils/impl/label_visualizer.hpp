#ifndef LABEL_VISUALIZER_HPP
#define LABEL_VISUALIZER_HPP

#include <geometry_msgs/msg/pose_array.hpp>
#include <mir_perception_utils/color.h>
// #include <ros/node_handle.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using mir_perception_utils::visualization::Color;

namespace mir_perception_utils
{
namespace visualization
{
LabelVisualizer::LabelVisualizer(const ros::NodeHandle &nh, const std::string &topic_name,
                                 Color color, bool check_subscribers)
    : color_(color), check_subscribers_(check_subscribers)
{
  ros::NodeHandle nh_(nh);
  marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 10);
}

LabelVisualizer::LabelVisualizer(const std::string &topic_name, Color color, bool check_subscribers)
    : color_(color), check_subscribers_(check_subscribers)
{
  ros::NodeHandle nh("~");
  marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 10);
}

int LabelVisualizer::getNumSubscribers() { return marker_publisher_.getNumSubscribers(); }
void LabelVisualizer::publish(const std::vector<std::string> &labels,
                              const geometry_msgs::msg:PoseArray &poses)
{
  visualization_msgs::MarkerArray markers;
  for (int i = 0; i < labels.size(); i++) {
    visualization_msgs::Marker m;
    m.header = poses.header;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.z = 0.04;
    m.color = std_msgs::msg::ColorRGBA(color_);
    m.ns = "labels";
    m.id = i;
    m.pose.position.x = poses.poses[i].position.x;
    m.pose.position.y = poses.poses[i].position.y + 0.02;
    m.pose.position.z = poses.poses[i].position.z;
    m.pose.orientation.w = 1.0;
    m.text = labels[i];
    markers.markers.push_back(m);
  }
  marker_publisher_.publish(markers);
}
}
}

#endif /* BOUNDING_BOX_VISUALIZER_HPP */
