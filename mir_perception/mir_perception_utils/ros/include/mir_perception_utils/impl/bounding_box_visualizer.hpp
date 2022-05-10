#ifndef BOUNDING_BOX_VISUALIZER_HPP
#define BOUNDING_BOX_VISUALIZER_HPP

#include <mir_perception_utils/color.h>
#include <ros/node_handle.h>
#include <visualization_msgs/Marker.h>

using mir_perception_utils::visualization::Color;

namespace mir_perception_utils
{
namespace visualization
{
BoundingBoxVisualizer::BoundingBoxVisualizer(ros::NodeHandle *nh, const std::string &topic_name,
                                             Color color, bool check_subscribers)
    : color_(color), check_subscribers_(check_subscribers)
{
  marker_publisher_ = nh->advertise<visualization_msgs::Marker>(topic_name, 10);
}

BoundingBoxVisualizer::BoundingBoxVisualizer(const std::string &topic_name, Color color,
                                             bool check_subscribers)
    : color_(color), check_subscribers_(check_subscribers)
{
  ros::NodeHandle nh("~");
  marker_publisher_ = nh.advertise<visualization_msgs::Marker>(topic_name, 10);
}

int BoundingBoxVisualizer::getNumSubscribers() { return marker_publisher_.getNumSubscribers(); }
void BoundingBoxVisualizer::publish(const mas_perception_msgs::BoundingBox &box,
                                    const std::string &frame_id)
{
  std::vector<mas_perception_msgs::BoundingBox> boxes;
  boxes.push_back(box);
  publish(boxes, frame_id);
}

void BoundingBoxVisualizer::publish(const std::vector<mas_perception_msgs::BoundingBox> &boxes,
                                    const std::string &frame_id)
{
  if (check_subscribers_ && marker_publisher_.getNumSubscribers() == 0) return;
  visualization_msgs::Marker lines;
  lines.header.frame_id = frame_id;
  lines.header.stamp = ros::Time::now();
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.001;
  lines.scale.y = 0.001;
  lines.color = std_msgs::ColorRGBA(color_);
  lines.ns = "bounding_boxes";
  lines.id = 1;

  for (size_t i = 0; i < boxes.size(); i++) {
    const std::vector<geometry_msgs::Point> &pt = boxes[i].vertices;
    lines.points.push_back(pt[0]);
    lines.points.push_back(pt[1]);
    lines.points.push_back(pt[0]);
    lines.points.push_back(pt[3]);
    lines.points.push_back(pt[0]);
    lines.points.push_back(pt[4]);
    lines.points.push_back(pt[1]);
    lines.points.push_back(pt[2]);
    lines.points.push_back(pt[1]);
    lines.points.push_back(pt[5]);
    lines.points.push_back(pt[2]);
    lines.points.push_back(pt[3]);
    lines.points.push_back(pt[2]);
    lines.points.push_back(pt[6]);
    lines.points.push_back(pt[3]);
    lines.points.push_back(pt[7]);
    lines.points.push_back(pt[4]);
    lines.points.push_back(pt[5]);
    lines.points.push_back(pt[4]);
    lines.points.push_back(pt[7]);
    lines.points.push_back(pt[5]);
    lines.points.push_back(pt[6]);
    lines.points.push_back(pt[6]);
    lines.points.push_back(pt[7]);
  }
  marker_publisher_.publish(lines);
}
}
}

#endif /* BOUNDING_BOX_VISUALIZER_HPP */
