/*
 * Copyright 2020 Bonn-Rhein-Sieg University
 *
 * Author: Iswariya Manivannan, Mohammad Wasil
 *
 */

#include "mir_perception_utils/label_visualizer.hpp"

using mir_perception_utils::visualization::Color;

namespace mir_perception_utils
{
    namespace visualization
    {
        LabelVisualizer::LabelVisualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                                         const std::string &topic_name, Color color,
                                         bool check_subscribers)
            : color_(color), check_subscribers_(check_subscribers)
        {
            auto qos_default = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default);
            marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
                topic_name, qos_default);
        }

        LabelVisualizer::LabelVisualizer(const std::string &topic_name, Color color,
                                         bool check_subscribers)
            : color_(color), check_subscribers_(check_subscribers)
        {
            auto qos_default = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default);
            rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("_");
            marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
                topic_name, qos_default);
        }

        int LabelVisualizer::getNumSubscribers()
        {
            return marker_publisher_->get_subscription_count();
        }

        void LabelVisualizer::publish(const std::vector<std::string> &labels, const geometry_msgs::msg::PoseArray &poses)
        {
            visualization_msgs::msg::MarkerArray markers;
            for (size_t i = 0; i < labels.size(); i++)
            {
                visualization_msgs::msg::Marker m;
                m.header = poses.header;
                m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = poses.poses[i].position.x;
                m.pose.position.y = poses.poses[i].position.y + 0.02;
                m.pose.position.z = poses.poses[i].position.z;
                m.pose.orientation.w = 1.0;
                m.scale.z = 0.04;
                m.color = std_msgs::msg::ColorRGBA(color_);
                m.ns = "labels";
                m.id = i;
                m.text = labels[i];
                markers.markers.push_back(m);
            }
            marker_publisher_->publish(markers);
        }
    } // namespace visualization
} // namespace mir_perception_utils
