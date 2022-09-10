#include "mir_perception_utils/bounding_box_visualizer.hpp"


using mir_perception_utils::visualization::Color;

namespace mir_perception_utils
{
    namespace visualization
    {
        BoundingBoxVisualizer::BoundingBoxVisualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                                                     const std::string &topic_name, Color color,
                                                     bool check_subscribers) : color_(color), check_subscribers_(check_subscribers)
        {
            
            qos_default(rclcpp::KeepLast(10), rmw_qos_profile_default);    
            marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(
                topic_name, qos_default);
        }

        BoundingBoxVisualizer::BoundingBoxVisualizer(const std::string &topic_name, Color color,
                                                     bool check_subscribers) : color_(color), check_subscribers_(check_subscribers)
        {
            qos_default(rclcpp::KeepLast(10), rmw_qos_profile_default);
            auto node = rclcpp::Node::make_shared("_");
            marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(
                topic_name, qos_default);
        }

        int BoundingBoxVisualizer::getNumSubscribers() { return marker_publisher_->get_subscription_count(); }

        void BoundingBoxVisualizer::publish(const mas_perception_msgs::msg::BoundingBox &box,
                                            const std::string &frame_id)
        {
            std::vector<mas_perception_msgs::msg::BoundingBox> boxes;
            boxes.push_back(box);
            publish(boxes, frame_id);
        }

        void BoundingBoxVisualizer::publish(const std::vector<mas_perception_msgs::msg::BoundingBox> &boxes,
                                            const std::string &frame_id)
        {
            if (check_subscribers_ && marker_publisher_->get_subscription_count() == 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("BoundingBoxVisualizer"),
                            "No subscribers to topic %s, not publishing",
                            marker_publisher_->get_topic_name());
                return;
            }

            visualization_msgs::msg::Marker lines;
            lines.header.frame_id = frame_id;
            // getting current time is not properly defined in ros2
            // the below line might not work
            lines.header.stamp = rclcpp::Clock().now();
            lines.type = visualization_msgs::msg::Marker::LINE_LIST;
            lines.action = visualization_msgs::msg::Marker::ADD;
            lines.scale.x = 0.01;
            lines.scale.y = 0.01;
            lines.color = std_msgs::msg::ColorRGBA(color_);
            lines.ns = "bounding_box";
            lines.id = 1;

            for (size_t i = 0; i < boxes.size(); i++)
            {
                const std::vector<geometry_msgs::msg::Point> &pt = boxes[i].vertices;
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
            marker_publisher_->publish(lines);
        }
    } // namespace visualization
} // namespace mir_perception_utils
