#include "mir_perception_utils/planar_polygon_visualizer.hpp"

namespace mir_perception_utils
{
    namespace visualization
    {
        PlanarPolygonVisualizer::PlanarPolygonVisualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                                                         const std::string &topic_name, Color color,
                                                         bool check_subscribers, double thickness)
            : color_(color), check_subscribers_(check_subscribers), thickness_(thickness),
              qos_default(rclcpp::KeepLast(10), rmw_qos_profile_default)
        {
            
            marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(topic_name, qos_default);
        }

        PlanarPolygonVisualizer::PlanarPolygonVisualizer(const std::string &topic_name, Color color,
                                                         bool check_subscribers, double thickness)
            : color_(color), check_subscribers_(check_subscribers), thickness_(thickness),
              qos_default(rclcpp::KeepLast(10), rmw_qos_profile_default)
        {

            auto node = rclcpp::Node::make_shared("_");

            marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(topic_name, qos_default);
        }
        
        void PlanarPolygonVisualizer::publish(const PlanarPolygon &polygon,
                                              const std::string &frame_id)
        {
            if (check_subscribers_ && !marker_publisher_->get_subscription_count())
            {
                return;
            }

            visualization_msgs::msg::Marker marker;
            buildPolygonMarker<PointT>(polygon.getContour(), marker, frame_id);
            marker_publisher_->publish(marker);
        }

        template <typename PointT>
        void PlanarPolygonVisualizer::buildPolygonMarker(const typename PointCloud::VectorType &points,
                                                         visualization_msgs::msg::Marker &marker,
                                                         const std::string &frame_id, int id)
        {
            if (!points.size())
                return;

            marker.header.frame_id = frame_id;
            marker.ns = "polygon";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = thickness_;
            marker.scale.y = thickness_;
            marker.color.a = 1.0;
            marker.color = color_;

            geometry_msgs::msg::Point first_point;
            first_point.x = points[0].x;
            first_point.y = points[0].y;
            first_point.z = points[0].z;
            marker.points.push_back(first_point);

            for (size_t i = 0; i < points.size(); i++)
            {
                geometry_msgs::msg::Point point;
                point.x = points[i].x;
                point.y = points[i].y;
                point.z = points[i].z;
                marker.points.push_back(point);
            }
            marker.points.push_back(first_point);
        }
    }
}