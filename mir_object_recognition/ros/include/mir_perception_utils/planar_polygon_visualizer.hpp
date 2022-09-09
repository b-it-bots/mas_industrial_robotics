#ifndef MIR_PERCEPTION_UTILS_PLANAR_POLYGON_VISUALIZER_HPP
#define MIR_PERCEPTION_UTILS_PLANAR_POLYGON_VISUALIZER_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rmw/qos_profiles.h"

#include "pcl/geometry/planar_polygon.h"
#include "pcl/point_cloud.h"
#include "visualization_msgs/msg/marker.hpp"
#include "mir_perception_utils/aliases.hpp"
#include "mir_perception_utils/color.hpp"

namespace mir_perception_utils
{
    namespace visualization
    {
        class PlanarPolygonVisualizer
        {
        public:
            PlanarPolygonVisualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                                    const std::string &topic_name, Color color,
                                    bool check_subscribers = true, double thickness = 0.005);

            PlanarPolygonVisualizer(const std::string &topic_name, Color color,
                                    bool check_subscribers = true, double thickness = 0.005);

            
            void publish(const PlanarPolygon &polygon,
                         const std::string &frame_id);

            /** Fill the fields of the marker object so that it visualizes the provided
                * vector of points by drawing a polyline through them. */
            template <typename PointT>
            void buildPolygonMarker(const typename PointCloud::VectorType &points,
                                    visualization_msgs::msg::Marker &marker,
                                    const std::string &frame_id, int id = 1);

        private:
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
            const std::string frame_id_;
            Color color_;
            bool check_subscribers_;
            double thickness_;
            rclcpp::QoS qos_default;
        };       
    }
}
#endif // MIR_PERCEPTION_UTILS_PLANAR_POLYGON_VISUALIZER_HPP