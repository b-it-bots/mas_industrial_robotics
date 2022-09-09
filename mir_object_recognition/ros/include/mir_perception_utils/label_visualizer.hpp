#ifndef MIR_PERCEPTION_UTILS_LABEL_VISUALIZER_HPP
#define MIR_PERCEPTION_UTILS_LABEL_VISUALIZER_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rmw/qos_profiles.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "mir_perception_utils/aliases.hpp"
#include "mir_perception_utils/color.hpp"

using mir_perception_utils::visualization::Color;

namespace mir_perception_utils
{
    namespace visualization
    {
        class LabelVisualizer
        {
        public:
            LabelVisualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                            const std::string &topic_name, Color color,
                            bool check_subscribers = true);

            LabelVisualizer(const std::string &topic_name, Color color,
                            bool check_subscribers = true);

            void publish(const std::vector<std::string> &labels, const geometry_msgs::msg::PoseArray &poses);

            int getNumSubscribers();
        
        private:
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
            const Color color_;
            bool check_subscribers_;
            rclcpp::QoS qos_default;
            
        };
    }
}
#endif // MIR_PERCEPTION_UTILS_LABEL_VISUALIZER_HPP