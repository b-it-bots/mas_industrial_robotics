#ifndef MIR_PERCEPTION_UTILS_CLUSTERED_POINT_CLOUD_VISUALIZER_HPP
#define MIR_PERCEPTION_UTILS_CLUSTERED_POINT_CLOUD_VISUALIZER_HPP

#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "mir_perception_utils/aliases.hpp"
#include "mir_perception_utils/color.hpp"

namespace mir_perception_utils
{
  namespace visualization
  {
    class ClusteredPointCloudVisualizer
    {
    public:
      ClusteredPointCloudVisualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                                    const std::string &topic_name, bool check_subscribers = true);

      ClusteredPointCloudVisualizer(const std::string &topic_name, bool check_subscribers = true);

      
      void publish(const std::vector<PointCloudBSPtr> &clusters,
                   const std::string &frame_id);
      int getNumSubscribers();

      rclcpp::QoS qos_sensor;

    private:
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

      bool check_subscribers_;

      static const size_t COLORS_NUM = 32;
      float COLORS[COLORS_NUM];
      
      
    };

  } // namespace visualization

} // namespace mir
//#include "mir_perception_utils/impl/clustered_point_cloud_visualizer.hpp"
#endif // MIR_PERCEPTION_UTILS_CLUSTERED_POINT_CLOUD_VISUALIZER_H