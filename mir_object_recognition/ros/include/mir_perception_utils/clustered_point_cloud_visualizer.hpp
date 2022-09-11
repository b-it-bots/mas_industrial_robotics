/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 * ROS2 contributors: Vamsi Kalagaturu, Vivek Mannava.
 *
 */

#ifndef MIR_PERCEPTION_UTILS_CLUSTERED_POINT_CLOUD_VISUALIZER_HPP
#define MIR_PERCEPTION_UTILS_CLUSTERED_POINT_CLOUD_VISUALIZER_HPP

#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "mir_perception_utils/aliases.hpp"
#include "mir_perception_utils/color.hpp"

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"

#include "rmw/qos_profiles.h"

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

    private:
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

      bool check_subscribers_;

      static const size_t COLORS_NUM = 32;
      float COLORS[COLORS_NUM];
    };

  } // namespace visualization
} // namespace mir
#endif // MIR_PERCEPTION_UTILS_CLUSTERED_POINT_CLOUD_VISUALIZER_H