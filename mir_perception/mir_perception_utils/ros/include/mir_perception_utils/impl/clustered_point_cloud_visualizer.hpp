#ifndef CLUSTERED_POINT_CLOUD_VISUALIZER_HPP
#define CLUSTERED_POINT_CLOUD_VISUALIZER_HPP

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <mir_perception_utils/aliases.h>
#include <mir_perception_utils/color.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using mir_perception_utils::visualization::Color;

namespace mir_perception_utils {

namespace visualization {

ClusteredPointCloudVisualizer::ClusteredPointCloudVisualizer(
    const boost::shared_ptr<ros::NodeHandle> &nh, const std::string &topic_name,
    bool check_subscribers)
    : check_subscribers_(check_subscribers) {
  cloud_publisher_ = nh->advertise<sensor_msgs::PointCloud2>(topic_name, 1);
  for (size_t i = 0; i < COLORS_NUM; ++i) {
    COLORS[i] = 1.0f * rand() / RAND_MAX;
  }
}

ClusteredPointCloudVisualizer::ClusteredPointCloudVisualizer(
    const std::string &topic_name, bool check_subscribers)
    : check_subscribers_(check_subscribers) {
  ros::NodeHandle nh("~");
  cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
  for (size_t i = 0; i < COLORS_NUM; ++i) {
    COLORS[i] = 1.0f * rand() / RAND_MAX;
  }
}

int ClusteredPointCloudVisualizer::getNumSubscribers() {
  return cloud_publisher_.getNumSubscribers();
}

template <typename PointT>
void ClusteredPointCloudVisualizer::publish(
    const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clusters,
    const std::string &frame_id) {
  if (cloud_publisher_.getNumSubscribers() == 0) return;
  pcl::PointCloud<pcl::PointXYZRGB> composite;
  size_t color = 0;

  for (size_t i = 0; i < clusters.size(); i++) {
    const PointCloud::Ptr &cloud = clusters[i];
    for (size_t j = 0; j < cloud->points.size(); j++) {
      const PointT &point = cloud->points[j];
      pcl::PointXYZRGB pt;
      pt.x = point.x;
      pt.y = point.y;
      pt.z = point.z;
      pt.rgb = float(Color(static_cast<Color::Name>(color)));
      composite.points.push_back(pt);
    }
    color++;
  }
  composite.header.frame_id = frame_id;
  composite.width = static_cast<uint32_t>(composite.points.size());
  composite.height = 1;

  pcl::PCLPointCloud2 pc2;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toPCLPointCloud2(composite, pc2);
  pcl_conversions::fromPCL(pc2, cloud_msg);
  cloud_publisher_.publish(cloud_msg);
}
}
}

#endif /* CLUSTERED_POINT_CLOUD_VISUALIZER_HPP */
