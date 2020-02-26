/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 *
 */
#ifndef MIR_PERCEPTION_UTILS_CLUSTERED_POINT_CLOUD_VISUALIZER_H
#define MIR_PERCEPTION_UTILS_CLUSTERED_POINT_CLOUD_VISUALIZER_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <pcl/point_cloud.h>

#include <mir_perception_utils/color.h>

namespace mir_perception_utils
{

namespace visualization
{

class ClusteredPointCloudVisualizer
{
public:
    ClusteredPointCloudVisualizer(const boost::shared_ptr<ros::NodeHandle> &nh, const std::string& topic_name,
                                  bool check_subscribers = true);

    ClusteredPointCloudVisualizer(const std::string& topic_name,
                                  bool check_subscribers = true);

    template<typename PointT>
    void publish(const std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters,
                 const std::string& frame_id);
    int getNumSubscribers();

private:
    ros::Publisher cloud_publisher_;

    bool check_subscribers_;

    static const size_t COLORS_NUM = 32;
    float COLORS[COLORS_NUM];
};

}  // namespace visualization

}  // namespace mir
#include "impl/clustered_point_cloud_visualizer.hpp"

#endif  // MIR_PERCEPTION_UTILS_CLUSTERED_POINT_CLOUD_VISUALIZER_H
