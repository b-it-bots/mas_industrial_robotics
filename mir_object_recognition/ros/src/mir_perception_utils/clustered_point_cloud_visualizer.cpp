#include "mir_perception_utils/clustered_point_cloud_visualizer.hpp"

using mir_perception_utils::visualization::Color;

namespace mir_perception_utils
{
    namespace visualization
    {
        ClusteredPointCloudVisualizer::ClusteredPointCloudVisualizer(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
            const std::string &topic_name, bool check_subscribers)
            : check_subscribers_(check_subscribers)
        {
            auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
            cloud_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
                topic_name, qos_sensor);
            for (size_t i = 0; i < COLORS_NUM; i++)
            {
                COLORS[i] = 1.0f * rand() / RAND_MAX;
            }
        }

        ClusteredPointCloudVisualizer::ClusteredPointCloudVisualizer(
            const std::string &topic_name, bool check_subscribers)
            : check_subscribers_(check_subscribers)
        {
            auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
            rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("_");
            cloud_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
                topic_name, qos_sensor);
            for (size_t i = 0; i < COLORS_NUM; i++)
            {
                COLORS[i] = 1.0f * rand() / RAND_MAX;
            }
        }

        int ClusteredPointCloudVisualizer::getNumSubscribers()
        {
            return cloud_publisher_->get_subscription_count();
        }
        
        void ClusteredPointCloudVisualizer::publish(
            const std::vector<PointCloudBSPtr> &clusters,
            const std::string &frame_id)
        {
            if (getNumSubscribers() == 0)
                return;

            pcl::PointCloud<pcl::PointXYZRGB> composite;
            size_t color = 0;

            for (size_t i = 0; i < clusters.size(); i++)
            {
                const PointCloudBSPtr &cloud = clusters[i];
                for (size_t j = 0; j < cloud->points.size(); j++)
                {
                    const PointT &point = cloud->points[j];
                    pcl::PointXYZRGB pt;
                    pt.x = point.x;
                    pt.y = point.y;
                    pt.z = point.z;
                    pt.rgb = float(Color(static_cast<Color::Name>(color)));
                    composite.points.push_back(point);
                }
                color++;
            }
            composite.header.frame_id = frame_id;
            composite.width = static_cast<uint32_t>(composite.points.size());
            composite.height = 1;

            pcl::PCLPointCloud2 pc2;
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toPCLPointCloud2(composite, pc2);
            pcl_conversions::fromPCL(pc2, cloud_msg);
            cloud_publisher_->publish(cloud_msg);
        }
    } // namespace visualization
} // namespace mir_perception_utils
