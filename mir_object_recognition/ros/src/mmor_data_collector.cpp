#include "mir_object_recognition/mmor_data_collector.hpp"
namespace perception_namespace
{
    DataCollector::DataCollector(const rclcpp::NodeOptions& options): MultiModalObjectRecognitionROS(options)
    {
    }
    void DataCollector::recognizeCloudAndImage()
    {
        RCLCPP_INFO(get_logger(),"Inside data collection node recognize cloud image function");
        mas_perception_msgs::msg::ObjectList cloud_object_list;
        std::vector<PointCloudBSPtr> clusters_3d;
        std::vector<mpu::object::BoundingBox> boxes;

        this->segmentPointCloud(cloud_object_list, clusters_3d, boxes);
        
        std::string filename;
        RCLCPP_INFO(get_logger(),"Saving pointcloud clusters");
        for (auto& cluster : clusters_3d)
        {                
            filename = "";
            filename.append("pcd_cluster_");
            filename.append(std::to_string(this->get_clock()->now().seconds()));
            mpu::object::savePcd(cluster, logdir_, filename);
        }
        // Save raw image
        cv_bridge::CvImagePtr raw_cv_image;
        if (mpu::object::getCVImage(image_msg_, raw_cv_image))
        {
            RCLCPP_INFO(get_logger(),"Saving RGB Image");
            std::string filename = "";
            filename.append("rgb_raw_");
            filename.append(std::to_string(this->get_clock()->now().seconds()));
            mpu::object::saveCVImage(raw_cv_image, logdir_, filename);
        }
        else
        {
            RCLCPP_ERROR(get_logger(),"Cannot generate cv image...");
        }

        return;
    }

} // namespace perception_namespace

RCLCPP_COMPONENTS_REGISTER_NODE(perception_namespace::DataCollector)