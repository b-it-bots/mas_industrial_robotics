// #include <rclcpp.hpp>

// class DataLogger : public rclcpp::Node
// {
// public:
//     DataLogger("data_logger")
//     {

//     }

//-----------------------`-----------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include "std_msgs/msg/string.hpp"
#include "mir_perception_utils/pointcloud_utils_ros.hpp"
#include "mir_perception_utils/object_utils_ros.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <memory>
#include <vector>
using std::placeholders::_1;

namespace mpu = mir_perception_utils;

class DataLogger : public rclcpp::Node
{
public:
    DataLogger(): Node("data_logger")
    {
        this->declare_parameter<std::string>("logdir", "/tmp/");
        this->get_parameter("logdir", logdir_);
        // clusters_3d_subscriber_ = this->create_subscription<std::vector<PointCloud::Ptr>>("pointcloud_topic", 10, std::bind(&DataLogger::save_pointcloud, this, _1));
        raw_cv_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("raw_cv_image_topic", 10, std::bind(&DataLogger::save_cv_image, this, _1));
        clusters_3d_subscriber_ = this->create_subscription<std::vector<std::shared_ptr<PointCloud>>>("pointcloud_topic", 10, std::bind(&DataLogger::save_pointcloud, this, _1));
    }
private:
    void save_pointcloud(std::vector<std::shared_ptr<PointCloud>> &clusters_3d_)
    {
        for (auto &cluster : clusters_3d_)
       {
            std::string filename = "";
            filename.append("pcd_cluster");
            // filename.append(std::to_string(cluster->header.timestamp));
            mpu::object::savePcd(cluster, logdir_, filename);
            // RCLCPP_INFO(get_logger(), "Saved cluster with timestamp %d", cluster->header.timestamp);
        }
    }
    void save_cv_image(const std::shared_ptr<sensor_msgs::msg::Image> image_msg_)
    {
        cv_bridge::CvImagePtr raw_cv_image;
        if (mpu::object::getCVImage(image_msg_, raw_cv_image))
        {
            std::string filename = "";
            filename.append("raw_cv_image");
            // filename.append(std::to_string(raw_cv_image->header.timestamp));
            mpu::object::saveCVImage(raw_cv_image, logdir_, filename);
            // RCLCPP_INFO(get_logger(), "Saved raw_cv_image with timestamp %d", raw_cv_image->header.timestamp);
        }
    }
    // rclcpp::Subscription<std::vector<PointCloud::Ptr>>::SharedPtr clusters_3d_subscriber_;
    rclcpp::Subscription<std::vector<std::shared_ptr<PointCloud>>>::SharedPtr clusters_3d_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_cv_image_subscriber_;
    std::string logdir_;

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLogger>());
  rclcpp::shutdown();
  return 0;
}