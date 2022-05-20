
#include <std_msgs/msg/float64.hpp>
#include <mir_object_recognition/multimodal_object_recognition.hpp>
#include "scene_segmentation_ros.cpp"

using std::placeholders::_1;

class Transformer : public rclcpp::Node
{
public:
    Transformer():Node("pointcloud_tranformer")
    {
        cloud_ = PointCloud::Ptr(new PointCloud);
        scene_segmentation_ros_ = std::shared_ptr<SceneSegmentationROS>(new SceneSegmentationROS());

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        pc_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points",10,std::bind(&Transformer::callback_func,this,_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformer/pointcloud",10);
    }
    bool transformpointcloud(const std::shared_ptr<tf2_ros::TransformListener> &tf_listener, 
                             const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                             const std::string target_frame, 
                             const sensor_msgs::msg::PointCloud2 cloud_in,
                             sensor_msgs::msg::PointCloud2 cloud_out)
    {
        // if(tf_listener)
        // {
        //     RCLCPP_INFO(this->get_logger(),"Coming to the transform function");
        // }
        if (tf_listener) 
        {
            // geometry_msgs::msg::TransformStamped transformStamped;
            try 
            {
                // transformStamped = tf_buffer->lookupTransform(target_frame, cloud_in.header.frame_id,tf2::TimePointZero);
                // pcl_ros::transformPointcloud
                // RCLCPP_INFO(this->get_logger(),"transformed stamped obtained");
                pcl_ros::transformPointCloud(target_frame,cloud_in,cloud_out,*tf_buffer);
                RCLCPP_INFO(this->get_logger(), "Transform throws no error");
                publisher_->publish(cloud_out);
                scene_segmentation_ros_->addCloudAccumulation(cloud_);
            } 
            catch (tf2::TransformException & ex) 
            {
                RCLCPP_INFO(this->get_logger(), "Could not transform");
                return (false);
            }
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "TF listener not initialized");
            // RCLCPP_ERROR_THROTTLE(2.0, "TF listener not initialized.");
            return (false);
        }
        return (true);
    }
private:
    void callback_func(const sensor_msgs::msg::PointCloud2 cloud)
    {
        RCLCPP_INFO(this->get_logger(),"Coming to the call back function");
        sensor_msgs::msg::PointCloud2 transformed_msg;
        this->transformpointcloud(tf_listener_, tf_buffer_, target_frame_id_, cloud, transformed_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_id_ = "base_link";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    typedef std::shared_ptr<SceneSegmentationROS> SceneSegmentationROSSPtr;
    SceneSegmentationROSSPtr scene_segmentation_ros_;
    PointCloud::Ptr cloud_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transformer>());
  rclcpp::shutdown();
  return 0;
}

