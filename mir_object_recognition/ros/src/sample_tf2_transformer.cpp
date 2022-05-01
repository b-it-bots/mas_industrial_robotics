#include <std_msgs/msg/float64.hpp>
#include <mir_object_recognition/multimodal_object_recognition.hpp>

using std::placeholders::_1;

class Transformer : public rclcpp::Node
{
public:
    Transformer():Node("pointcloud_tranformer")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        pc_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points",10,std::bind(&Transformer::callback_func,this,_1));
    }
    void transformpointcloud(const std::shared_ptr<tf2_ros::TransformListener> &tf_listener, 
                             const std::string target_frame, 
                             const sensor_msgs::msg::PointCloud2 cloud_in,
                             sensor_msgs::msg::PointCloud2 cloud_out)
    {
        RCLCPP_INFO(this->get_logger(),"TF listener: ",tf_listener);
        if(tf_listener)
        {
            RCLCPP_INFO(this->get_logger(),"Coming to the transform function");
        }
    }
private:
    void callback_func(const sensor_msgs::msg::PointCloud2 cloud)
    {
        RCLCPP_INFO(this->get_logger(),"Coming to the call back function");
        sensor_msgs::msg::PointCloud2 transformed_msg;
        this->transformpointcloud(tf_listener, target_frame_id_, cloud, transformed_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_id_ = "base_link";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transformer>());
  rclcpp::shutdown();
  return 0;
}

