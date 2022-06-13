#include "mir_object_recognition/multimodal_object_recognition.hpp"

namespace perception_namespace
{
class DataCollector : public MultiModalObjectRecognitionROS
{
public:
    
    explicit DataCollector(const rclcpp::NodeOptions& options): MultiModalObjectRecognitionROS(options)
    {
        RCLCPP_INFO(get_logger(), "Hello from callback");
    }
private:
    std::string logdir_;
};
} // namespace perception_namespace

RCLCPP_COMPONENTS_REGISTER_NODE(perception_namespace::DataCollector)