#include "mir_object_recognition/multimodal_object_recognition.hpp"

namespace vivek
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
} // namespace vivek

RCLCPP_COMPONENTS_REGISTER_NODE(vivek::DataCollector)