#include <mir_object_recognition/multimodal_object_recognition.hpp>

namespace perception_namespace
{
class DataCollector : public MultiModalObjectRecognitionROS
{
    public:
        explicit DataCollector(const rclcpp::NodeOptions& options);
        void recognizeCloudAndImage();
};
    
} // namespace perception_namespace