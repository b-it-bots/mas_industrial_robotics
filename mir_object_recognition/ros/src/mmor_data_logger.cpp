#include "mir_object_recognition/multimodal_object_recognition.hpp"

class DataLogger : public MultiModalObjectRecognitionROS
{
public:
    
    DataLogger(): MultiModalObjectRecognitionROS("data_logger",rclcpp::NodeOptions().use_intra_process_comms(false))
    {
        this->declare_parameter<std::string>("logdir", "/tmp/");
        this->get_parameter("logdir", logdir_);
    }
virtual ~DataLogger();
private:
    std::string logdir_;
};

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<DataLogger>());
//   rclcpp::shutdown();
//   return 0;
// }

DataLogger::~DataLogger()
{
    // std::cout << "DataLogger destructor" << std::endl;
}
int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<DataLogger> mmor_lc_node = std::make_shared<DataLogger>();

  exe.add_node(mmor_lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}