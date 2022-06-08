#include <std_msgs/msg/float64.hpp>

#include "mir_object_recognition/multimodal_object_recognition.hpp"

#include "mir_perception_utils/pointcloud_utils_ros.hpp"

MultiModalObjectRecognitionROS::MultiModalObjectRecognitionROS(const std::string & node_name, bool intra_process_comms):
    rclcpp_lifecycle::LifecycleNode(node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
        cluster_visualizer_rgb_("output/tabletop_cluster_rgb", true),
        cluster_visualizer_pc_("output/tabletop_cluster_pc")
{
    RCLCPP_INFO(get_logger(), "constructor called");
    this -> declare_parameter<std::string>("target_frame_id", "base_link");
    this -> get_parameter("target_frame_id", target_frame_id_);
}

void MultiModalObjectRecognitionROS::synchronizeCallback(const std::shared_ptr<sensor_msgs::msg::Image> &image,
                      const std::shared_ptr<sensor_msgs::msg::PointCloud2> &cloud)
{

    RCLCPP_INFO(get_logger(), "synchro callback");
    RCLCPP_INFO(get_logger(), "TS: [%u]; [%u]", image -> header.stamp.sec, cloud -> header.stamp.sec);
    
    pointcloud_msg_ = cloud;
    image_msg_ = image;

    // pre-process the pointcloud
    this -> preprocessPointCloud(pointcloud_msg_);
}

void MultiModalObjectRecognitionROS::preprocessPointCloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &cloud_msg)
{
    sensor_msgs::msg::PointCloud2 msg_transformed;
    msg_transformed.header.frame_id = target_frame_id_;
    if (!mpu::pointcloud::transformPointCloudMsg(tf_buffer_, target_frame_id_, *cloud_msg, msg_transformed))
    {
        RCLCPP_ERROR(get_logger(), "Failed to transform point cloud");
        return;
    }
    
    std::shared_ptr<pcl::PCLPointCloud2> pc2 = std::make_shared<pcl::PCLPointCloud2>();
    pcl_conversions::toPCL(msg_transformed, *pc2);
    pc2 -> header.frame_id = msg_transformed.header.frame_id;
    
    cloud_ = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::fromPCLPointCloud2(*pc2, *cloud_);

    RCLCPP_INFO(get_logger(), "Transformed point cloud");
}

void MultiModalObjectRecognitionROS::segmentPointCloud(mas_perception_msgs::msg::ObjectList &obj_list,
                        std::vector<std::shared_ptr<PointCloud>> &clusters,
                        std::vector<mpu::object::BoundingBox> boxes)
{
    std::shared_ptr<PointCloud> cloud = std::make_shared<PointCloud>();
    cloud -> header.frame_id = target_frame_id_;

}

void MultiModalObjectRecognitionROS::recognizeCloudAndImage()
{
    mas_perception_msgs::msg::ObjectList cloud_object_list;
    std::vector<std::shared_ptr<PointCloud>> clusters_3d;
    std::vector<mpu::object::BoundingBox> boxes;


}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_configure(const rclcpp_lifecycle::State &)
{
    // This callback is supposed to be used for initialization and
    // configuring purposes.
    // We thus initialize and configure our publishers and timers.
    // The lifecycle node API does return lifecycle components such as
    // lifecycle publishers. These entities obey the lifecycle and
    // can comply to the current state of the node.
    // As of the beta version, there is only a lifecycle publisher
    // available.
    
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    image_sub_ -> subscribe(this, "input_image_topic");
    cloud_sub_ -> subscribe(this, "input_cloud_topic");
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformer/pointcloud",10);

    //msg_sync_.reset(new Sync(msgSyncPolicy(10), image_sub_, cloud_sub_));
    msg_sync_ = std::make_shared<Sync>(msgSyncPolicy(10), *image_sub_, *cloud_sub_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "unconfigured" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_activate(const rclcpp_lifecycle::State &)
{
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    // Let's sleep for 2 seconds.
    // We emulate we are doing important
    // work in the activating phase.
    std::this_thread::sleep_for(2s);

    msg_sync_ -> registerCallback(&MultiModalObjectRecognitionROS::synchronizeCallback, this);

    // We return a success and hence invoke the transition to the next
    // step: "active".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    image_sub_ -> unsubscribe();
    cloud_sub_ -> unsubscribe();

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "active" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_cleanup(const rclcpp_lifecycle::State &)
{
    // In our cleanup phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    // obj_list_pub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    msg_sync_.reset();

    // We return a success and hence invoke the transition to the next
    // step: "unconfigured".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_shutdown(const rclcpp_lifecycle::State & state)
{
    // In our shutdown phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    // obj_list_pub_.reset();

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());

    // We return a success and hence invoke the transition to the next
    // step: "finalized".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the current state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<MultiModalObjectRecognitionROS> mmor_lc_node =
    std::make_shared<MultiModalObjectRecognitionROS>("multimodal_object_recognition", false);

  exe.add_node(mmor_lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}