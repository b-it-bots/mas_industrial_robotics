#include <std_msgs/msg/float64.hpp>
#include "mir_object_recognition/multimodal_object_recognition.hpp"



MultiModalObjectRecognitionROS::MultiModalObjectRecognitionROS(const std::string & node_name, bool intra_process_comms):
    rclcpp_lifecycle::LifecycleNode(node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
        bounding_box_visualizer_pc_("output/bounding_boxes", Color(Color::IVORY)),
        cluster_visualizer_rgb_("output/tabletop_cluster_rgb"),
        cluster_visualizer_pc_("output/tabletop_cluster_pc"),
        label_visualizer_rgb_("output/rgb_labels", Color(Color::SEA_GREEN)),
        label_visualizer_pc_("output/pc_labels", Color(Color::IVORY))
        {RCLCPP_INFO(get_logger(), "constructor called");}

void MultiModalObjectRecognitionROS::synchronizeCallback(const sensor_msgs::msg::Image &image,
                      const sensor_msgs::msg::PointCloud2 &cloud)
{

    RCLCPP_INFO(get_logger(), "synchro callback");
    RCLCPP_INFO(get_logger(), "TS: [%u]; [%u]", image.header.stamp.sec, cloud.header.stamp.sec);
    // sensor_msgs::msg::PointCloud2 transformed_msg;
    // this->preprocessPointCloud(tf_listener_, tf_buffer_, target_frame_id_, cloud, transformed_msg);
    this->preprocessPointCloud(cloud);
}

// bool MultiModalObjectRecognitionROS::preprocessPointCloud(const std::shared_ptr<tf2_ros::TransformListener> &tf_listener, 
//                                                           const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
//                                                           const std::string target_frame, 
//                                                           const sensor_msgs::msg::PointCloud2 cloud_in,
//                                                           sensor_msgs::msg::PointCloud2 cloud_out)
bool MultiModalObjectRecognitionROS::preprocessPointCloud(const sensor_msgs::msg::PointCloud2 &cloud_msg)
{
    RCLCPP_INFO(get_logger(), "preprocess point cloud");
    sensor_msgs::msg::PointCloud2 msg_transformed;
    msg_transformed.header.frame_id = target_frame_id_;
    if (!mpu::pointcloud::transformPointCloudMsg(tf_buffer_, target_frame_id_, cloud_msg, msg_transformed))
    {
        RCLCPP_ERROR(this->get_logger(),"Unable to transform pointcloud. Are you sure target_frame_id_ and pointcloud_source_frame_id are set correctly?");
        RCLCPP_ERROR(this->get_logger(),"pointcloud_source_frame_id: %s, target_frame_id: %s", pointcloud_source_frame_id_.c_str(), target_frame_id_.c_str());
        RCLCPP_ERROR(this->get_logger(),"pointcloud_source_frame_id may need to be arm_cam3d_camera_color_frame or fixed_camera_link");
        RCLCPP_ERROR(this->get_logger(),"target_frame_id may need to be base_link or base_link_static");
        return false;
    }

    pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(msg_transformed, *pc2);
    pc2->header.frame_id = msg_transformed.header.frame_id;

    cloud_ = PointCloud::Ptr(new PointCloud);
    pcl::fromPCLPointCloud2(*pc2, *cloud_);
    return true;

    // if (tf_listener) 
    //     {
    //         // geometry_msgs::msg::TransformStamped transformStamped;
    //         try 
    //         {
    //             pcl_ros::transformPointCloud(target_frame,cloud_in,cloud_out,*tf_buffer);
    //             RCLCPP_INFO(this->get_logger(), "Transform throws no error");
    //             publisher_->publish(cloud_out);
    //         } 
    //         catch (tf2::TransformException & ex) 
    //         {
    //             RCLCPP_INFO(this->get_logger(), "Could not transform");
    //             return (false);
    //         }
    //     }
    //     else 
    //     {
    //         RCLCPP_INFO(this->get_logger(), "TF listener not initialized");
    //         // RCLCPP_ERROR_THROTTLE(2.0, "TF listener not initialized.");
    //         return (false);
    //     }
    //     return (true);

}

// void MultiModalObjectRecognitionROS::publishDebug(mas_perception_msgs::msg::ObjectList &combined_object_list,
//                                                 std::vector<PointCloud::Ptr> &clusters_3d,
//                                                 std::vector<PointCloud::Ptr> &clusters_2d)
// {
//     RCLCPP_INFO(this->get_logger(), "Inside the publish debug function");
//     RCLCPP_INFO_STREAM(this->get_logger(), "Cloud list: " << recognized_cloud_list_.objects.size());
//     RCLCPP_INFO_STREAM(this->get_logger(), "RGB list: " << recognized_image_list_.objects.size());
//     RCLCPP_INFO_STREAM(this->get_logger(), "Combined object list: "<< combined_object_list.objects.size());
//     const Eigen::Vector3f normal = scene_segmentation_ros_->getPlaneNormal();

//     std::string names = "";
//     if (recognized_cloud_list_.objects.size() > 0)
//     {
//         // Bounding boxes
//         if (clusters_3d.size() > 0)
//         {
//             mas_perception_msgs::msg::BoundingBoxList bounding_boxes;
//             cluster_visualizer_pc_.publish<PointT>(clusters_3d, target_frame_id_);
//             bounding_boxes.bounding_boxes.resize(clusters_3d.size());
//             for (int i=0; i < clusters_3d.size(); i++)
//             {
//                 mpu::object::BoundingBox bbox;
//                 mpu::object::get3DBoundingBox(clusters_3d[i], normal, bbox, bounding_boxes.bounding_boxes[i]);
//             }
//             if (bounding_boxes.bounding_boxes.size() > 0)
//             {
//                 bounding_box_visualizer_pc_.publish(bounding_boxes.bounding_boxes, target_frame_id_);
//             }
//         }
//         // PCL Pose array for debug mode only
//         geometry_msgs::msg::PoseArray pcl_object_pose_array;
//         pcl_object_pose_array.header.frame_id = target_frame_id_;
//         // pcl_object_pose_array.header.stamp = rclcpp::Time::now();
//         pcl_object_pose_array.header.stamp = rclcpp::Clock().now();
//         pcl_object_pose_array.poses.resize(recognized_cloud_list_.objects.size());
//         std::vector<std::string> pcl_labels;
//         int pcl_count = 0;
//         for (int i=0; i < combined_object_list.objects.size(); i++)
//         {
//             if (combined_object_list.objects[i].database_id < 99)
//             {
//                 names += combined_object_list.objects[i].name + ", ";
//                 pcl_object_pose_array.poses[pcl_count] = combined_object_list.objects[i].pose.pose;
//                 pcl_labels.push_back(combined_object_list.objects[i].name);
//                 pcl_count++;
//             }
//         }
//         RCLCPP_INFO_STREAM(this->get_logger(),"[Cloud] Objects: " << names);
//         // Publish pose array
//         if (pcl_object_pose_array.poses.size() > 0)
//         {
//             pub_pc_object_pose_array_->publish(pcl_object_pose_array);
//         }
//         // Publish label visualizer
//         if ((pcl_labels.size() == pcl_object_pose_array.poses.size()) &&
//             (pcl_labels.size() > 0) && (pcl_object_pose_array.poses.size() > 0))
//         {
//             label_visualizer_pc_.publish(pcl_labels, pcl_object_pose_array);
//         }
//     }
//     if (clusters_2d.size() > 0)
//     {
//         cluster_visualizer_rgb_.publish<PointT>(clusters_2d, target_frame_id_);
//         // RGB Pose array for debug mode only
//         geometry_msgs::msg::PoseArray rgb_object_pose_array;
//         rgb_object_pose_array.header.frame_id = target_frame_id_;
//         rgb_object_pose_array.header.stamp = rclcpp::Clock().now();
//         rgb_object_pose_array.poses.resize(recognized_image_list_.objects.size());
//         std::vector<std::string> rgb_labels;
//         int rgb_count = 0;
//         names = "";
//         for (int i = 0; i < combined_object_list.objects.size(); i++)
//         {
//             if (combined_object_list.objects[i].database_id > 99)
//             {
//                 names += combined_object_list.objects[i].name + ", ";
//                 rgb_object_pose_array.poses[rgb_count] = combined_object_list.objects[i].pose.pose;
//                 rgb_labels.push_back(combined_object_list.objects[i].name);
//                 rgb_count++;
//             }
//         }
//         RCLCPP_INFO_STREAM(this->get_logger(),"[RGB] Objects: "<< names);
//         // Publish pose array
//         if (rgb_object_pose_array.poses.size() > 0)
//         {
//         pub_rgb_object_pose_array_->publish(rgb_object_pose_array);
//         }
//         // Publish label visualizer
//         if ((rgb_labels.size() == rgb_object_pose_array.poses.size()) &&
//             (rgb_labels.size() > 0) && (rgb_object_pose_array.poses.size() > 0))
//         {
//         label_visualizer_rgb_.publish(rgb_labels, rgb_object_pose_array);
//         }
//     }
// }

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

    image_sub_.subscribe(this, "input_image_topic");
    cloud_sub_.subscribe(this, "input_cloud_topic");
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformer/pointcloud",10);
    pub_pc_object_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("output/pc_object_pose_array", 10);
    pub_rgb_object_pose_array_  = this->create_publisher<geometry_msgs::msg::PoseArray>("output/rgb_object_pose_array", 10);

    //msg_sync_.reset(new Sync(msgSyncPolicy(10), image_sub_, cloud_sub_));
    msg_sync_ = std::make_shared<Sync>(msgSyncPolicy(10), image_sub_, cloud_sub_);

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

    image_sub_.unsubscribe();
    cloud_sub_.unsubscribe();

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