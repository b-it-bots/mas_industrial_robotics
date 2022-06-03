#include <std_msgs/msg/float64.hpp>

#include "mir_object_recognition/multimodal_object_recognition.hpp"

#include "mir_perception_utils/clustered_point_clouid_visualizer.hpp"
#include "mir_perception_utils/bounding_box_visualizer.hpp"
#include "mir_perception_utils/label_visualizer.hpp"
#include "mir_perception_utils/bounding_box.hpp"

// just for testing, remove later
#include "mir_perception_utils/planar_polygon_visualizer.hpp"


void MultiModalObjectRecognitionROS::declare_all_parameters(){
    
    rcl_interfaces::msg::ParameterDescriptor descriptor1;
    descriptor1.description = "The size of a leaf (on x,y,z) used for downsampling.";
    rcl_interfaces::msg::FloatingPointRange range1;
    range1.set__from_value(0.0).set__to_value(1.0);
    descriptor1.floating_point_range = {range1};
    this->declare_parameter("voxel_leaf_size", 0.009, descriptor1); 

    rcl_interfaces::msg::ParameterDescriptor descriptor2;
    descriptor2.description = "The field name used for filtering";
    this->declare_parameter("voxel_filter_field_name", "z", descriptor2);

    rcl_interfaces::msg::ParameterDescriptor descriptor3;
    descriptor3.description = "The minimum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range3;
    range3.set__from_value(-10.0).set__to_value(10.0);
    descriptor3.floating_point_range = {range3};
    this->declare_parameter("voxel_filter_limit_min", -0.15, descriptor3);

    rcl_interfaces::msg::ParameterDescriptor descriptor4;
    descriptor4.description = "The maximum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range4;
    range4.set__from_value(-10.0).set__to_value(10.0);
    descriptor4.floating_point_range = {range4};
    this->declare_parameter("voxel_filter_limit_max", 0.25, descriptor4);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor descriptor5;
    descriptor5.description = "Enable passthrough filter";
    this->declare_parameter("enable_passthrough_filter", false, descriptor5);

    rcl_interfaces::msg::ParameterDescriptor descriptor6;
    descriptor6.description = "The field name used for filtering";
    this->declare_parameter("passthrough_filter_field_name", "x", descriptor6);

    rcl_interfaces::msg::ParameterDescriptor descriptor7;
    descriptor7.description = "The minimum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range7;
    range7.set__from_value(-10.0).set__to_value(10.0);
    descriptor7.floating_point_range = {range7};
    this->declare_parameter("passthrough_filter_limit_min", 0.0, descriptor7);

    rcl_interfaces::msg::ParameterDescriptor descriptor8;
    descriptor8.description = "The maximum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range8;
    range8.set__from_value(-10.0).set__to_value(10.0);
    descriptor8.floating_point_range = {range8};
    this->declare_parameter("passthrough_filter_limit_max", 0.8, descriptor8);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor descriptor9;
    descriptor9.description = "Sphere radius for nearest neighbor search";
    rcl_interfaces::msg::FloatingPointRange range9;
    range9.set__from_value(0.0).set__to_value(0.5);
    descriptor9.floating_point_range = {range9};
    this->declare_parameter("normal_radius_search", 0.03, descriptor9);

    rcl_interfaces::msg::ParameterDescriptor descriptor10;
    descriptor10.description = "Use Open MP to estimate normal";
    this->declare_parameter("use_omp", false, descriptor10);

    rcl_interfaces::msg::ParameterDescriptor descriptor11;
    descriptor11.description = "The number of cores to use for OMP normal estimation";
    rcl_interfaces::msg::IntegerRange range11;
    range11.set__from_value(1).set__to_value(16);
    descriptor11.integer_range = {range11};
    this->declare_parameter("num_cores", 4, descriptor11);

    rcl_interfaces::msg::ParameterDescriptor descriptor12;
    descriptor12.description = "The maximum number of iterations the algorithm will run for";
    rcl_interfaces::msg::IntegerRange range12;
    range12.set__from_value(0).set__to_value(100000);
    descriptor12.integer_range = {range12};
    this->declare_parameter("sac_max_iterations", 1000, descriptor12);

    rcl_interfaces::msg::ParameterDescriptor descriptor13;
    descriptor13.description = "The distance to model threshold";
    rcl_interfaces::msg::FloatingPointRange range13;
    range13.set__from_value(0.0).set__to_value(1.0);
    descriptor13.floating_point_range = {range13};
    this->declare_parameter("sac_distance_threshold", 0.01, descriptor13);

    rcl_interfaces::msg::ParameterDescriptor descriptor14;
    descriptor14.description = "Model coefficient refinement";
    this->declare_parameter("sac_optimize_coefficients", true, descriptor14);


    rcl_interfaces::msg::ParameterDescriptor descriptor15;
    descriptor15.description = "The x axis to which the plane should be perpendicular, the eps angle > 0 to activate axis-angle constraint";
    rcl_interfaces::msg::FloatingPointRange range15;
    range15.set__from_value(0.0).set__to_value(1.0);
    descriptor15.floating_point_range = {range15};
    this->declare_parameter("sac_x_axis", 0.0, descriptor15);

    rcl_interfaces::msg::ParameterDescriptor descriptor16;
    descriptor16.description = "The y axis to which the plane should be perpendicular, the eps angle > 0 to activate axis-angle constraint";
    rcl_interfaces::msg::FloatingPointRange range16;
    range16.set__from_value(0.0).set__to_value(1.0);
    descriptor16.floating_point_range = {range16};
    this->declare_parameter("sac_y_axis", 0.0, descriptor16);

    rcl_interfaces::msg::ParameterDescriptor descriptor17;
    descriptor17.description = "The z axis to which the plane should be perpendicular, the eps angle > 0 to activate axis-angle constraint";
    rcl_interfaces::msg::FloatingPointRange range17;
    range17.set__from_value(0.0).set__to_value(1.0);
    descriptor17.floating_point_range = {range17};
    this->declare_parameter("sac_z_axis", 1.0, descriptor17);

    rcl_interfaces::msg::ParameterDescriptor descriptor18;
    descriptor18.description = "The maximum allowed difference between the model normal and the given axis in radians.";
    rcl_interfaces::msg::FloatingPointRange range18;
    range18.set__from_value(0.0).set__to_value(1.5707);
    descriptor18.floating_point_range = {range18};
    this->declare_parameter("sac_eps_angle", 0.09, descriptor18);

    rcl_interfaces::msg::ParameterDescriptor descriptor19;
    descriptor19.description = "The relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal.";
    rcl_interfaces::msg::FloatingPointRange range19;
    range19.set__from_value(0.0).set__to_value(1.0);
    descriptor19.floating_point_range = {range19};
    this->declare_parameter("sac_normal_distance_weight", 0.05, descriptor19);

    rcl_interfaces::msg::ParameterDescriptor descriptor20;
    descriptor20.description = "The minimum height above the plane from which to construct the polygonal prism";
    rcl_interfaces::msg::FloatingPointRange range20;
    range20.set__from_value(0.0).set__to_value(5.0);
    descriptor20.floating_point_range = {range20};
    this->declare_parameter("prism_min_height", 0.01, descriptor20);

    rcl_interfaces::msg::ParameterDescriptor descriptor21;
    descriptor21.description = "The maximum height above the plane from which to construct the polygonal prism";
    rcl_interfaces::msg::FloatingPointRange range21;
    range21.set__from_value(0.0).set__to_value(5.0);
    descriptor21.floating_point_range = {range21};
    this->declare_parameter("prism_max_height", 0.1, descriptor21);

    rcl_interfaces::msg::ParameterDescriptor descriptor22;
    descriptor22.description = "Radius of the sphere that will determine which points are neighbors.";
    rcl_interfaces::msg::FloatingPointRange range22;
    range22.set__from_value(0.0).set__to_value(10.0);
    descriptor22.floating_point_range = {range22};
    this->declare_parameter("outlier_radius_search", 0.03, descriptor22);

    rcl_interfaces::msg::ParameterDescriptor descriptor23;
    descriptor23.description = "The number of neighbors that need to be present in order to be classified as an inlier.";
    rcl_interfaces::msg::IntegerRange range23;
    range23.set__from_value(0).set__to_value(1000);
    descriptor23.integer_range = {range23};
    this->declare_parameter("outlier_min_neighbors", 20, descriptor23);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor descriptor24;
    descriptor24.description = "The spatial tolerance as a measure in the L2 Euclidean space";
    rcl_interfaces::msg::FloatingPointRange range24;
    range24.set__from_value(0.0).set__to_value(2.0);
    descriptor24.floating_point_range = {range24};
    this->declare_parameter("cluster_tolerance", 0.02, descriptor24);

    rcl_interfaces::msg::ParameterDescriptor descriptor25;
    descriptor25.description = "The minimum number of points that a cluster must contain in order to be accepted";
    rcl_interfaces::msg::IntegerRange range25;
    range25.set__from_value(0).set__to_value(1000);
    descriptor25.integer_range = {range25};
    this->declare_parameter("cluster_min_size", 25, descriptor25);

    rcl_interfaces::msg::ParameterDescriptor descriptor26;
    descriptor26.description = "The maximum number of points that a cluster must contain in order to be accepted";
    rcl_interfaces::msg::IntegerRange range26;
    range26.set__from_value(0).set__to_value(2147483647);
    descriptor26.integer_range = {range26};
    this->declare_parameter("cluster_max_size", 20000, descriptor26);

    rcl_interfaces::msg::ParameterDescriptor descriptor27;
    descriptor27.description = "The minimum height of the cluster above the given polygon";
    rcl_interfaces::msg::FloatingPointRange range27;
    range27.set__from_value(0.0).set__to_value(5.0);
    descriptor27.floating_point_range = {range27};
    this->declare_parameter("cluster_min_height", 0.011, descriptor27);

    rcl_interfaces::msg::ParameterDescriptor descriptor28;
    descriptor28.description = "The maximum height of the cluster above the given polygon";
    rcl_interfaces::msg::FloatingPointRange range28;
    range28.set__from_value(0.0).set__to_value(5.0);
    descriptor28.floating_point_range = {range28};
    this->declare_parameter("cluster_max_height", 0.09, descriptor28);

    rcl_interfaces::msg::ParameterDescriptor descriptor29;
    descriptor29.description = "The maximum length of the cluster";
    rcl_interfaces::msg::FloatingPointRange range29;
    range29.set__from_value(0.0).set__to_value(5.0);
    descriptor29.floating_point_range = {range29};
    this->declare_parameter("cluster_max_length", 0.25, descriptor29);

    rcl_interfaces::msg::ParameterDescriptor descriptor30;
    descriptor30.description = "The minimum height of the cluster above the given polygon";
    rcl_interfaces::msg::FloatingPointRange range30;
    range30.set__from_value(0.0).set__to_value(5.0);
    descriptor30.floating_point_range = {range30};
    this->declare_parameter("cluster_min_distance_to_polygon", 0.04, descriptor30);

    rcl_interfaces::msg::ParameterDescriptor descriptor31;
    descriptor31.description = "Center cluster";
    this->declare_parameter("center_cluster", true, descriptor31);

    rcl_interfaces::msg::ParameterDescriptor descriptor32;
    descriptor32.description = "Pad cluster so that it has the same size";
    this->declare_parameter("pad_cluster", false, descriptor32);

    rcl_interfaces::msg::ParameterDescriptor descriptor33;
    descriptor33.description = "The size of the padded cluster";
    rcl_interfaces::msg::IntegerRange range33;
    range33.set__from_value(128).set__to_value(4096);
    descriptor33.integer_range = {range33};
    this->declare_parameter("padded_cluster_size", 2048, descriptor33);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor descriptor34;
    descriptor34.description = "The height of the object above the workspace";
    rcl_interfaces::msg::FloatingPointRange range34;
    range34.set__from_value(0.0).set__to_value(2.0);
    descriptor34.floating_point_range = {range34};
    this->declare_parameter("object_height_above_workspace", 0.052, descriptor34);

    rcl_interfaces::msg::ParameterDescriptor descriptor35;
    descriptor35.description = "The height of the container pose";
    rcl_interfaces::msg::FloatingPointRange range35;
    range35.set__from_value(0.0).set__to_value(2.0);
    descriptor35.floating_point_range = {range35};
    this->declare_parameter("container_height", 0.0335, descriptor35);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor descriptor36;
    descriptor36.description = "RGB bounding box/ROI adjustment in pixel";
    rcl_interfaces::msg::FloatingPointRange range36;
    range36.set__from_value(0.0).set__to_value(50.0);
    descriptor36.floating_point_range = {range36};
    this->declare_parameter("rgb_roi_adjustment", 2.0, descriptor36);

    rcl_interfaces::msg::ParameterDescriptor descriptor37;
    descriptor37.description = "Allowed RGB bounding box min diagonal";
    rcl_interfaces::msg::FloatingPointRange range37;
    range37.set__from_value(0.0).set__to_value(500.0);
    descriptor37.floating_point_range = {range37};
    this->declare_parameter("rgb_bbox_min_diag", 21.0, descriptor37);

    rcl_interfaces::msg::ParameterDescriptor descriptor38;
    descriptor38.description = "Allowed RGB bounding box max diagonal";
    rcl_interfaces::msg::FloatingPointRange range38;
    range38.set__from_value(0.0).set__to_value(500.0);
    descriptor38.floating_point_range = {range38};
    this->declare_parameter("rgb_bbox_max_diag", 200.0, descriptor38);

    rcl_interfaces::msg::ParameterDescriptor descriptor39;
    descriptor39.description = "Passthrough filter min for the generated pc from rgb proposal";
    rcl_interfaces::msg::FloatingPointRange range39;
    range39.set__from_value(-1.0).set__to_value(1.0);
    descriptor39.floating_point_range = {range39};
    this->declare_parameter("rgb_cluster_filter_limit_min", 0.009, descriptor39);

    rcl_interfaces::msg::ParameterDescriptor descriptor40;
    descriptor40.description = "Passthrough filter max for the generated pc from rgb proposal";
    rcl_interfaces::msg::FloatingPointRange range40;
    range40.set__from_value(-1.0).set__to_value(1.0);
    descriptor40.floating_point_range = {range40};
    this->declare_parameter("rgb_cluster_filter_limit_max", 0.35, descriptor40);

    rcl_interfaces::msg::ParameterDescriptor descriptor41;
    descriptor41.description = "Remove cloud cluster generated from RGB ROI";
    this->declare_parameter("rgb_cluster_remove_outliers", true, descriptor41);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor descriptor42;
    descriptor42.description = "Enable ROI filter";
    this->declare_parameter("enable_roi", true, descriptor42);

    rcl_interfaces::msg::ParameterDescriptor descriptor43;
    descriptor43.description = "Base link to laser distance";
    rcl_interfaces::msg::FloatingPointRange range43;
    range43.set__from_value(0.0).set__to_value(1.0);
    descriptor43.floating_point_range = {range43};
    this->declare_parameter("roi_base_link_to_laser_distance", 0.350, descriptor43);

    rcl_interfaces::msg::ParameterDescriptor descriptor44;
    descriptor44.description = "Max object pose x distance to base link";
    rcl_interfaces::msg::FloatingPointRange range44;
    range44.set__from_value(0.0).set__to_value(2.0);
    descriptor44.floating_point_range = {range44};
    this->declare_parameter("roi_max_object_pose_x_to_base_link", 0.650, descriptor44);

    rcl_interfaces::msg::ParameterDescriptor descriptor45;
    descriptor45.description = "Min height of objects";
    rcl_interfaces::msg::FloatingPointRange range45;
    range45.set__from_value(0.0).set__to_value(1.0);
    descriptor45.floating_point_range = {range45};
    this->declare_parameter("roi_min_bbox_z", 0.03, descriptor45);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor descriptor46;
    descriptor46.description = "Enable rgb object detection and recognition";
    this->declare_parameter("enable_rgb_recognizer", true, descriptor46);

    rcl_interfaces::msg::ParameterDescriptor descriptor47;
    descriptor47.description = "Enable pointcloud object detection and recognition";
    this->declare_parameter("enable_pc_recognizer", true, descriptor47);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
    this->declare_parameter("octree_resolution", 0.0025);  
    
}

rcl_interfaces::msg::SetParametersResult 
MultiModalObjectRecognitionROS::parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "success";
      for (const auto &param: parameters)
      {
          RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
          RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());
      }
      return result;
  }

MultiModalObjectRecognitionROS::MultiModalObjectRecognitionROS(const std::string & node_name, bool intra_process_comms):
    rclcpp_lifecycle::LifecycleNode(node_name, 
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        // cluster_visualizer_rgb_("output/tabletop_cluster_rgb", true),
        // cluster_visualizer_pc_("output/tabletop_cluster_pc")
{
    RCLCPP_INFO(get_logger(), "constructor called");
    MultiModalObjectRecognitionROS::declare_all_parameters();
    
}

void MultiModalObjectRecognitionROS::synchronizeCallback(const sensor_msgs::msg::Image &image,
                      const sensor_msgs::msg::PointCloud2 &cloud)
{

    RCLCPP_INFO(get_logger(), "synchro callback");
    RCLCPP_INFO(get_logger(), "TS: [%u]; [%u]", image.header.stamp.sec, cloud.header.stamp.sec);
    sensor_msgs::msg::PointCloud2 transformed_msg;
    this->preprocessPointCloud(tf_listener_, tf_buffer_, target_frame_id_, cloud, transformed_msg);
}

bool MultiModalObjectRecognitionROS::preprocessPointCloud(const std::shared_ptr<tf2_ros::TransformListener> &tf_listener, 
                                                          const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                                                          const std::string target_frame, 
                                                          const sensor_msgs::msg::PointCloud2 cloud_in,
                                                          sensor_msgs::msg::PointCloud2 cloud_out)
{
    RCLCPP_INFO(get_logger(), "preprocess point cloud");
    if (tf_listener) 
        {
            // geometry_msgs::msg::TransformStamped transformStamped;
            try 
            {
                pcl_ros::transformPointCloud(target_frame,cloud_in,cloud_out,*tf_buffer);
                RCLCPP_INFO(this->get_logger(), "Transform throws no error");
                publisher_->publish(cloud_out);
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

    //msg_sync_.reset(new Sync(msgSyncPolicy(10), image_sub_, cloud_sub_));
    msg_sync_ = std::make_shared<Sync>(msgSyncPolicy(10), image_sub_, cloud_sub_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MultiModalObjectRecognitionROS::parametersCallback, this, std::placeholders::_1));

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
    this->remove_on_set_parameters_callback(callback_handle_.get());
    
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