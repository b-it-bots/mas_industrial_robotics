#include <std_msgs/msg/float64.hpp>
#include "mir_object_recognition/multimodal_object_recognition.hpp"
#include "mir_perception_utils/pointcloud_utils_ros.hpp"

void MultiModalObjectRecognitionROS::declare_all_parameters()
{

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
    this->declare_parameter("voxel_filter_limit_max", 0.3, descriptor4);

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

    rcl_interfaces::msg::ParameterDescriptor descr_passthrough_filter_field_y;
    descr_passthrough_filter_field_y.description = "The field name used for filtering";
    this->declare_parameter("passthrough_filter_field_y", "y", descr_passthrough_filter_field_y);

    rcl_interfaces::msg::ParameterDescriptor descr_passthrough_filter_lim_min;
    descr_passthrough_filter_lim_min.description = "The minimum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_passthrough_filter_lim_min;
    range_passthrough_filter_lim_min.set__from_value(-10.0).set__to_value(10.0);
    descr_passthrough_filter_lim_min.floating_point_range = {range_passthrough_filter_lim_min};
    this->declare_parameter("passthrough_filter_y_limit_min", -0.5, descr_passthrough_filter_lim_min);

    rcl_interfaces::msg::ParameterDescriptor descr_passthrough_filter_lim_max;
    descr_passthrough_filter_lim_max.description = "The maximum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_passthrough_filter_lim_max;
    range_passthrough_filter_lim_max.set__from_value(-10.0).set__to_value(10.0);
    descr_passthrough_filter_lim_max.floating_point_range = {range_passthrough_filter_lim_max};
    this->declare_parameter("passthrough_filter_y_limit_max", 0.8, descr_passthrough_filter_lim_max);

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
    this->declare_parameter("num_cores", 8, descriptor11);

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
    this->declare_parameter("center_cluster", false, descriptor31);

    rcl_interfaces::msg::ParameterDescriptor descriptor32;
    descriptor32.description = "Pad cluster so that it has the same size";
    this->declare_parameter("pad_cluster", true, descriptor32);

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
    this->declare_parameter("object_height_above_workspace", 0.022, descriptor34);

    rcl_interfaces::msg::ParameterDescriptor descriptor35;
    descriptor35.description = "The height of the container pose";
    rcl_interfaces::msg::FloatingPointRange range35;
    range35.set__from_value(0.0).set__to_value(2.0);
    descriptor35.floating_point_range = {range35};
    this->declare_parameter("container_height", 0.07, descriptor35);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor descriptor36;
    descriptor36.description = "RGB bounding box/ROI adjustment in pixel";
    rcl_interfaces::msg::IntegerRange range36;
    range36.set__from_value(0).set__to_value(50);
    descriptor36.integer_range = {range36};
    this->declare_parameter("rgb_roi_adjustment", 2, descriptor36);

    rcl_interfaces::msg::ParameterDescriptor descriptor37;
    descriptor37.description = "Allowed RGB bounding box min diagonal";
    rcl_interfaces::msg::IntegerRange range37;
    range37.set__from_value(0).set__to_value(500);
    descriptor37.integer_range = {range37};
    this->declare_parameter("rgb_bbox_min_diag", 21, descriptor37);

    rcl_interfaces::msg::ParameterDescriptor descriptor38;
    descriptor38.description = "Allowed RGB bounding box max diagonal";
    rcl_interfaces::msg::IntegerRange range38;
    range38.set__from_value(0).set__to_value(500);
    descriptor38.integer_range = {range38};
    this->declare_parameter("rgb_bbox_max_diag", 175, descriptor38);

    rcl_interfaces::msg::ParameterDescriptor descriptor39;
    descriptor39.description = "Passthrough filter min for the generated pc from rgb proposal";
    rcl_interfaces::msg::FloatingPointRange range39;
    range39.set__from_value(-1.0).set__to_value(1.0);
    descriptor39.floating_point_range = {range39};
    this->declare_parameter("rgb_cluster_filter_limit_min", 0.0060, descriptor39);

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
    this->declare_parameter("roi_max_object_pose_x_to_base_link", 0.700, descriptor44);

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
    rcl_interfaces::msg::ParameterDescriptor descriptor48;
    descriptor48.description = "Octree resolution";
    rcl_interfaces::msg::FloatingPointRange range48;
    range48.set__from_value(0.0).set__to_value(2.0);
    descriptor48.floating_point_range = {range48};
    this->declare_parameter("octree_resolution", 0.0025, descriptor48);
}

void MultiModalObjectRecognitionROS::get_all_parameters()
{
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
    this->get_parameter("voxel_filter_field_name", voxel_filter_field_name_);
    this->get_parameter("voxel_filter_limit_min", voxel_filter_limit_min_);
    this->get_parameter("voxel_filter_limit_max", voxel_filter_limit_max_);
    this->get_parameter("enable_passthrough_filter", enable_passthrough_filter_);
    this->get_parameter("passthrough_filter_field_name", passthrough_filter_field_name_);
    this->get_parameter("passthrough_filter_limit_min", passthrough_filter_limit_min_);
    this->get_parameter("passthrough_filter_field_y", passthrough_filter_field_y_);
    this->get_parameter("passthrough_filter_limit_max", passthrough_filter_y_limit_max_);
    this->get_parameter("passthrough_filter_y_limit_min", passthrough_filter_y_limit_min_);
    this->get_parameter("passthrough_filter_y_limit_max", passthrough_filter_limit_max_);
    this->get_parameter("normal_radius_search", normal_radius_search_);
    this->get_parameter("use_omp", use_omp_);
    this->get_parameter("num_cores", num_cores_);
    this->get_parameter("sac_max_iterations", sac_max_iterations_);
    this->get_parameter("sac_distance_threshold", sac_distance_threshold_);
    this->get_parameter("sac_optimize_coefficients", sac_optimize_coefficients_);
    this->get_parameter("sac_x_axis", sac_x_axis_);
    this->get_parameter("sac_y_axis", sac_y_axis_);
    this->get_parameter("sac_z_axis", sac_z_axis_);
    this->get_parameter("sac_eps_angle", sac_eps_angle_);
    this->get_parameter("sac_normal_distance_weight", sac_normal_distance_weight_);
    this->get_parameter("prism_min_height", prism_min_height_);
    this->get_parameter("prism_max_height", prism_max_height_);
    this->get_parameter("outlier_radius_search", outlier_radius_search_);
    this->get_parameter("outlier_min_neighbors", outlier_min_neighbors_);
    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    this->get_parameter("cluster_min_size", cluster_min_size_);
    this->get_parameter("cluster_max_size", cluster_max_size_);
    this->get_parameter("cluster_min_height", cluster_min_height_);
    this->get_parameter("cluster_max_height", cluster_max_height_);
    this->get_parameter("cluster_max_length", cluster_max_length_);
    this->get_parameter("cluster_min_distance_to_polygon", cluster_min_distance_to_polygon_);
    this->get_parameter("center_cluster", center_cluster_);
    this->get_parameter("pad_cluster", pad_cluster_);
    this->get_parameter("padded_cluster_size", padded_cluster_size_);
    this->get_parameter("octree_resolution", octree_resolution_);
    this->get_parameter("object_height_above_workspace", object_height_above_workspace_);
    this->get_parameter("container_height", container_height_);
    this->get_parameter("enable_rgb_recognizer", enable_rgb_recognizer_);
    this->get_parameter("enable_pc_recognizer", enable_pc_recognizer_);
    this->get_parameter("rgb_roi_adjustment", rgb_roi_adjustment_);
    this->get_parameter("rgb_bbox_min_diag", rgb_bbox_min_diag_);
    this->get_parameter("rgb_bbox_max_diag", rgb_bbox_max_diag_);
    this->get_parameter("rgb_cluster_filter_limit_min", rgb_cluster_filter_limit_min_);
    this->get_parameter("rgb_cluster_filter_limit_max", rgb_cluster_filter_limit_max_);
    this->get_parameter("rgb_cluster_remove_outliers", rgb_cluster_remove_outliers_);
    this->get_parameter("enable_roi", enable_roi_);
    this->get_parameter("roi_base_link_to_laser_distance", roi_base_link_to_laser_distance_);
    this->get_parameter("roi_max_object_pose_x_to_base_link", roi_max_object_pose_x_to_base_link_);
    this->get_parameter("roi_min_bbox_z", roi_min_bbox_z_);

    scene_segmentation_ros_->setVoxelGridParams(voxel_leaf_size_, voxel_filter_field_name_,
                                                voxel_filter_limit_min_, voxel_filter_limit_max_);
    scene_segmentation_ros_->setPassthroughParams(enable_passthrough_filter_,
                                                  passthrough_filter_field_name_,
                                                  passthrough_filter_limit_min_,
                                                  passthrough_filter_limit_max_,
                                                  passthrough_filter_field_y_,
                                                  passthrough_filter_y_limit_min_,
                                                  passthrough_filter_y_limit_max_);
    scene_segmentation_ros_->setNormalParams(normal_radius_search_, use_omp_, num_cores_);
    Eigen::Vector3f axis(sac_x_axis_, sac_y_axis_, sac_z_axis_);
    scene_segmentation_ros_->setSACParams(sac_max_iterations_, sac_distance_threshold_,
                                          sac_optimize_coefficients_, axis, sac_eps_angle_,
                                          sac_normal_distance_weight_);
    scene_segmentation_ros_->setPrismParams(prism_min_height_, prism_max_height_);
    scene_segmentation_ros_->setOutlierParams(outlier_radius_search_, outlier_min_neighbors_);
    scene_segmentation_ros_->setClusterParams(cluster_tolerance_, cluster_min_size_, cluster_max_size_,
                                              cluster_min_height_, cluster_max_height_, cluster_max_length_,
                                              cluster_min_distance_to_polygon_);
}

rcl_interfaces::msg::SetParametersResult
MultiModalObjectRecognitionROS::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    RCLCPP_INFO(this->get_logger(), "Hello from callabck");

    for (const auto &param : parameters)
    {
        RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());
        if (param.get_name() == "voxel_leaf_size")
        {
            this->voxel_leaf_size_ = param.get_value<double>();
        }
        if (param.get_name() == "voxel_filter_field_name")
        {
            this->voxel_filter_field_name_ = param.get_value<std::string>();
        }
        if (param.get_name() == "voxel_filter_limit_min")
        {
            this->voxel_filter_limit_min_ = param.get_value<double>();
        }
        if (param.get_name() == "voxel_filter_limit_max")
        {
            this->voxel_filter_limit_max_ = param.get_value<double>();
        }
        if (param.get_name() == "enable_passthrough_filter")
        {
            this->enable_passthrough_filter_ = param.get_value<bool>();
        }
        if (param.get_name() == "passthrough_filter_field_name")
        {
            this->passthrough_filter_field_name_ = param.get_value<std::string>();
        }
        if (param.get_name() == "passthrough_filter_limit_min")
        {
            this->passthrough_filter_limit_min_ = param.get_value<double>();
        }
        if (param.get_name() == "passthrough_filter_limit_max")
        {
            this->passthrough_filter_limit_max_ = param.get_value<double>();
        }
        if (param.get_name() == "passthrough_filter_field_y")
        {
            this->passthrough_filter_field_y_ = param.get_value<std::string>();
        }
        if (param.get_name() == "passthrough_filter_y_limit_min")
        {
            this->passthrough_filter_y_limit_min_ = param.get_value<double>();
        }
        if (param.get_name() == "passthrough_filter_y_limit_max")
        {
            this->passthrough_filter_y_limit_max_ = param.get_value<double>();
        }
        if (param.get_name() == "normal_radius_search")
        {
            this->normal_radius_search_ = param.get_value<double>();
        }
        if (param.get_name() == "use_omp")
        {
            this->use_omp_ = param.get_value<bool>();
        }
        if (param.get_name() == "num_cores")
        {
            this->num_cores_ = param.get_value<int>();
        }
        if (param.get_name() == "sac_max_iterations")
        {
            this->sac_max_iterations_ = param.get_value<int>();
        }
        if (param.get_name() == "sac_distance_threshold")
        {
            this->sac_distance_threshold_ = param.get_value<double>();
        }
        if (param.get_name() == "sac_optimize_coefficients")
        {
            this->sac_optimize_coefficients_ = param.get_value<bool>();
        }
        if (param.get_name() == "sac_x_axis")
        {
            this->sac_x_axis_ = param.get_value<bool>();
        }
        if (param.get_name() == "sac_y_axis")
        {
            this->sac_y_axis_ = param.get_value<bool>();
        }
        if (param.get_name() == "sac_z_axis")
        {
            this->sac_z_axis_ = param.get_value<bool>();
        }
        if (param.get_name() == "sac_eps_angle")
        {
            this->sac_eps_angle_ = param.get_value<double>();
        }
        if (param.get_name() == "sac_normal_distance_weight")
        {
            this->sac_normal_distance_weight_ = param.get_value<double>();
        }
        if (param.get_name() == "prism_min_height")
        {
            this->prism_min_height_ = param.get_value<double>();
        }
        if (param.get_name() == "prism_max_height")
        {
            this->prism_max_height_ = param.get_value<double>();
        }
        if (param.get_name() == "outlier_radius_search")
        {
            this->outlier_radius_search_ = param.get_value<double>();
        }
        if (param.get_name() == "outlier_min_neighbors")
        {
            this->outlier_min_neighbors_ = param.get_value<double>();
        }
        if (param.get_name() == "cluster_tolerance")
        {
            this->cluster_tolerance_ = param.get_value<double>();
        }
        if (param.get_name() == "cluster_min_size")
        {
            this->cluster_min_size_ = param.get_value<int>();
        }
        if (param.get_name() == "cluster_max_size")
        {
            this->cluster_max_size_ = param.get_value<int>();
        }
        if (param.get_name() == "cluster_min_height")
        {
            this->cluster_min_height_ = param.get_value<double>();
        }
        if (param.get_name() == "cluster_max_height")
        {
            this->cluster_max_height_ = param.get_value<double>();
        }
        if (param.get_name() == "cluster_max_length")
        {
            this->cluster_max_length_ = param.get_value<double>();
        }
        if (param.get_name() == "cluster_min_distance_to_polygon")
        {
            this->cluster_min_distance_to_polygon_ = param.get_value<double>();
        }
        if (param.get_name() == "center_cluster")
        {
            this->center_cluster_ = param.get_value<bool>();
        }
        if (param.get_name() == "pad_cluster")
        {
            this->pad_cluster_ = param.get_value<bool>();
        }
        if (param.get_name() == "padded_cluster_size")
        {
            this->padded_cluster_size_ = param.get_value<int>();
        }
        if (param.get_name() == "octree_resolution")
        {
            this->octree_resolution_ = param.get_value<double>();
        }
        if (param.get_name() == "object_height_above_workspace")
        {
            this->object_height_above_workspace_ = param.get_value<double>();
        }
        if (param.get_name() == "container_height")
        {
            this->container_height_ = param.get_value<double>();
        }
        if (param.get_name() == "enable_rgb_recognizer")
        {
            this->enable_rgb_recognizer_ = param.get_value<bool>();
        }
        if (param.get_name() == "enable_pc_recognizer")
        {
            this->enable_pc_recognizer_ = param.get_value<bool>();
        }
        if (param.get_name() == "rgb_roi_adjustment")
        {
            this->rgb_roi_adjustment_ = param.get_value<int>();
        }
        if (param.get_name() == "rgb_bbox_min_diag")
        {
            this->rgb_bbox_min_diag_ = param.get_value<int>();
        }
        if (param.get_name() == "rgb_bbox_max_diag")
        {
            this->rgb_bbox_max_diag_ = param.get_value<int>();
        }
        if (param.get_name() == "rgb_cluster_filter_limit_min")
        {
            this->rgb_cluster_filter_limit_min_ = param.get_value<double>();
        }
        if (param.get_name() == "rgb_cluster_filter_limit_max")
        {
            this->rgb_cluster_filter_limit_max_ = param.get_value<double>();
        }
        if (param.get_name() == "rgb_cluster_remove_outliers")
        {
            this->rgb_cluster_remove_outliers_ = param.get_value<bool>();
        }
        if (param.get_name() == "enable_roi")
        {
            this->enable_roi_ = param.get_value<bool>();
        }
        if (param.get_name() == "roi_base_link_to_laser_distance")
        {
            this->roi_base_link_to_laser_distance_ = param.get_value<double>();
        }
        if (param.get_name() == "roi_max_object_pose_x_to_base_link")
        {
            this->roi_max_object_pose_x_to_base_link_ = param.get_value<double>();
        }
        if (param.get_name() == "roi_min_bbox_z")
        {
            this->roi_min_bbox_z_ = param.get_value<double>();
        }
    }

    scene_segmentation_ros_->setVoxelGridParams(voxel_leaf_size_, voxel_filter_field_name_,
                                                voxel_filter_limit_min_, voxel_filter_limit_max_);
    scene_segmentation_ros_->setPassthroughParams(enable_passthrough_filter_,
                                                  passthrough_filter_field_name_,
                                                  passthrough_filter_limit_min_,
                                                  passthrough_filter_limit_max_,
                                                  passthrough_filter_field_y_,
                                                  passthrough_filter_y_limit_min_,
                                                  passthrough_filter_y_limit_max_);
    scene_segmentation_ros_->setNormalParams(normal_radius_search_, use_omp_, num_cores_);
    Eigen::Vector3f axis(sac_x_axis_, sac_y_axis_, sac_z_axis_);
    scene_segmentation_ros_->setSACParams(sac_max_iterations_, sac_distance_threshold_,
                                          sac_optimize_coefficients_, axis, sac_eps_angle_,
                                          sac_normal_distance_weight_);
    scene_segmentation_ros_->setPrismParams(prism_min_height_, prism_max_height_);
    scene_segmentation_ros_->setOutlierParams(outlier_radius_search_, outlier_min_neighbors_);
    scene_segmentation_ros_->setClusterParams(cluster_tolerance_, cluster_min_size_, cluster_max_size_,
                                              cluster_min_height_, cluster_max_height_, cluster_max_length_,
                                              cluster_min_distance_to_polygon_);

    return result;
}

MultiModalObjectRecognitionROS::MultiModalObjectRecognitionROS(const std::string &node_name, bool intra_process_comms) : 
    rclcpp_lifecycle::LifecycleNode(node_name,
                                    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
    cluster_visualizer_rgb_("output/tabletop_cluster_rgb", true),
    cluster_visualizer_pc_("output/tabletop_cluster_pc"),
    received_recognized_image_list_flag_(false),
    received_recognized_cloud_list_flag_(false),
    rgb_object_id_(100),
    enable_rgb_recognizer_(true),
    enable_pc_recognizer_(true),
    rgb_roi_adjustment_(2),
    rgb_cluster_remove_outliers_(true)
{
    RCLCPP_INFO(get_logger(), "constructor called");
    this->declare_parameter<std::string>("target_frame_id", "base_link");
    this->get_parameter("target_frame_id", target_frame_id_);
    this->declare_parameter<bool>("debug_mode_", false);
    this->get_parameter("debug_mode_", debug_mode_);
    this->declare_parameter<std::string>("logdir", "/tmp/");
    this->get_parameter("logdir", logdir_);
    scene_segmentation_ros_ = SceneSegmentationROSSPtr(new SceneSegmentationROS());

    MultiModalObjectRecognitionROS::declare_all_parameters();
}

void MultiModalObjectRecognitionROS::synchronizeCallback(const std::shared_ptr<sensor_msgs::msg::Image> &image,
                                                         const std::shared_ptr<sensor_msgs::msg::PointCloud2> &cloud)
{

    RCLCPP_INFO(get_logger(), "synchro callback");
    RCLCPP_INFO(get_logger(), "TS: [%u]; [%u]", image->header.stamp.sec, cloud->header.stamp.sec);

    pointcloud_msg_ = cloud;
    image_msg_ = image;

    // pre-process the pointcloud
    this->preprocessPointCloud(pointcloud_msg_);
    scene_segmentation_ros_->addCloudAccumulation(cloud_);
    this->recognizeCloudAndImage();
}

void MultiModalObjectRecognitionROS::recognizedImageCallback(const mas_perception_msgs::msg::ObjectList &msg)
{
    RCLCPP_INFO(get_logger(), "Received recognized image callback");
    if (!received_recognized_image_list_flag_)
    {
        recognized_image_list_ = msg;
        received_recognized_image_list_flag_ = true;
    }
}

void MultiModalObjectRecognitionROS::recognizedCloudCallback(const mas_perception_msgs::msg::ObjectList &msg)
{
    RCLCPP_INFO(get_logger(), "Received recognized cloud callback");
    if (!received_recognized_cloud_list_flag_)
    {
        recognized_cloud_list_ = msg;
        received_recognized_cloud_list_flag_ = true;
    }
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
    pc2->header.frame_id = msg_transformed.header.frame_id;

    cloud_ = PointCloudBSPtr(new PointCloud);
    pcl::fromPCLPointCloud2(*pc2, *cloud_);

    RCLCPP_INFO(get_logger(), "Point cloud transformed.");
}

void MultiModalObjectRecognitionROS::segmentPointCloud(mas_perception_msgs::msg::ObjectList &object_list,
                                                       std::vector<PointCloudBSPtr> &clusters,
                                                       std::vector<mpu::object::BoundingBox> boxes)
{
    PointCloudBSPtr cloud = PointCloudBSPtr(new PointCloud);
    cloud->header.frame_id = target_frame_id_;

    scene_segmentation_ros_->getCloudAccumulation(cloud);

    // if the cluster is centered,it looses the correct location of the object
    scene_segmentation_ros_->segmentCloud(cloud, object_list, clusters, boxes,
                                          center_cluster_ = false, pad_cluster_, padded_cluster_size_);

    // get workspace height
    std_msgs::msg::Float64 workspace_height_msg;
    workspace_height_msg.data = scene_segmentation_ros_->getWorkspaceHeight();
    pub_workspace_height_->publish(workspace_height_msg);

    if (debug_mode_)
    {
        PointCloudBSPtr cloud_debug(new PointCloud);
        cloud_debug = scene_segmentation_ros_->getCloudDebug();
        sensor_msgs::msg::PointCloud2 ros_pc2;
        pcl::toROSMsg(*cloud_debug, ros_pc2);
        ros_pc2.header.frame_id = target_frame_id_;
        pub_debug_cloud_plane_->publish(ros_pc2);
    }
}

void MultiModalObjectRecognitionROS::recognizeCloudAndImage()
{
    mas_perception_msgs::msg::ObjectList cloud_object_list;
    std::vector<PointCloudBSPtr> clusters_3d;
    std::vector<mpu::object::BoundingBox> boxes;

    this->segmentPointCloud(cloud_object_list, clusters_3d, boxes);

    if (!cloud_object_list.objects.empty() && enable_rgb_recognizer_)
    {
        // publish the recognized objects
        RCLCPP_INFO_STREAM(get_logger(), "Publishing images for recognition");
        pub_cloud_to_recognizer_->publish(cloud_object_list);
    }

    mas_perception_msgs::msg::ImageList image_list;
    image_list.images.resize(1);
    image_list.images[0] = *image_msg_;
    if (!image_list.images.empty() && enable_rgb_recognizer_)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Publishing images for recognition");
        pub_image_to_recognizer_->publish(image_list);
    }
    RCLCPP_INFO_STREAM(get_logger(), "Waiting for message from Cloud and Image recognizer");
    // loop till it received the message from the 3d and rgb recognition
    int loop_rate_hz = 30;
    int timeout_wait = 2; // secs
    rclcpp::Rate loop_rate(loop_rate_hz);
    int loop_rate_count = 0;
    if (cloud_object_list.objects.size() > 0)
    {
        RCLCPP_INFO_STREAM(get_logger(), "[Cloud] Waiting message from PCL recognizer node");
        while (!received_recognized_cloud_list_flag_)
        {
            loop_rate_count += 1;
            // not sure this will give same result as intended
            rclcpp::spin_some(this->get_node_base_interface());
            loop_rate.sleep();
            if (received_recognized_cloud_list_flag_ == true)
            {
                RCLCPP_INFO(get_logger(), "[Cloud] Received %ld objects from pcl recognizer", recognized_cloud_list_.objects.size());
            }
            if (loop_rate_count > loop_rate_hz * timeout_wait)
            {
                received_recognized_cloud_list_flag_ = false;
                RCLCPP_WARN(get_logger(), "[Cloud] No message received from PCL recognizer.");
                break;
            }
        }
    }

    // Merge recognized_cloud_list and rgb_object_list
    mas_perception_msgs::msg::ObjectList combined_object_list;
    if (!recognized_cloud_list_.objects.empty())
    {
        combined_object_list.objects.insert(combined_object_list.objects.end(),
                                            recognized_cloud_list_.objects.begin(),
                                            recognized_cloud_list_.objects.end());
    }

    loop_rate_count = 0;
    timeout_wait = 3; //  secs

    if (image_list.images.size() > 0)
    {
        RCLCPP_INFO_STREAM(get_logger(), "[RGB] Waiting message from RGB recognizer node");
        while (!received_recognized_image_list_flag_)
        {
            loop_rate_count += 1;
            // not sure this will give same result as intended
            rclcpp::spin_some(this->get_node_base_interface());
            loop_rate.sleep();
            if (received_recognized_image_list_flag_ == true)
            {
                RCLCPP_INFO(get_logger(), "[RGB] Received %d objects from rgb recognizer", (int)(recognized_image_list_.objects.size()));
            }
            if (loop_rate_count > loop_rate_hz * timeout_wait)
            {
                received_recognized_image_list_flag_ = false;
                RCLCPP_WARN(get_logger(), "[RGB] No message received from RGB recognizer.");
                break;
            }
        }
    }

    // Reset recognition callback flags
    received_recognized_cloud_list_flag_ = false;
    received_recognized_image_list_flag_ = false;

    mas_perception_msgs::msg::ObjectList rgb_object_list;
    mas_perception_msgs::msg::BoundingBoxList bounding_boxes;
    std::vector<PointCloudBSPtr> clusters_2d;

    cv_bridge::CvImagePtr cv_image;
    if (recognized_image_list_.objects.size() > 0)
    {
        try
        {
            cv_image = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        bounding_boxes.bounding_boxes.resize(recognized_image_list_.objects.size());
        rgb_object_list.objects.resize(recognized_image_list_.objects.size());

        for (size_t i = 0; i < recognized_image_list_.objects.size(); i++)
        {
            mas_perception_msgs::msg::Object object = recognized_image_list_.objects[i];
            // Check qualitative info of the object
            if (round_objects_.count(recognized_image_list_.objects[i].name))
            {
                object.shape.shape = object.shape.SPHERE;
            }
            else
            {
                object.shape.shape = object.shape.OTHER;
            }
            // Get ROI
            sensor_msgs::msg::RegionOfInterest roi_2d = object.roi;
            const cv::Rect2d rect2d(roi_2d.x_offset, roi_2d.y_offset, roi_2d.width, roi_2d.height);

            if (debug_mode_)
            {
                cv::Point pt1;
                cv::Point pt2;

                pt1.x = roi_2d.x_offset;
                pt1.y = roi_2d.y_offset;
                pt2.x = roi_2d.x_offset + roi_2d.width;
                pt2.y = roi_2d.y_offset + roi_2d.height;

                // draw bbox
                cv::rectangle(cv_image->image, pt1, pt2, cv::Scalar(0, 255, 0), 1, 8, 0);

                // add label
                cv::putText(cv_image->image, object.name, cv::Point(pt1.x, pt2.y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }

            // Remove large 2d misdetected bbox (misdetection)
            double len_diag = sqrt(powf(((roi_2d.width + roi_2d.width) >> 1), 2));
            if (len_diag > rgb_bbox_min_diag_ && len_diag < rgb_bbox_max_diag_)
            {
                PointCloudBSPtr cloud_roi(new PointCloud);
                bool getROISuccess = mpu::pointcloud::getPointCloudROI(roi_2d, cloud_, 
                                                                        cloud_roi,
                                                                        rgb_roi_adjustment_,
                                                                        rgb_cluster_remove_outliers_);
                
                // ToDo: Filter big objects from 2d proposal, if the height is less than 3 mm
                // pcl::PointXYZRGB min_pt;
                // pcl::PointXYZRGB max_pt;
                // pcl::getMinMax3D(*cloud_roi, min_pt, max_pt);
                // float obj_height = max_pt.z - scene_segmentation_ros_->getWorkspaceHeight();

                if (getROISuccess)
                {
                    sensor_msgs::msg::PointCloud2 ros_pc2;
                    PCLPointCloud2BSPtr pc2(new pcl::PCLPointCloud2);
                    pcl::toPCLPointCloud2(*cloud_roi, *pc2);
                    pcl_conversions::fromPCL(*pc2, ros_pc2);
                    ros_pc2.header.frame_id = target_frame_id_;
                    ros_pc2.header.stamp = this->get_clock()->now();

                    clusters_2d.push_back(cloud_roi);
                    
                    // Get pose
                    geometry_msgs::msg::PoseStamped pose;
                    mpu::object::estimatePose(cloud_roi, pose, object.shape.shape, 
                                                rgb_cluster_filter_limit_min_, 
                                                rgb_cluster_filter_limit_max_);

                    // Transform pose
                    std::string frame_id = cloud_ -> header.frame_id;
                    pose.header.stamp = this->get_clock()->now();
                    pose.header.frame_id = frame_id;
                    if (frame_id != target_frame_id_)
                    {
                        mpu::object::transformPose(tf_buffer_, target_frame_id_,
                                                    pose, rgb_object_list.objects[i].pose);
                    }
                    else
                    {
                        rgb_object_list.objects[i].pose = pose;
                    }
                    rgb_object_list.objects[i].probability = recognized_image_list_.objects[i].probability;
                    rgb_object_list.objects[i].database_id = rgb_object_id_;
                    rgb_object_list.objects[i].name = recognized_image_list_.objects[i].name;
                }
                else
                {
                    RCLCPP_DEBUG(get_logger(), "[RGB] DECOY");
                    rgb_object_list.objects[i].name = "DECOY";
                    rgb_object_list.objects[i].database_id = rgb_object_id_;
                }
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "[RGB] DECOY");
                rgb_object_list.objects[i].name = "DECOY";
                rgb_object_list.objects[i].database_id = rgb_object_id_;
            }
            rgb_object_id_++;
        }
        combined_object_list.objects.insert(combined_object_list.objects.end(),
                    rgb_object_list.objects.begin(),
                    rgb_object_list.objects.end());
    }

    if (!combined_object_list.objects.empty())
    {
        if (enable_roi_)
        {
            for (size_t i = 0; i < combined_object_list.objects.size(); i++)
            {
                double current_object_pose_x = combined_object_list.objects[i].pose.pose.position.x;
                if (current_object_pose_x < roi_base_link_to_laser_distance_ ||
                    current_object_pose_x > roi_max_object_pose_x_to_base_link_)
                    /* combined_object_list.objects[i].pose.pose.position.z < scene_segmentation_ros_ */
                    /* ->object_height_above_workspace_ - 0.05) */
                {
                    RCLCPP_WARN_STREAM(get_logger(), "This object " << combined_object_list.objects[i].name << " out of RoI");
                    combined_object_list.objects[i].name = "DECOY";
                }
            }
        }
        // Adjust RPY to make pose flat, adjust container pose
        // Adjust Axis and Bolt pose
        adjustObjectPose(combined_object_list);
        // Publish object to object list merger
        publishObjectList(combined_object_list);
    }
    else
    {
        RCLCPP_WARN(get_logger(), "No object detected to publish");
        return;
    }

    if (debug_mode_)
    {
        RCLCPP_WARN_STREAM(get_logger(), "Debug mode: publishing object information");
        publishDebug(combined_object_list, clusters_3d, clusters_2d);

        rclcpp::Time time_now = this->get_clock()->now();

        // save debug image
        if(recognized_image_list_.objects.size() > 0)
        {
            std::string filename = "";
            filename.append("rgb_debug_");
            filename.append(std::to_string(time_now.seconds()));
            mpu::object::saveCVImage(cv_image, logdir_, filename);
            RCLCPP_INFO_STREAM(get_logger(), "Image:" << filename << " saved to " << logdir_);
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "No Objects found. Cannot save debug image...");
        }

        // Save raw image
        cv_bridge::CvImagePtr raw_cv_image;
        if (mpu::object::getCVImage(image_msg_, raw_cv_image))
        {
            std::string filename = "";
            filename = "";
            filename.append("rgb_raw_");
            filename.append(std::to_string(time_now.seconds()));
            mpu::object::saveCVImage(raw_cv_image, logdir_, filename);
            RCLCPP_INFO_STREAM(get_logger(), "Image:" << filename << " saved to " << logdir_);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Cannot generate cv image...");
        }

        // Save pointcloud debug
        for (auto& cluster : clusters_3d)
        {
            std::string filename = "";
            filename = "";
            filename.append("pcd_cluster_");
            filename.append(std::to_string(time_now.seconds()));
            mpu::object::savePcd(cluster, logdir_, filename);
            RCLCPP_INFO_STREAM(get_logger(), "Point cloud:" << filename << " saved to " << logdir_);
        }
    }

}

void MultiModalObjectRecognitionROS::adjustObjectPose(mas_perception_msgs::msg::ObjectList &object_list)
{

}

void MultiModalObjectRecognitionROS::publishObjectList(mas_perception_msgs::msg::ObjectList &object_list)
{

}

void MultiModalObjectRecognitionROS::publishDebug(mas_perception_msgs::msg::ObjectList &combined_object_list,
                                                std::vector<PointCloudBSPtr> &clusters_3d,
                                                std::vector<PointCloudBSPtr> &clusters_2d)
{

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
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformer/pointcloud", 10);

    // msg_sync_.reset(new Sync(msgSyncPolicy(10), image_sub_, cloud_sub_));
    msg_sync_ = std::make_shared<Sync>(msgSyncPolicy(10), image_sub_, cloud_sub_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MultiModalObjectRecognitionROS::parametersCallback, this, std::placeholders::_1));

    MultiModalObjectRecognitionROS::get_all_parameters();

    // publish workspace height
    pub_workspace_height_ = this->create_publisher<std_msgs::msg::Float64>("workspace_height", 1);

    // publish debug
    pub_debug_cloud_plane_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/debug_cloud_plane", 1);

    // Publish cloud and images to cloud and rgb recognition topics
    pub_cloud_to_recognizer_ = this->create_publisher<mas_perception_msgs::msg::ObjectList>("recognizer/pc/input/object_list", 1);
    pub_image_to_recognizer_ = this->create_publisher<mas_perception_msgs::msg::ImageList>("recognizer/rgb/input/images", 1);

    // Subscribe to cloud and rgb recognition topics
    sub_recognized_image_list_ = this->create_subscription<mas_perception_msgs::msg::ObjectList>(
        "recognizer/rgb/output/object_list", 1, std::bind(&MultiModalObjectRecognitionROS::recognizedImageCallback, this, std::placeholders::_1));

    sub_recognized_cloud_list_ = this->create_subscription<mas_perception_msgs::msg::ObjectList>(
        "recognizer/pc/output/object_list", 1, std::bind(&MultiModalObjectRecognitionROS::recognizedCloudCallback, this, std::placeholders::_1));

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
    pub_workspace_height_->on_activate();
    pub_debug_cloud_plane_->on_activate();
    std::this_thread::sleep_for(2s);

    msg_sync_->registerCallback(&MultiModalObjectRecognitionROS::synchronizeCallback, this);

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

    pub_workspace_height_->on_deactivate();
    pub_debug_cloud_plane_->on_deactivate();

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
MultiModalObjectRecognitionROS::on_shutdown(const rclcpp_lifecycle::State &state)
{
    // In our shutdown phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    // obj_list_pub_.reset();

    pub_workspace_height_->on_deactivate();
    pub_debug_cloud_plane_->on_deactivate();

    image_sub_.unsubscribe();
    cloud_sub_.unsubscribe();

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
int main(int argc, char *argv[])
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
