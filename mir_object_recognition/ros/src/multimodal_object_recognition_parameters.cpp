#include "mir_object_recognition/multimodal_object_recognition.hpp"

namespace perception_namespace
{
void MultiModalObjectRecognitionROS::declare_all_parameters()
{
    this->declare_parameter<std::string>("target_frame_id", "base_link");
    this->get_parameter("target_frame_id", target_frame_id_);
    
    rcl_interfaces::msg::ParameterDescriptor debug_mode_descriptor;
    debug_mode_descriptor.description = "Debug mode";
    debug_mode_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    this->declare_parameter("debug_mode", false, debug_mode_descriptor);
    this->get_parameter("debug_mode", debug_mode_);

    this->declare_parameter<std::string>("logdir", "/tmp/");
    this->get_parameter("logdir", logdir_);

    // get object_info parameter from launch file
    rcl_interfaces::msg::ParameterDescriptor object_info_path_descriptor;
    object_info_path_descriptor.description = "Path to objects.yaml";
    object_info_path_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("objects_info", "", object_info_path_descriptor);
    this->get_parameter<std::string>("objects_info", objects_info_path_);

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

    // Passthrough filter parameters
    rcl_interfaces::msg::ParameterDescriptor descr_enable_passthrough_filter;
    descr_enable_passthrough_filter.description = "Enable passthrough filter";
    this->declare_parameter("enable_passthrough_filter", false, descr_enable_passthrough_filter);

    rcl_interfaces::msg::ParameterDescriptor descr_passthrough_filter_field_name;
    descr_passthrough_filter_field_name.description = "The field name used for filtering";
    this->declare_parameter("passthrough_filter_field_name", "x", descr_passthrough_filter_field_name);

    rcl_interfaces::msg::ParameterDescriptor descr_passthrough_filter_limit_min;
    descr_passthrough_filter_limit_min.description = "The minimum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_passthrough_filter_limit_min;
    range_passthrough_filter_limit_min.set__from_value(-10.0).set__to_value(10.0);
    descr_passthrough_filter_limit_min.floating_point_range = {range_passthrough_filter_limit_min};
    this->declare_parameter("passthrough_filter_limit_min", 0.0, descr_passthrough_filter_limit_min);

    rcl_interfaces::msg::ParameterDescriptor descr_passthrough_filter_limit_max;
    descr_passthrough_filter_limit_max.description = "The maximum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_passthrough_filter_limit_max;
    range_passthrough_filter_limit_max.set__from_value(-10.0).set__to_value(10.0);
    descr_passthrough_filter_limit_max.floating_point_range = {range_passthrough_filter_limit_max};
    this->declare_parameter("passthrough_filter_limit_max", 0.8, descr_passthrough_filter_limit_max);

    // Crop box filter parameters
    rcl_interfaces::msg::ParameterDescriptor descr_enable_cropbox_filter;
    descr_enable_cropbox_filter.description = "Enable crop box filter";
    this->declare_parameter("enable_cropbox_filter", false, descr_enable_cropbox_filter);

    rcl_interfaces::msg::ParameterDescriptor descr_cropbox_filter_x_limit_min;
    descr_cropbox_filter_x_limit_min.description = "The minimum allowed x value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_cropbox_filter_x_limit_min;
    range_cropbox_filter_x_limit_min.set__from_value(-10.0).set__to_value(10.0);
    descr_cropbox_filter_x_limit_min.floating_point_range = {range_cropbox_filter_x_limit_min};
    this->declare_parameter("cropbox_filter_x_limit_min", 0.0, descr_cropbox_filter_x_limit_min);

    rcl_interfaces::msg::ParameterDescriptor descr_cropbox_filter_x_limit_max;
    descr_cropbox_filter_x_limit_max.description = "The maximum allowed x value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_cropbox_filter_x_limit_max;
    range_cropbox_filter_x_limit_max.set__from_value(-10.0).set__to_value(10.0);
    descr_cropbox_filter_x_limit_max.floating_point_range = {range_cropbox_filter_x_limit_max};
    this->declare_parameter("cropbox_filter_x_limit_max", 0.8, descr_cropbox_filter_x_limit_max);

    rcl_interfaces::msg::ParameterDescriptor descr_cropbox_filter_y_limit_min;
    descr_cropbox_filter_y_limit_min.description = "The minimum allowed y value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_cropbox_filter_y_limit_min;
    range_cropbox_filter_y_limit_min.set__from_value(-10.0).set__to_value(10.0);
    descr_cropbox_filter_y_limit_min.floating_point_range = {range_cropbox_filter_y_limit_min};
    this->declare_parameter("cropbox_filter_y_limit_min", -0.4, descr_cropbox_filter_y_limit_min);

    rcl_interfaces::msg::ParameterDescriptor descr_cropbox_filter_y_limit_max;
    descr_cropbox_filter_y_limit_max.description = "The maximum allowed y value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_cropbox_filter_y_limit_max;
    range_cropbox_filter_y_limit_max.set__from_value(-10.0).set__to_value(10.0);
    descr_cropbox_filter_y_limit_max.floating_point_range = {range_cropbox_filter_y_limit_max};
    this->declare_parameter("cropbox_filter_y_limit_max", 0.4, descr_cropbox_filter_y_limit_max);

    rcl_interfaces::msg::ParameterDescriptor descr_cropbox_filter_z_limit_min;
    descr_cropbox_filter_z_limit_min.description = "The minimum allowed z value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_cropbox_filter_z_limit_min;
    range_cropbox_filter_z_limit_min.set__from_value(-10.0).set__to_value(10.0);
    descr_cropbox_filter_z_limit_min.floating_point_range = {range_cropbox_filter_z_limit_min};
    this->declare_parameter("cropbox_filter_z_limit_min", -0.2, descr_cropbox_filter_z_limit_min);

    rcl_interfaces::msg::ParameterDescriptor descr_cropbox_filter_z_limit_max;
    descr_cropbox_filter_z_limit_max.description = "The maximum allowed z value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange range_cropbox_filter_z_limit_max;
    range_cropbox_filter_z_limit_max.set__from_value(-10.0).set__to_value(10.0);
    descr_cropbox_filter_z_limit_max.floating_point_range = {range_cropbox_filter_z_limit_max};
    this->declare_parameter("cropbox_filter_z_limit_max", 0.6, descr_cropbox_filter_z_limit_max);

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
    this->get_parameter("debug_mode", debug_mode_);
    this->get_parameter("target_frame_id", target_frame_id_);
    this->get_parameter("logdir", logdir_);
    this->get_parameter("objects_info", objects_info_path_);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
    this->get_parameter("voxel_filter_field_name", voxel_filter_field_name_);
    this->get_parameter("voxel_filter_limit_min", voxel_filter_limit_min_);
    this->get_parameter("voxel_filter_limit_max", voxel_filter_limit_max_);
    this->get_parameter("enable_passthrough_filter", enable_passthrough_filter_);
    this->get_parameter("passthrough_filter_field_name", passthrough_filter_field_name_);
    this->get_parameter("passthrough_filter_limit_min", passthrough_filter_limit_min_);
    this->get_parameter("passthrough_filter_limit_max", passthrough_filter_limit_max_);
    this->get_parameter("enable_cropbox_filter", enable_cropbox_filter_);
    this->get_parameter("cropbox_filter_x_limit_min", cropbox_filter_x_limit_min_);
    this->get_parameter("cropbox_filter_x_limit_max", cropbox_filter_x_limit_max_);
    this->get_parameter("cropbox_filter_y_limit_min", cropbox_filter_y_limit_min_);
    this->get_parameter("cropbox_filter_y_limit_max", cropbox_filter_y_limit_max_);
    this->get_parameter("cropbox_filter_z_limit_min", cropbox_filter_z_limit_min_);
    this->get_parameter("cropbox_filter_z_limit_max", cropbox_filter_z_limit_max_);
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
    
    // use either passthrough or cropbox filter
    if (enable_passthrough_filter_) {
        scene_segmentation_ros_->setPassthroughParams(enable_passthrough_filter_, passthrough_filter_field_name_,
            passthrough_filter_limit_min_, passthrough_filter_limit_max_);
    } else if (enable_cropbox_filter_) {
        scene_segmentation_ros_->setCropBoxParams(enable_cropbox_filter_, cropbox_filter_x_limit_min_,
            cropbox_filter_x_limit_max_, cropbox_filter_y_limit_min_, cropbox_filter_y_limit_max_,
            cropbox_filter_z_limit_min_, cropbox_filter_z_limit_max_);
    } else if (enable_cropbox_filter_ && enable_passthrough_filter_) {
        RCLCPP_WARN(this->get_logger(), "Both passthrough and cropbox filters are enabled."
                            "Only cropbox filter will take effect. Please disable one of them.");
        scene_segmentation_ros_->setCropBoxParams(enable_cropbox_filter_, cropbox_filter_x_limit_min_,
            cropbox_filter_x_limit_max_, cropbox_filter_y_limit_min_, cropbox_filter_y_limit_max_,
            cropbox_filter_z_limit_min_, cropbox_filter_z_limit_max_);
    }

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
    
    for (const auto &param : parameters)
    {
        RCLCPP_INFO(this->get_logger(), "Value of param %s changed to %s", param.get_name().c_str(), param.value_to_string().c_str());
        if (param.get_name() == "debug_mode")
        {
            this->debug_mode_ = param.get_value<bool>();
        }
        if (param.get_name() == "logdir")
        {
            this->logdir_ = param.get_value<std::string>();
        }
        if (param.get_name() == "target_frame_id")
        {
            this->target_frame_id_ = param.get_value<std::string>();
        }
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
        if (param.get_name() == "enable_cropbox_filter")
        {
            this->enable_cropbox_filter_ = param.get_value<bool>();
        }
        if (param.get_name() == "cropbox_filter_x_limit_min")
        {
            this->cropbox_filter_x_limit_min_ = param.get_value<double>();
        }
        if (param.get_name() == "cropbox_filter_x_limit_max")
        {
            this->cropbox_filter_x_limit_max_ = param.get_value<double>();
        }
        if (param.get_name() == "cropbox_filter_y_limit_min")
        {
            this->cropbox_filter_y_limit_min_ = param.get_value<double>();
        }
        if (param.get_name() == "cropbox_filter_y_limit_max")
        {
            this->cropbox_filter_y_limit_max_ = param.get_value<double>();
        }
        if (param.get_name() == "cropbox_filter_z_limit_min")
        {
            this->cropbox_filter_z_limit_min_ = param.get_value<double>();
        }
        if (param.get_name() == "cropbox_filter_z_limit_max")
        {
            this->cropbox_filter_z_limit_max_ = param.get_value<double>();
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
            this->sac_x_axis_ = param.get_value<double>();
        }
        if (param.get_name() == "sac_y_axis")
        {
            this->sac_y_axis_ = param.get_value<double>();
        }
        if (param.get_name() == "sac_z_axis")
        {
            this->sac_z_axis_ = param.get_value<double>();
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
            this->outlier_min_neighbors_ = param.get_value<int>();
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
    // use either passthrough or cropbox filter
    if (enable_passthrough_filter_) {
        scene_segmentation_ros_->setPassthroughParams(enable_passthrough_filter_, passthrough_filter_field_name_,
            passthrough_filter_limit_min_, passthrough_filter_limit_max_);
    } else if (enable_cropbox_filter_) {
        scene_segmentation_ros_->setCropBoxParams(enable_cropbox_filter_, cropbox_filter_x_limit_min_,
            cropbox_filter_x_limit_max_, cropbox_filter_y_limit_min_, cropbox_filter_y_limit_max_,
            cropbox_filter_z_limit_min_, cropbox_filter_z_limit_max_);
    } else if (enable_cropbox_filter_ && enable_passthrough_filter_) {
        RCLCPP_WARN(this->get_logger(), "Both passthrough and cropbox filters are enabled."
                            "Only cropbox filter will take effect. Please disable one of them.");
        scene_segmentation_ros_->setCropBoxParams(enable_cropbox_filter_, cropbox_filter_x_limit_min_,
            cropbox_filter_x_limit_max_, cropbox_filter_y_limit_min_, cropbox_filter_y_limit_max_,
            cropbox_filter_z_limit_min_, cropbox_filter_z_limit_max_);
    }
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


} //end of namespace




// RCLCPP_COMPONENTS_REGISTER_NODE(perception_namespace::MultiModalObjectRecognitionROS)