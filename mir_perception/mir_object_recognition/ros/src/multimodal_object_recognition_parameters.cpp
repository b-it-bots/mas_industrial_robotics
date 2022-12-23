/*
 * Copyright 2022 Bonn-Rhein-Sieg University
 *
 * Author: Shubham Shinde, Vamsi Kalagaturu.
 *
 */

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
    this->declare_parameter("debug_mode", true, debug_mode_descriptor);
    this->get_parameter("debug_mode", debug_mode_);

    // get object_info parameter from launch file
    rcl_interfaces::msg::ParameterDescriptor object_info_path_descriptor;
    object_info_path_descriptor.description = "Path to objects.yaml";
    object_info_path_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("objects_info", "", object_info_path_descriptor);
    this->get_parameter<std::string>("objects_info", objects_info_path_);

    // get yolo_classes_info parameter from launch file
    rcl_interfaces::msg::ParameterDescriptor yolo_classes_info_path_descriptor;
    yolo_classes_info_path_descriptor.description = "Path to yolo_classes_info (.names) file";
    yolo_classes_info_path_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("yolo_classes_info", "", yolo_classes_info_path_descriptor);
    this->get_parameter<std::string>("yolo_classes_info", yolo_classes_info_path_);

    // get yolo_weights parameter from launch file
    rcl_interfaces::msg::ParameterDescriptor yolo_weights_path_descriptor;
    yolo_weights_path_descriptor.description = "Path to yolo weights (.onnx) file";
    yolo_weights_path_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("yolo_weights", "", yolo_weights_path_descriptor);
    this->get_parameter<std::string>("yolo_weights", yolo_weights_path_);

    rcl_interfaces::msg::ParameterDescriptor voxel_leaf_size_descriptor;
    voxel_leaf_size_descriptor.description = "The size of a leaf (on x,y,z) used for downsampling.";
    rcl_interfaces::msg::FloatingPointRange voxel_leaf_size_range;
    voxel_leaf_size_range.set__from_value(0.0).set__to_value(1.0);
    voxel_leaf_size_descriptor.floating_point_range = {voxel_leaf_size_range};
    this->declare_parameter("voxel_leaf_size", 0.009, voxel_leaf_size_descriptor);

    rcl_interfaces::msg::ParameterDescriptor voxel_filter_field_name_descriptor;
    voxel_filter_field_name_descriptor.description = "The field name used for filtering";
    this->declare_parameter("voxel_filter_field_name", "z", voxel_filter_field_name_descriptor);

    rcl_interfaces::msg::ParameterDescriptor voxel_filter_limit_min_descriptor;
    voxel_filter_limit_min_descriptor.description = "The minimum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange voxel_filter_limit_min_range;
    voxel_filter_limit_min_range.set__from_value(-10.0).set__to_value(10.0);
    voxel_filter_limit_min_descriptor.floating_point_range = {voxel_filter_limit_min_range};
    this->declare_parameter("voxel_filter_limit_min", -0.15, voxel_filter_limit_min_descriptor);

    rcl_interfaces::msg::ParameterDescriptor voxel_filter_limit_max_descriptor;
    voxel_filter_limit_max_descriptor.description = "The maximum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange voxel_filter_limit_max_range;
    voxel_filter_limit_max_range.set__from_value(-10.0).set__to_value(10.0);
    voxel_filter_limit_max_descriptor.floating_point_range = {voxel_filter_limit_max_range};
    this->declare_parameter("voxel_filter_limit_max", 0.3, voxel_filter_limit_max_descriptor);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Passthrough filter parameters
    rcl_interfaces::msg::ParameterDescriptor enable_passthrough_filter_descriptor;
    enable_passthrough_filter_descriptor.description = "Enable passthrough filter";
    this->declare_parameter("enable_passthrough_filter", false, enable_passthrough_filter_descriptor);

    rcl_interfaces::msg::ParameterDescriptor passthrough_filter_field_name_descriptor;
    passthrough_filter_field_name_descriptor.description = "The field name used for filtering";
    this->declare_parameter("passthrough_filter_field_name", "x", passthrough_filter_field_name_descriptor);

    rcl_interfaces::msg::ParameterDescriptor passthrough_filter_limit_min_descriptor;
    passthrough_filter_limit_min_descriptor.description = "The minimum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange passthrough_filter_limit_min_range;
    passthrough_filter_limit_min_range.set__from_value(-10.0).set__to_value(10.0);
    passthrough_filter_limit_min_descriptor.floating_point_range = {passthrough_filter_limit_min_range};
    this->declare_parameter("passthrough_filter_limit_min", 0.0, passthrough_filter_limit_min_descriptor);

    rcl_interfaces::msg::ParameterDescriptor passthrough_filter_limit_max_descriptor;
    passthrough_filter_limit_max_descriptor.description = "The maximum allowed field value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange passthrough_filter_limit_max_range;
    passthrough_filter_limit_max_range.set__from_value(-10.0).set__to_value(10.0);
    passthrough_filter_limit_max_descriptor.floating_point_range = {passthrough_filter_limit_max_range};
    this->declare_parameter("passthrough_filter_limit_max", 0.8, passthrough_filter_limit_max_descriptor);

    // Crop box filter parameters
    rcl_interfaces::msg::ParameterDescriptor enable_cropbox_filter_descriptor;
    enable_cropbox_filter_descriptor.description = "Enable crop box filter";
    this->declare_parameter("enable_cropbox_filter", true, enable_cropbox_filter_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cropbox_filter_x_limit_min_descriptor;
    cropbox_filter_x_limit_min_descriptor.description = "The minimum allowed x value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange cropbox_filter_x_limit_min_range;
    cropbox_filter_x_limit_min_range.set__from_value(-10.0).set__to_value(10.0);
    cropbox_filter_x_limit_min_descriptor.floating_point_range = {cropbox_filter_x_limit_min_range};
    this->declare_parameter("cropbox_filter_x_limit_min", 0.0, cropbox_filter_x_limit_min_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cropbox_filter_x_limit_max_descriptor;
    cropbox_filter_x_limit_max_descriptor.description = "The maximum allowed x value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange cropbox_filter_x_limit_max_range;
    cropbox_filter_x_limit_max_range.set__from_value(-10.0).set__to_value(10.0);
    cropbox_filter_x_limit_max_descriptor.floating_point_range = {cropbox_filter_x_limit_max_range};
    this->declare_parameter("cropbox_filter_x_limit_max", 0.8, cropbox_filter_x_limit_max_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cropbox_filter_y_limit_min_descriptor;
    cropbox_filter_y_limit_min_descriptor.description = "The minimum allowed y value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange cropbox_filter_y_limit_min_range;
    cropbox_filter_y_limit_min_range.set__from_value(-10.0).set__to_value(10.0);
    cropbox_filter_y_limit_min_descriptor.floating_point_range = {cropbox_filter_y_limit_min_range};
    this->declare_parameter("cropbox_filter_y_limit_min", -0.4, cropbox_filter_y_limit_min_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cropbox_filter_y_limit_max_descriptor;
    cropbox_filter_y_limit_max_descriptor.description = "The maximum allowed y value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange cropbox_filter_y_limit_max_range;
    cropbox_filter_y_limit_max_range.set__from_value(-10.0).set__to_value(10.0);
    cropbox_filter_y_limit_max_descriptor.floating_point_range = {cropbox_filter_y_limit_max_range};
    this->declare_parameter("cropbox_filter_y_limit_max", 0.4, cropbox_filter_y_limit_max_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cropbox_filter_z_limit_min_descriptor;
    cropbox_filter_z_limit_min_descriptor.description = "The minimum allowed z value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange cropbox_filter_z_limit_min_range;
    cropbox_filter_z_limit_min_range.set__from_value(-10.0).set__to_value(10.0);
    cropbox_filter_z_limit_min_descriptor.floating_point_range = {cropbox_filter_z_limit_min_range};
    this->declare_parameter("cropbox_filter_z_limit_min", -0.2, cropbox_filter_z_limit_min_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cropbox_filter_z_limit_max_descriptor;
    cropbox_filter_z_limit_max_descriptor.description = "The maximum allowed z value a point will be considered from";
    rcl_interfaces::msg::FloatingPointRange cropbox_filter_z_limit_max_range;
    cropbox_filter_z_limit_max_range.set__from_value(-10.0).set__to_value(10.0);
    cropbox_filter_z_limit_max_descriptor.floating_point_range = {cropbox_filter_z_limit_max_range};
    this->declare_parameter("cropbox_filter_z_limit_max", 0.6, cropbox_filter_z_limit_max_descriptor);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor normal_radius_search_descriptor;
    normal_radius_search_descriptor.description = "Sphere radius for nearest neighbor search";
    rcl_interfaces::msg::FloatingPointRange normal_radius_search_range;
    normal_radius_search_range.set__from_value(0.0).set__to_value(0.5);
    normal_radius_search_descriptor.floating_point_range = {normal_radius_search_range};
    this->declare_parameter("normal_radius_search", 0.03, normal_radius_search_descriptor);

    rcl_interfaces::msg::ParameterDescriptor use_omp_descriptor;
    use_omp_descriptor.description = "Use Open MP to estimate normal";
    this->declare_parameter("use_omp", false, use_omp_descriptor);

    rcl_interfaces::msg::ParameterDescriptor num_cores_descriptor;
    num_cores_descriptor.description = "The number of cores to use for OMP normal estimation";
    rcl_interfaces::msg::IntegerRange num_cores_range;
    num_cores_range.set__from_value(1).set__to_value(16);
    num_cores_descriptor.integer_range = {num_cores_range};
    this->declare_parameter("num_cores", 8, num_cores_descriptor);

    rcl_interfaces::msg::ParameterDescriptor sac_max_iterations_descriptor;
    sac_max_iterations_descriptor.description = "The maximum number of iterations the algorithm will run for";
    rcl_interfaces::msg::IntegerRange sac_max_iterations_range;
    sac_max_iterations_range.set__from_value(0).set__to_value(100000);
    sac_max_iterations_descriptor.integer_range = {sac_max_iterations_range};
    this->declare_parameter("sac_max_iterations", 1000, sac_max_iterations_descriptor);

    rcl_interfaces::msg::ParameterDescriptor sac_distance_threshold_descriptor;
    sac_distance_threshold_descriptor.description = "The distance to model threshold";
    rcl_interfaces::msg::FloatingPointRange sac_distance_threshold_range;
    sac_distance_threshold_range.set__from_value(0.0).set__to_value(1.0);
    sac_distance_threshold_descriptor.floating_point_range = {sac_distance_threshold_range};
    this->declare_parameter("sac_distance_threshold", 0.01, sac_distance_threshold_descriptor);

    rcl_interfaces::msg::ParameterDescriptor sac_optimize_coefficients_descriptor;
    sac_optimize_coefficients_descriptor.description = "Model coefficient refinement";
    this->declare_parameter("sac_optimize_coefficients", true, sac_optimize_coefficients_descriptor);

    rcl_interfaces::msg::ParameterDescriptor sac_x_axis_descriptor;
    sac_x_axis_descriptor.description = "The x axis to which the plane should be perpendicular, the eps angle > 0 to activate axis-angle constraint";
    rcl_interfaces::msg::FloatingPointRange sac_x_axis_range;
    sac_x_axis_range.set__from_value(0.0).set__to_value(1.0);
    sac_x_axis_descriptor.floating_point_range = {sac_x_axis_range};
    this->declare_parameter("sac_x_axis", 0.0, sac_x_axis_descriptor);

    rcl_interfaces::msg::ParameterDescriptor sac_y_axis_descriptor;
    sac_y_axis_descriptor.description = "The y axis to which the plane should be perpendicular, the eps angle > 0 to activate axis-angle constraint";
    rcl_interfaces::msg::FloatingPointRange sac_y_axis_range;
    sac_y_axis_range.set__from_value(0.0).set__to_value(1.0);
    sac_y_axis_descriptor.floating_point_range = {sac_y_axis_range};
    this->declare_parameter("sac_y_axis", 0.0, sac_y_axis_descriptor);

    rcl_interfaces::msg::ParameterDescriptor sac_z_axis_descriptor;
    sac_z_axis_descriptor.description = "The z axis to which the plane should be perpendicular, the eps angle > 0 to activate axis-angle constraint";
    rcl_interfaces::msg::FloatingPointRange sac_z_axis_range;
    sac_z_axis_range.set__from_value(0.0).set__to_value(1.0);
    sac_z_axis_descriptor.floating_point_range = {sac_z_axis_range};
    this->declare_parameter("sac_z_axis", 1.0, sac_z_axis_descriptor);

    rcl_interfaces::msg::ParameterDescriptor sac_eps_angle_descriptor;
    sac_eps_angle_descriptor.description = "The maximum allowed difference between the model normal and the given axis in radians.";
    rcl_interfaces::msg::FloatingPointRange sac_eps_angle_range;
    sac_eps_angle_range.set__from_value(0.0).set__to_value(1.5707);
    sac_eps_angle_descriptor.floating_point_range = {sac_eps_angle_range};
    this->declare_parameter("sac_eps_angle", 0.09, sac_eps_angle_descriptor);

    rcl_interfaces::msg::ParameterDescriptor sac_normal_distance_weight_descriptor;
    sac_normal_distance_weight_descriptor.description = "The relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal.";
    rcl_interfaces::msg::FloatingPointRange sac_normal_distance_weight_range;
    sac_normal_distance_weight_range.set__from_value(0.0).set__to_value(1.0);
    sac_normal_distance_weight_descriptor.floating_point_range = {sac_normal_distance_weight_range};
    this->declare_parameter("sac_normal_distance_weight", 0.05, sac_normal_distance_weight_descriptor);

    rcl_interfaces::msg::ParameterDescriptor prism_min_height_descriptor;
    prism_min_height_descriptor.description = "The minimum height above the plane from which to construct the polygonal prism";
    rcl_interfaces::msg::FloatingPointRange prism_min_height_range;
    prism_min_height_range.set__from_value(0.0).set__to_value(5.0);
    prism_min_height_descriptor.floating_point_range = {prism_min_height_range};
    this->declare_parameter("prism_min_height", 0.01, prism_min_height_descriptor);

    rcl_interfaces::msg::ParameterDescriptor prism_max_height_descriptor;
    prism_max_height_descriptor.description = "The maximum height above the plane from which to construct the polygonal prism";
    rcl_interfaces::msg::FloatingPointRange prism_max_height_range;
    prism_max_height_range.set__from_value(0.0).set__to_value(5.0);
    prism_max_height_descriptor.floating_point_range = {prism_max_height_range};
    this->declare_parameter("prism_max_height", 0.1, prism_max_height_descriptor);

    rcl_interfaces::msg::ParameterDescriptor outlier_radius_search_descriptor;
    outlier_radius_search_descriptor.description = "Radius of the sphere that will determine which points are neighbors.";
    rcl_interfaces::msg::FloatingPointRange outlier_radius_search_range;
    outlier_radius_search_range.set__from_value(0.0).set__to_value(10.0);
    outlier_radius_search_descriptor.floating_point_range = {outlier_radius_search_range};
    this->declare_parameter("outlier_radius_search", 0.03, outlier_radius_search_descriptor);

    rcl_interfaces::msg::ParameterDescriptor outlier_min_neighbors_descriptor;
    outlier_min_neighbors_descriptor.description = "The number of neighbors that need to be present in order to be classified as an inlier.";
    rcl_interfaces::msg::IntegerRange outlier_min_neighbors_range;
    outlier_min_neighbors_range.set__from_value(0).set__to_value(1000);
    outlier_min_neighbors_descriptor.integer_range = {outlier_min_neighbors_range};
    this->declare_parameter("outlier_min_neighbors", 20, outlier_min_neighbors_descriptor);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor cluster_tolerance_descriptor;
    cluster_tolerance_descriptor.description = "The spatial tolerance as a measure in the L2 Euclidean space";
    rcl_interfaces::msg::FloatingPointRange cluster_tolerance_range;
    cluster_tolerance_range.set__from_value(0.0).set__to_value(2.0);
    cluster_tolerance_descriptor.floating_point_range = {cluster_tolerance_range};
    this->declare_parameter("cluster_tolerance", 0.02, cluster_tolerance_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cluster_min_size_descriptor;
    cluster_min_size_descriptor.description = "The minimum number of points that a cluster must contain in order to be accepted";
    rcl_interfaces::msg::IntegerRange cluster_min_size_range;
    cluster_min_size_range.set__from_value(0).set__to_value(1000);
    cluster_min_size_descriptor.integer_range = {cluster_min_size_range};
    this->declare_parameter("cluster_min_size", 25, cluster_min_size_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cluster_max_size_descriptor;
    cluster_max_size_descriptor.description = "The maximum number of points that a cluster must contain in order to be accepted";
    rcl_interfaces::msg::IntegerRange cluster_max_size_range;
    cluster_max_size_range.set__from_value(0).set__to_value(2147483647);
    cluster_max_size_descriptor.integer_range = {cluster_max_size_range};
    this->declare_parameter("cluster_max_size", 20000, cluster_max_size_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cluster_min_height_descriptor;
    cluster_min_height_descriptor.description = "The minimum height of the cluster above the given polygon";
    rcl_interfaces::msg::FloatingPointRange cluster_min_height_range;
    cluster_min_height_range.set__from_value(0.0).set__to_value(5.0);
    cluster_min_height_descriptor.floating_point_range = {cluster_min_height_range};
    this->declare_parameter("cluster_min_height", 0.011, cluster_min_height_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cluster_max_height_descriptor;
    cluster_max_height_descriptor.description = "The maximum height of the cluster above the given polygon";
    rcl_interfaces::msg::FloatingPointRange cluster_max_height_range;
    cluster_max_height_range.set__from_value(0.0).set__to_value(5.0);
    cluster_max_height_descriptor.floating_point_range = {cluster_max_height_range};
    this->declare_parameter("cluster_max_height", 0.09, cluster_max_height_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cluster_max_length_descriptor;
    cluster_max_length_descriptor.description = "The maximum length of the cluster";
    rcl_interfaces::msg::FloatingPointRange cluster_max_length_range;
    cluster_max_length_range.set__from_value(0.0).set__to_value(5.0);
    cluster_max_length_descriptor.floating_point_range = {cluster_max_length_range};
    this->declare_parameter("cluster_max_length", 0.25, cluster_max_length_descriptor);

    rcl_interfaces::msg::ParameterDescriptor cluster_min_distance_to_polygon_descriptor;
    cluster_min_distance_to_polygon_descriptor.description = "The minimum height of the cluster above the given polygon";
    rcl_interfaces::msg::FloatingPointRange cluster_min_distance_to_polygon_range;
    cluster_min_distance_to_polygon_range.set__from_value(0.0).set__to_value(5.0);
    cluster_min_distance_to_polygon_descriptor.floating_point_range = {cluster_min_distance_to_polygon_range};
    this->declare_parameter("cluster_min_distance_to_polygon", 0.04, cluster_min_distance_to_polygon_descriptor);

    rcl_interfaces::msg::ParameterDescriptor center_cluster_descriptor;
    center_cluster_descriptor.description = "Center cluster";
    this->declare_parameter("center_cluster", false, center_cluster_descriptor);

    rcl_interfaces::msg::ParameterDescriptor pad_cluster_descriptor;
    pad_cluster_descriptor.description = "Pad cluster so that it has the same size";
    this->declare_parameter("pad_cluster", true, pad_cluster_descriptor);

    rcl_interfaces::msg::ParameterDescriptor padded_cluster_size_descriptor;
    padded_cluster_size_descriptor.description = "The size of the padded cluster";
    rcl_interfaces::msg::IntegerRange padded_cluster_size_range;
    padded_cluster_size_range.set__from_value(128).set__to_value(4096);
    padded_cluster_size_descriptor.integer_range = {padded_cluster_size_range};
    this->declare_parameter("padded_cluster_size", 2048, padded_cluster_size_descriptor);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor object_height_above_workspace_descriptor;
    object_height_above_workspace_descriptor.description = "The height of the object above the workspace";
    rcl_interfaces::msg::FloatingPointRange object_height_above_workspace_range;
    object_height_above_workspace_range.set__from_value(0.0).set__to_value(2.0);
    object_height_above_workspace_descriptor.floating_point_range = {object_height_above_workspace_range};
    this->declare_parameter("object_height_above_workspace", 0.022, object_height_above_workspace_descriptor);

    rcl_interfaces::msg::ParameterDescriptor container_height_descriptor;
    container_height_descriptor.description = "The height of the container pose";
    rcl_interfaces::msg::FloatingPointRange container_height_range;
    container_height_range.set__from_value(0.0).set__to_value(2.0);
    container_height_descriptor.floating_point_range = {container_height_range};
    this->declare_parameter("container_height", 0.07, container_height_descriptor);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor rgb_roi_adjustment_descriptor;
    rgb_roi_adjustment_descriptor.description = "RGB bounding box/ROI adjustment in pixel";
    rcl_interfaces::msg::IntegerRange rgb_roi_adjustment_range;
    rgb_roi_adjustment_range.set__from_value(0).set__to_value(50);
    rgb_roi_adjustment_descriptor.integer_range = {rgb_roi_adjustment_range};
    this->declare_parameter("rgb_roi_adjustment", 2, rgb_roi_adjustment_descriptor);

    rcl_interfaces::msg::ParameterDescriptor rgb_bbox_min_diag_descriptor;
    rgb_bbox_min_diag_descriptor.description = "Allowed RGB bounding box min diagonal";
    rcl_interfaces::msg::IntegerRange rgb_bbox_min_diag_range;
    rgb_bbox_min_diag_range.set__from_value(0).set__to_value(500);
    rgb_bbox_min_diag_descriptor.integer_range = {rgb_bbox_min_diag_range};
    this->declare_parameter("rgb_bbox_min_diag", 21, rgb_bbox_min_diag_descriptor);

    rcl_interfaces::msg::ParameterDescriptor rgb_bbox_max_diag_descriptor;
    rgb_bbox_max_diag_descriptor.description = "Allowed RGB bounding box max diagonal";
    rcl_interfaces::msg::IntegerRange rgb_bbox_max_diag_range;
    rgb_bbox_max_diag_range.set__from_value(0).set__to_value(500);
    rgb_bbox_max_diag_descriptor.integer_range = {rgb_bbox_max_diag_range};
    this->declare_parameter("rgb_bbox_max_diag", 175, rgb_bbox_max_diag_descriptor);

    rcl_interfaces::msg::ParameterDescriptor rgb_cluster_filter_limit_min_descriptor;
    rgb_cluster_filter_limit_min_descriptor.description = "Passthrough filter min for the generated pc from rgb proposal";
    rcl_interfaces::msg::FloatingPointRange rgb_cluster_filter_limit_min_range;
    rgb_cluster_filter_limit_min_range.set__from_value(-1.0).set__to_value(1.0);
    rgb_cluster_filter_limit_min_descriptor.floating_point_range = {rgb_cluster_filter_limit_min_range};
    this->declare_parameter("rgb_cluster_filter_limit_min", 0.0060, rgb_cluster_filter_limit_min_descriptor);

    rcl_interfaces::msg::ParameterDescriptor rgb_cluster_filter_limit_max_descriptor;
    rgb_cluster_filter_limit_max_descriptor.description = "Passthrough filter max for the generated pc from rgb proposal";
    rcl_interfaces::msg::FloatingPointRange rgb_cluster_filter_limit_max_range;
    rgb_cluster_filter_limit_max_range.set__from_value(-1.0).set__to_value(1.0);
    rgb_cluster_filter_limit_max_descriptor.floating_point_range = {rgb_cluster_filter_limit_max_range};
    this->declare_parameter("rgb_cluster_filter_limit_max", 0.35, rgb_cluster_filter_limit_max_descriptor);

    rcl_interfaces::msg::ParameterDescriptor rgb_cluster_remove_outliers_descriptor;
    rgb_cluster_remove_outliers_descriptor.description = "Remove cloud cluster generated from RGB ROI";
    this->declare_parameter("rgb_cluster_remove_outliers", true, rgb_cluster_remove_outliers_descriptor);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor enable_roi_descriptor;
    enable_roi_descriptor.description = "Enable ROI filter";
    this->declare_parameter("enable_roi", false, enable_roi_descriptor);

    rcl_interfaces::msg::ParameterDescriptor roi_base_link_to_laser_distance_descriptor;
    roi_base_link_to_laser_distance_descriptor.description = "Base link to laser distance";
    rcl_interfaces::msg::FloatingPointRange roi_base_link_to_laser_distance_range;
    roi_base_link_to_laser_distance_range.set__from_value(0.0).set__to_value(1.0);
    roi_base_link_to_laser_distance_descriptor.floating_point_range = {roi_base_link_to_laser_distance_range};
    this->declare_parameter("roi_base_link_to_laser_distance", 0.350, roi_base_link_to_laser_distance_descriptor);

    rcl_interfaces::msg::ParameterDescriptor roi_max_object_pose_x_to_base_link_descriptor;
    roi_max_object_pose_x_to_base_link_descriptor.description = "Max object pose x distance to base link";
    rcl_interfaces::msg::FloatingPointRange roi_max_object_pose_x_to_base_link_range;
    roi_max_object_pose_x_to_base_link_range.set__from_value(0.0).set__to_value(2.0);
    roi_max_object_pose_x_to_base_link_descriptor.floating_point_range = {roi_max_object_pose_x_to_base_link_range};
    this->declare_parameter("roi_max_object_pose_x_to_base_link", 0.700, roi_max_object_pose_x_to_base_link_descriptor);

    rcl_interfaces::msg::ParameterDescriptor roi_min_bbox_z_descriptor;
    roi_min_bbox_z_descriptor.description = "Min height of objects";
    rcl_interfaces::msg::FloatingPointRange roi_min_bbox_z_range;
    roi_min_bbox_z_range.set__from_value(0.0).set__to_value(1.0);
    roi_min_bbox_z_descriptor.floating_point_range = {roi_min_bbox_z_range};
    this->declare_parameter("roi_min_bbox_z", 0.03, roi_min_bbox_z_descriptor);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rcl_interfaces::msg::ParameterDescriptor enable_rgb_recognizer_descriptor;
    enable_rgb_recognizer_descriptor.description = "Enable rgb object detection and recognition";
    this->declare_parameter("enable_rgb_recognizer", true, enable_rgb_recognizer_descriptor);

    rcl_interfaces::msg::ParameterDescriptor enable_pc_recognizer_descriptor;
    enable_pc_recognizer_descriptor.description = "Enable pointcloud object detection and recognition";
    this->declare_parameter("enable_pc_recognizer", false, enable_pc_recognizer_descriptor);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    rcl_interfaces::msg::ParameterDescriptor octree_resolution_descriptor;
    octree_resolution_descriptor.description = "Octree resolution";
    rcl_interfaces::msg::FloatingPointRange octree_resolution_range;
    octree_resolution_range.set__from_value(0.0).set__to_value(2.0);
    octree_resolution_descriptor.floating_point_range = {octree_resolution_range};
    this->declare_parameter("octree_resolution", 0.0025, octree_resolution_descriptor);
  }

  void MultiModalObjectRecognitionROS::get_all_parameters()
  {
    this->get_parameter("debug_mode", debug_mode_);
    this->get_parameter("target_frame_id", target_frame_id_);
    this->get_parameter("objects_info", objects_info_path_);
    this->get_parameter("yolo_classes_info", yolo_classes_info_path_);
    this->get_parameter("yolo_weights", yolo_weights_path_);
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
    if (enable_passthrough_filter_)
    {
      scene_segmentation_ros_->setPassthroughParams(enable_passthrough_filter_, passthrough_filter_field_name_,
                                                    passthrough_filter_limit_min_, passthrough_filter_limit_max_);
    }
    else if (enable_cropbox_filter_)
    {
      scene_segmentation_ros_->setCropBoxParams(enable_cropbox_filter_, cropbox_filter_x_limit_min_,
                                                cropbox_filter_x_limit_max_, cropbox_filter_y_limit_min_, cropbox_filter_y_limit_max_,
                                                cropbox_filter_z_limit_min_, cropbox_filter_z_limit_max_);
    }
    else if (enable_cropbox_filter_ && enable_passthrough_filter_)
    {
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
    if (enable_passthrough_filter_)
    {
      scene_segmentation_ros_->setPassthroughParams(enable_passthrough_filter_, passthrough_filter_field_name_,
                                                    passthrough_filter_limit_min_, passthrough_filter_limit_max_);
    }
    else if (enable_cropbox_filter_)
    {
      scene_segmentation_ros_->setCropBoxParams(enable_cropbox_filter_, cropbox_filter_x_limit_min_,
                                                cropbox_filter_x_limit_max_, cropbox_filter_y_limit_min_, cropbox_filter_y_limit_max_,
                                                cropbox_filter_z_limit_min_, cropbox_filter_z_limit_max_);
    }
    else if (enable_cropbox_filter_ && enable_passthrough_filter_)
    {
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
} // end of namespace