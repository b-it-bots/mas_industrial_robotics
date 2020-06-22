#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mir_empty_space_detection/empty_space_detector.h>
#include <stdlib.h>
#include <time.h>

EmptySpaceDetector::EmptySpaceDetector() : nh_("~") {
  nh_.param<std::string>("output_frame", output_frame_, "base_link");
  nh_.param<bool>("enable_debug_pc_pub", enable_debug_pc_pub_, true);
  float octree_resolution;
  nh_.param<float>("octree_resolution", octree_resolution, 0.0025);
  add_to_octree_ = false;

  pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("empty_spaces", 1);
  event_out_pub_ = nh_.advertise<std_msgs::String>("event_out", 1);

  pc_sub_ = nh_.subscribe("input_point_cloud", 1,
                          &EmptySpaceDetector::pcCallback, this);
  event_in_sub_ =
      nh_.subscribe("event_in", 1, &EmptySpaceDetector::eventInCallback, this);

  if (enable_debug_pc_pub_) {
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);
  }

  cloud_accumulation_ =
      CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution));
  scene_segmentation_ = SceneSegmentationSPtr(new SceneSegmentation());
  loadParams();

  tf_listener_.reset(new tf::TransformListener);
  ROS_INFO("Initialised");
}

EmptySpaceDetector::~EmptySpaceDetector() {}

void EmptySpaceDetector::loadParams() {
  std::string passthrough_filter_field_name;
  float passthrough_filter_limit_min, passthrough_filter_limit_max;
  bool enable_passthrough_filter;
  nh_.param<bool>("enable_passthrough_filter", enable_passthrough_filter,
                  false);
  nh_.param<std::string>("passthrough_filter_field_name",
                         passthrough_filter_field_name, "x");
  nh_.param<float>("passthrough_filter_limit_min", passthrough_filter_limit_min,
                   0.0);
  nh_.param<float>("passthrough_filter_limit_max", passthrough_filter_limit_max,
                   0.8);
  scene_segmentation_->setPassthroughParams(
      enable_passthrough_filter, passthrough_filter_field_name,
      passthrough_filter_limit_min, passthrough_filter_limit_max);

  float voxel_leaf_size, voxel_filter_limit_min, voxel_filter_limit_max;
  std::string voxel_filter_field_name;
  nh_.param<float>("voxel_leaf_size", voxel_leaf_size, 1.0);
  nh_.param<std::string>("voxel_filter_field_name", voxel_filter_field_name,
                         "z");
  nh_.param<float>("voxel_filter_limit_min", voxel_filter_limit_min, -0.15);
  nh_.param<float>("voxel_filter_limit_max", voxel_filter_limit_max, 0.3);
  scene_segmentation_->setVoxelGridParams(
      voxel_leaf_size, voxel_filter_field_name, voxel_filter_limit_min,
      voxel_filter_limit_max);

  float normal_radius_search;
  bool use_omp;
  int num_cores;
  nh_.param<float>("normal_radius_search", normal_radius_search, 0.03);
  nh_.param<bool>("use_omp", use_omp, false);
  scene_segmentation_->setNormalParams(normal_radius_search, use_omp,
                                       num_cores);

  int sac_max_iterations;
  float sac_distance_threshold, sac_x_axis, sac_y_axis, sac_z_axis,
      sac_eps_angle;
  float sac_normal_distance_weight;
  bool sac_optimize_coefficients;
  nh_.param<int>("sac_max_iterations", sac_max_iterations, 1000);
  nh_.param<float>("sac_distance_threshold", sac_distance_threshold, 0.01);
  nh_.param<bool>("sac_optimize_coefficients", sac_optimize_coefficients, true);
  nh_.param<float>("sac_x_axis", sac_x_axis, 0.0);
  nh_.param<float>("sac_y_axis", sac_y_axis, 0.0);
  nh_.param<float>("sac_z_axis", sac_z_axis, 1.0);
  Eigen::Vector3f axis(sac_x_axis, sac_y_axis, sac_z_axis);
  nh_.param<float>("sac_eps_angle", sac_eps_angle, 0.09);
  nh_.param<float>("sac_normal_distance_weight", sac_normal_distance_weight,
                   0.05);
  scene_segmentation_->setSACParams(sac_max_iterations, sac_distance_threshold,
                                    sac_optimize_coefficients, axis,
                                    sac_eps_angle, sac_normal_distance_weight);

  nh_.param<float>("empty_space_point_count_percentage_threshold",
                   empty_space_pnt_cnt_perc_thresh_, 0.8);
  nh_.param<float>("empty_space_radius", empty_space_radius_, 0.05);
  expected_num_of_points_ = (empty_space_radius_ * empty_space_radius_ * M_PI) /
                            (voxel_leaf_size * voxel_leaf_size);
}

void EmptySpaceDetector::eventInCallback(
    const std_msgs::String::ConstPtr &msg) {
  std_msgs::String event_out;
  if (msg->data == "e_add_cloud") {
    add_to_octree_ = true;
    return;
  } else if (msg->data == "e_add_cloud_stop") {
    add_to_octree_ = false;
    event_out.data = "e_add_cloud_stopped";
  } else if (msg->data == "e_trigger") {
    bool success = findEmptySpaces();
    event_out.data = (success) ? "e_success" : "e_failure";
  } else {
    return;
  }
  event_out_pub_.publish(event_out);
}

void EmptySpaceDetector::pcCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if (add_to_octree_) {
    sensor_msgs::PointCloud2 msg_transformed;
    if (!mpu::pointcloud::transformPointCloudMsg(tf_listener_, output_frame_,
                                                 *msg, msg_transformed))
      return;

    PointCloud::Ptr input_pc(new PointCloud);
    pcl::fromROSMsg(msg_transformed, *input_pc);

    cloud_accumulation_->addCloud(input_pc);

    add_to_octree_ = false;

    std_msgs::String event_out;
    event_out.data = "e_added_cloud";
    event_out_pub_.publish(event_out);
  }
}

bool EmptySpaceDetector::findEmptySpaces() {
  PointCloud::Ptr plane(new PointCloud);
  bool plane_found = this->findPlane(plane);
  if (!plane_found) {
    return false;
  }

  geometry_msgs::PoseArray empty_space_poses;
  this->findEmptySpacesOnPlane(plane, empty_space_poses);
  ROS_DEBUG_STREAM(empty_space_poses);
  if (empty_space_poses.poses.size() == 0) {
    return false;
  }

  pose_array_pub_.publish(empty_space_poses);

  if (enable_debug_pc_pub_) {
    /* publish debug pointcloud */
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*plane, output);
    output.header.frame_id = output_frame_;
    output.header.stamp = ros::Time::now();
    pc_pub_.publish(output);
    ROS_INFO("Publishing debug pointcloud");
  }

  return true;
}

void EmptySpaceDetector::findEmptySpacesOnPlane(
    const PointCloud::Ptr &plane, geometry_msgs::PoseArray &empty_space_poses) {
  srand(time(NULL));
  int num_of_empty_spaces_required;
  nh_.param<int>("num_of_empty_spaces_required", num_of_empty_spaces_required,
                 3);

  float trial_duration_sec;
  nh_.param<float>("trial_duration", trial_duration_sec, 3.0);
  ros::Duration trial_duration(trial_duration_sec);
  ros::Time start_time = ros::Time::now();

  empty_space_poses.header.frame_id = output_frame_;
  empty_space_poses.header.stamp = ros::Time::now();
  int attempts = 0;

  while (ros::Time::now() - start_time < trial_duration) {
    attempts++;
    ROS_DEBUG_STREAM("Attempt: " << attempts);
    PointCloud::Ptr new_plane(new PointCloud);
    pcl::copyPointCloud(*plane, *new_plane);
    pcl::PointIndices::Ptr empty_space(new pcl::PointIndices());

    std::vector<PointT> samples;
    for (int i = 0; i < num_of_empty_spaces_required; ++i) {
      int random_index = rand() % new_plane->width;
      samples.push_back(new_plane->points[random_index]);
    }

    bool success = true;
    for (PointT p : samples) {
      std::vector<int> ids;
      std::vector<float> sq_distances;
      kdtree_.setInputCloud(new_plane);
      if (kdtree_.radiusSearch(p, empty_space_radius_, ids, sq_distances) > 0) {
        if (((float)ids.size() / expected_num_of_points_) >
            empty_space_pnt_cnt_perc_thresh_) {
          empty_space->indices = ids;
          extract_indices_.setInputCloud(new_plane);
          extract_indices_.setIndices(empty_space);
          extract_indices_.setNegative(true);
          extract_indices_.filter(*new_plane);
        } else {
          success = false;
          break;
        }
      }
    }
    if (success) {
      ROS_INFO_STREAM("Found solution at attempt: " << attempts);
      for (PointT p : samples) {
        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = p.z;
        pose.orientation.w = 1.0;
        empty_space_poses.poses.push_back(pose);
      }
      break;
    }
  }
}

bool EmptySpaceDetector::findPlane(PointCloud::Ptr plane) {
  PointCloud::Ptr cloud_in(new PointCloud);
  cloud_accumulation_->getAccumulatedCloud(*cloud_in);

  PointCloud::Ptr hull(new PointCloud);
  pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);
  double workspace_height;

  PointCloud::Ptr debug = scene_segmentation_->findPlane(
      cloud_in, hull, plane, model_coefficients, workspace_height);
  bool success = plane->points.size() > 0;
  return success;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "empty_space_detector");
  EmptySpaceDetector es_detector;
  ros::Rate loop_rate(10.0);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
