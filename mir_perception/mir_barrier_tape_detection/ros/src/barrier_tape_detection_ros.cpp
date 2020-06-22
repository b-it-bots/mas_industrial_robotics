#include <mir_barrier_tape_detection/barrier_tape_detection_ros.h>

BarrierTapeDetectionRos::BarrierTapeDetectionRos(ros::NodeHandle &nh)
    : node_handler_(nh), image_transporter_(nh) {
  transform_listener_ = new tf::TransformListener();

  nh.param<std::string>("target_frame", target_frame_, "/base_link");
  nh.param<int>("num_of_retrial", num_of_retrial_, 30);
  nh.param<int>("num_pixels_to_extrapolate", num_pixels_to_extrapolate_, 30);
  dynamic_reconfigure_server_.setCallback(boost::bind(
      &BarrierTapeDetectionRos::dynamicReconfigCallback, this, _1, _2));

  event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
  pub_yellow_barrier_tape_cloud_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(
      "output/yellow_barrier_tape_pointcloud", 1);
  pub_yellow_barrier_tape_pose_array_ = nh.advertise<geometry_msgs::PoseArray>(
      "output/yellow_barrier_tape_pose_array", 1);
  image_pub_ = image_transporter_.advertise("debug_image", 1);

  event_sub_ = node_handler_.subscribe(
      "event_in", 1, &BarrierTapeDetectionRos::eventCallback, this);
  sub_pointcloud_.subscribe(node_handler_, "input_pointcloud", 1);
  sub_rgb_image_.subscribe(node_handler_, "input_rgb_image", 1);

  sub_pointcloud_.unsubscribe();
  sub_rgb_image_.unsubscribe();

  sync_input_ =
      boost::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(10);
  sync_input_->connectInput(sub_pointcloud_, sub_rgb_image_);
  sync_input_->registerCallback(boost::bind(
      &BarrierTapeDetectionRos::synchronizedCallback, this, _1, _2));

  current_state_ = INIT;
  has_image_data_ = false;

  barrier_tape_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

BarrierTapeDetectionRos::~BarrierTapeDetectionRos() {
  image_pub_.shutdown();
  pub_yellow_barrier_tape_pose_array_.shutdown();
  event_pub_.shutdown();
  event_sub_.shutdown();
}

void BarrierTapeDetectionRos::dynamicReconfigCallback(
    mir_barrier_tape_detection::BarrierTapeConfig &config, uint32_t level) {
  is_debug_mode_ = config.is_debug_mode;
  btd_.updateDynamicVariables(
      is_debug_mode_, config.min_area, config.color_thresh_min_h,
      config.color_thresh_min_s, config.color_thresh_min_v,
      config.color_thresh_max_h, config.color_thresh_max_s,
      config.color_thresh_max_v);
}

void BarrierTapeDetectionRos::synchronizedCallback(
    const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
    const sensor_msgs::Image::ConstPtr &rgb_image_msg) {
  pointcloud_msg_ = pointcloud_msg;
  rgb_image_msg_ = rgb_image_msg;
  has_image_data_ = true;
}

void BarrierTapeDetectionRos::eventCallback(const std_msgs::String &event_msg) {
  event_in_msg_ = event_msg;
}

void BarrierTapeDetectionRos::states() {
  switch (current_state_) {
    case INIT:
      initState();
      break;
    case IDLE:
      idleState();
      break;
    case RUNNING:
      runState();
      break;
    default:
      initState();
  }
}

void BarrierTapeDetectionRos::initState() {
  if (event_in_msg_.data == "e_start") {
    current_state_ = IDLE;
    event_in_msg_.data = "";
    sub_pointcloud_.subscribe();
    sub_rgb_image_.subscribe();
  } else {
    current_state_ = INIT;
  }
}

void BarrierTapeDetectionRos::idleState() {
  if (event_in_msg_.data == "e_stop") {
    current_state_ = INIT;
    event_in_msg_.data = "";
    sub_pointcloud_.unsubscribe();
    sub_rgb_image_.unsubscribe();
  } else if (has_image_data_) {
    current_state_ = RUNNING;
    has_image_data_ = false;
  } else {
    current_state_ = IDLE;
  }
}

void BarrierTapeDetectionRos::runState() {
  if (event_in_msg_.data == "e_reset") {
    barrier_tape_cloud_->points.clear();
    event_in_msg_.data = "";
  }
  detectBarrierTape();
  current_state_ = IDLE;

  if (is_debug_mode_) {
    cv_bridge::CvImage debug_image_msg;
    debug_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
    debug_image_msg.image = debug_image_;
    image_pub_.publish(debug_image_msg.toImageMsg());

    pub_yellow_barrier_tape_pose_array_.publish(pose_array_);
  }
}

void BarrierTapeDetectionRos::detectBarrierTape() {
  cv_bridge::CvImagePtr cv_img_tmp1 =
      cv_bridge::toCvCopy(rgb_image_msg_, sensor_msgs::image_encodings::BGR8);
  cv::Mat rgb_image_frame = cv_img_tmp1->image;

  barrier_tape_cloud_->header.frame_id = target_frame_;
  pcl_conversions::toPCL(pointcloud_msg_->header.stamp,
                         barrier_tape_cloud_->header.stamp);

  std::vector<std::vector<std::vector<int>>> barrier_tape_img_coordinates;
  int pixel_y;
  int pixel_x;

  if (is_debug_mode_) {
    pose_array_.poses.clear();
    pose_array_.header = pointcloud_msg_->header;
    pose_array_.header.frame_id = target_frame_;
  }

  cv::Mat rgb_depth_image_frame;
  convertPointCloudToXYZImage(rgb_depth_image_frame);

  if (btd_.detectBarrierTape(rgb_image_frame, debug_image_,
                             barrier_tape_img_coordinates)) {
    for (int i = 0; i < barrier_tape_img_coordinates.size(); i++) {
      for (int j = 0; j < barrier_tape_img_coordinates[i].size(); j++) {
        pixel_y = barrier_tape_img_coordinates[i][j][1];
        pixel_x = barrier_tape_img_coordinates[i][j][0];

        cv::Vec3f point = rgb_depth_image_frame.at<cv::Vec3f>(pixel_y, pixel_x);

        if (point.val[0] == 0 && point.val[1] == 0 && point.val[2] == 0) {
          continue;
        }

        else {
          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.pose.position.x = point.val[0];
          pose_stamped.pose.position.y = point.val[1];
          pose_stamped.pose.position.z = point.val[2];
          pose_stamped.pose.orientation.w = 0;
          pose_stamped.pose.orientation.x = 0;
          pose_stamped.pose.orientation.y = 0;
          pose_stamped.pose.orientation.z = 1;
          pose_stamped.header = pointcloud_msg_->header;

          geometry_msgs::PoseStamped transformed_pose;
          try {
            transform_listener_->waitForTransform(
                target_frame_, pose_stamped.header.frame_id,
                pose_stamped.header.stamp, ros::Duration(0.1));
            transform_listener_->transformPose(target_frame_, pose_stamped,
                                               transformed_pose);
            pose_array_.poses.push_back(transformed_pose.pose);

            // Ignore points which are > 0 since we are only interested in
            // barrier tapes
            // on the floor
            if (transformed_pose.pose.position.z > 0.00) {
              ROS_DEBUG("transformed pose is greater than zero");
              continue;
            }

          } catch (tf::TransformException e) {
            ROS_WARN("%s", e.what());
          }

          pcl::PointXYZ pt;
          pt.x = transformed_pose.pose.position.x;
          pt.y = transformed_pose.pose.position.y;
          pt.z = transformed_pose.pose.position.z;

          barrier_tape_cloud_->points.push_back(pt);
          break;
        }
      }
    }
  }
  barrier_tape_cloud_->width = barrier_tape_cloud_->points.size();
  barrier_tape_cloud_->height = 1;

  pub_yellow_barrier_tape_cloud_.publish(barrier_tape_cloud_);
}

void BarrierTapeDetectionRos::convertPointCloudToXYZImage(
    cv::Mat &output_xyz_image) {
  pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);

  pcl_conversions::toPCL(*pointcloud_msg_, *pcl_input_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*pcl_input_cloud, *xyz_input_cloud);

  pcl::PointCloud<pcl::PointXYZ>::iterator b1;

  int index = 0;
  float *xyz_frame_buffer =
      new float[pointcloud_msg_->width * pointcloud_msg_->height * 3];
  cv::Mat xyz_frame(pointcloud_msg_->height, pointcloud_msg_->width, CV_32FC3,
                    xyz_frame_buffer);
  xyz_frame = cv::Scalar::all(0.0f);
  for (b1 = xyz_input_cloud->points.begin(); b1 < xyz_input_cloud->points.end();
       b1++) {
    pcl::PointXYZ pcl_point = *b1;
    if ((!pcl_isnan(pcl_point.x)) && (!pcl_isnan(pcl_point.y)) &&
        (!pcl_isnan(pcl_point.z) && (pcl_point.z > 0.01))) {
      xyz_frame_buffer[index * 3] = b1->x;
      xyz_frame_buffer[index * 3 + 1] = b1->y;
      xyz_frame_buffer[index * 3 + 2] = b1->z;
    }
    index++;
  }
  xyz_frame.copyTo(output_xyz_image);
  delete[] xyz_frame_buffer;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "barrier_tape_detection");
  ros::NodeHandle nh("~");
  ROS_INFO("Barrier Tape Detection Node Initialised");
  BarrierTapeDetectionRos btd(nh);

  int loop_rate = 30;
  nh.param<int>("loop_rate", loop_rate, 30);
  ros::Rate rate(loop_rate);

  while (ros::ok()) {
    ros::spinOnce();
    btd.states();
    rate.sleep();
  }

  return 0;
}
