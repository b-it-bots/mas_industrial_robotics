#include "mir_object_recognition/multimodal_object_recognition.hpp"

namespace perception_namespace
{
MultiModalObjectRecognitionROS::MultiModalObjectRecognitionROS(const rclcpp::NodeOptions& options) : 
                    rclcpp_lifecycle::LifecycleNode("mmor_node",options),
// MultiModalObjectRecognitionROS::MultiModalObjectRecognitionROS(const std::string &node_name, bool intra_process_comms) : 
//                     rclcpp_lifecycle::LifecycleNode(node_name,
//                     rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
                    bounding_box_visualizer_pc_("output/bounding_boxes", Color(Color::IVORY)),
                    cluster_visualizer_rgb_("output/tabletop_cluster_rgb", true),
                    cluster_visualizer_pc_("output/tabletop_cluster_pc"),
                    label_visualizer_rgb_("output/rgb_labels", Color(Color::SEA_GREEN)),
                    label_visualizer_pc_("output/pc_labels", Color(Color::IVORY)),
                    rgb_object_id_(100),
                    container_height_(0.05),
                    received_recognized_image_list_flag_(false),
                    received_recognized_cloud_list_flag_(false),
                    enable_rgb_recognizer_(true),
                    enable_pc_recognizer_(true),
                    rgb_roi_adjustment_(2),
                    rgb_cluster_remove_outliers_(true)
{
    RCLCPP_INFO(get_logger(), "constructor called");
    this->declare_parameter<std::string>("target_frame_id", "base_link");
    this->get_parameter("target_frame_id", target_frame_id_);
    
    // changing the debug mode to true for testing purposes, revert to false for production
    this->declare_parameter<bool>("debug_mode_", true);
    this->get_parameter("debug_mode_", debug_mode_);
    this->declare_parameter<std::string>("logdir", "~/Downloads/");
    this->get_parameter("logdir", logdir_);
    
    scene_segmentation_ros_ = SceneSegmentationROSSPtr(new SceneSegmentationROS());
    mm_object_recognition_utils_ = MultimodalObjectRecognitionUtilsSPtr(new MultimodalObjectRecognitionUtils());

    MultiModalObjectRecognitionROS::declare_all_parameters();
    object_info_path_ = "src/mir_object_recognition/ros/config/objects.yaml";
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

    // Reset received recognized cloud and image
    received_recognized_cloud_list_flag_ = false;
    received_recognized_image_list_flag_ = false;

    // reset object id
    rgb_object_id_ = 100;
    scene_segmentation_ros_->resetPclObjectId();

    // clear recognized image and cloud list
    recognized_image_list_.objects.clear();
    recognized_cloud_list_.objects.clear();

    scene_segmentation_ros_->resetCloudAccumulation();

    RCLCPP_INFO(get_logger(), "synchro callback complete");
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
        RCLCPP_ERROR(this->get_logger(),"Unable to transform pointcloud. Are you sure target_frame_id_ and pointcloud_source_frame_id are set correctly?");
        RCLCPP_ERROR(this->get_logger(),"pointcloud_source_frame_id: %s, target_frame_id: %s", pointcloud_source_frame_id_.c_str(), target_frame_id_.c_str());
        RCLCPP_ERROR(this->get_logger(),"pointcloud_source_frame_id may need to be arm_cam3d_camera_color_frame or fixed_camera_link");
        RCLCPP_ERROR(this->get_logger(),"target_frame_id may need to be base_link or base_link_static");
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
                                                       std::vector<mpu::object::BoundingBox> &boxes)
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
        RCLCPP_INFO_STREAM(get_logger(), "Publishing debug cloud plane");
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
        RCLCPP_INFO_STREAM(get_logger(), "Publishing clouds for recognition");
        pub_cloud_to_recognizer_->publish(cloud_object_list);
        test_pub_pose_->publish(cloud_object_list.objects[0].pose);
    }

    // commenting the rest of the code for now to test object detection

    /* 

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
                    // combined_object_list.objects[i].pose.pose.position.z < scene_segmentation_ros_ 
                    // ->object_height_above_workspace_ - 0.05) 
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
    
 */
}

void MultiModalObjectRecognitionROS::adjustObjectPose(mas_perception_msgs::msg::ObjectList &object_list)
{
    for (size_t i = 0; i < object_list.objects.size(); i++)
    {
        tf2::Quaternion q(
            object_list.objects[i].pose.pose.orientation.x,
            object_list.objects[i].pose.pose.orientation.y,
            object_list.objects[i].pose.pose.orientation.z,
            object_list.objects[i].pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double change_in_pitch = 0.0;
        if (round_objects_.count(object_list.objects[i].name))
        {
            yaw = 0.0;
        }
        // Update container pose
        if (object_list.objects[i].name == "CONTAINER_BOX_RED" || 
            object_list.objects[i].name == "CONTAINER_BOX_BLUE")
        {
            if (object_list.objects[i].database_id > 100)
            {
                RCLCPP_DEBUG_STREAM(get_logger(), "Updating RGB container pose");
                mm_object_recognition_utils_->adjustContainerPose(object_list.objects[i], container_height_);
            }
        }
        // Make pose flat
        tf2::Quaternion q2;
        q2.setRPY(0.0, change_in_pitch, yaw);

        object_list.objects[i].pose.pose.orientation.x = q2.x();
        object_list.objects[i].pose.pose.orientation.y = q2.y();
        object_list.objects[i].pose.pose.orientation.z = q2.z();
        object_list.objects[i].pose.pose.orientation.w = q2.w();

        // Update workspace height
        if (scene_segmentation_ros_ -> getWorkspaceHeight() != -1000.0)
        {
            object_list.objects[i].pose.pose.position.z = 
                scene_segmentation_ros_ -> getWorkspaceHeight() + object_height_above_workspace_;

            if (object_list.objects[i].name == "CONTAINER_BOX_RED" || 
                object_list.objects[i].name == "CONTAINER_BOX_BLUE")
            {
                object_list.objects[i].pose.pose.position.z = 
                    scene_segmentation_ros_ -> getWorkspaceHeight() + container_height_;
                RCLCPP_WARN_STREAM(get_logger(), "Updated container height: " << object_list.objects[i].pose.pose.position.z);
            }
        }
        // Update axis or bolt pose
        if (object_list.objects[i].name == "M20_100" || object_list.objects[i].name == "AXIS")
        {
            mm_object_recognition_utils_->adjustAxisBoltPose(object_list.objects[i]);
        }
    }
}

void MultiModalObjectRecognitionROS::publishObjectList(mas_perception_msgs::msg::ObjectList &object_list)
{
    for (size_t i = 0; i < object_list.objects.size(); i++)
    {
        // Empty cloud
        sensor_msgs::msg::PointCloud2 empty_ros_cloud;
        object_list.objects[i].views.resize(1);
        object_list.objects[i].views[0].point_cloud = empty_ros_cloud;
        // Rename container to match refbox naming
        if (object_list.objects[i].name == "BLUE_CONTAINER")
        {
            object_list.objects[i].name = "CONTAINER_BOX_BLUE";
        }
        else if (object_list.objects[i].name == "RED_CONTAINER")
        {
            object_list.objects[i].name = "CONTAINER_BOX_RED";
        }
    }
    // Publish object list to object list merger
    pub_object_list_ -> publish(object_list);
}


void MultiModalObjectRecognitionROS::publishDebug(mas_perception_msgs::msg::ObjectList &combined_object_list,
                                                std::vector<PointCloudBSPtr> &clusters_3d,
                                                std::vector<PointCloudBSPtr> &clusters_2d)
{
    RCLCPP_INFO_STREAM(get_logger(), "Cloud list: " << recognized_cloud_list_.objects.size());
    RCLCPP_INFO_STREAM(get_logger(), "RGB list: " << recognized_image_list_.objects.size());
    RCLCPP_INFO_STREAM(get_logger(), "Combined object list: "<< combined_object_list.objects.size());

    // Compute normal to generate parallel BBOX to the plane
    const Eigen::Vector3f normal = scene_segmentation_ros_ -> getPlaneNormal();

    std::string names = "";
    if (recognized_cloud_list_.objects.size() > 0)
    {
        // Bounding boxes
        if (clusters_3d.size() > 0)
        {
            mas_perception_msgs::msg::BoundingBoxList bounding_boxes;
            cluster_visualizer_pc_.publish(clusters_3d, target_frame_id_);
            bounding_boxes.bounding_boxes.resize(clusters_3d.size());
            for (size_t i=0; i < clusters_3d.size(); i++)
            {
                mpu::object::BoundingBox bbox;
                mpu::object::get3DBoundingBox(clusters_3d[i], normal, bbox, bounding_boxes.bounding_boxes[i]);
            }
            if (bounding_boxes.bounding_boxes.size() > 0)
            {
                bounding_box_visualizer_pc_.publish(bounding_boxes.bounding_boxes, target_frame_id_);
            }
        }
        // PCL Pose array for debug mode only
        geometry_msgs::msg::PoseArray pcl_object_pose_array;
        pcl_object_pose_array.header.frame_id = target_frame_id_;
        pcl_object_pose_array.header.stamp = this->get_clock()->now();
        pcl_object_pose_array.poses.resize(recognized_cloud_list_.objects.size());
        std::vector<std::string> pcl_labels;
        int pcl_count = 0;
        for (size_t i=0; i < combined_object_list.objects.size(); i++)
        {
            if (combined_object_list.objects[i].database_id < 99)
            {
                names += combined_object_list.objects[i].name + ", ";
                pcl_object_pose_array.poses[pcl_count] = combined_object_list.objects[i].pose.pose;
                pcl_labels.push_back(combined_object_list.objects[i].name);
                pcl_count++;
            }
        }
        RCLCPP_INFO_STREAM(get_logger(), "[Cloud] Objects: " << names);
        // Publish pose array
        if (pcl_object_pose_array.poses.size() > 0)
        {
            pub_pc_object_pose_array_ -> publish(pcl_object_pose_array);
        }
        // Publish label visualizer
        if ((pcl_labels.size() == pcl_object_pose_array.poses.size()) &&
            (pcl_labels.size() > 0) && (pcl_object_pose_array.poses.size() > 0))
        {
            label_visualizer_pc_.publish(pcl_labels, pcl_object_pose_array);
        }
    }
    if (clusters_2d.size() > 0)
    {
        cluster_visualizer_rgb_.publish(clusters_2d, target_frame_id_);
        // RGB Pose array for debug mode only
        geometry_msgs::msg::PoseArray rgb_object_pose_array;
        rgb_object_pose_array.header.frame_id = target_frame_id_;
        rgb_object_pose_array.header.stamp = this->get_clock()->now();
        rgb_object_pose_array.poses.resize(recognized_image_list_.objects.size());
        std::vector<std::string> rgb_labels;
        int rgb_count = 0;
        names = "";
        for (size_t i = 0; i < combined_object_list.objects.size(); i++)
        {
            if (combined_object_list.objects[i].database_id > 99)
            {
                names += combined_object_list.objects[i].name + ", ";
                rgb_object_pose_array.poses[rgb_count] = combined_object_list.objects[i].pose.pose;
                rgb_labels.push_back(combined_object_list.objects[i].name);
                rgb_count++;
            }
        }
        RCLCPP_INFO_STREAM(get_logger(), "[RGB] Objects: " << names);
        // Publish pose array
        if (rgb_object_pose_array.poses.size() > 0)
        {
            pub_rgb_object_pose_array_->publish(rgb_object_pose_array);
        }
        // Publish label visualizer
        if ((rgb_labels.size() == rgb_object_pose_array.poses.size()) &&
            (rgb_labels.size() > 0) && (rgb_object_pose_array.poses.size() > 0))
        {
            label_visualizer_rgb_.publish(rgb_labels, rgb_object_pose_array);
        }
    }
}

void MultiModalObjectRecognitionROS::loadObjectInfo(const std::string &filename)
{
    YAML::Node config = YAML::LoadFile(filename);
    RCLCPP_INFO(get_logger(), "File loaded !");
    mas_perception_msgs::msg::Object object1;
    if (config["object_info"])
    {
        for (unsigned j = 0; j < config["object_info"]["object"].size(); ++j)
        {
            Object f;
            f.name = config["object_info"]["object"][j]["name"].as<std::string>();
            f.shape = config["object_info"]["object"][j]["shape"].as<std::string>();
            f.color = config["object_info"]["object"][j]["color"].as<std::string>();
            // RCLCPP_INFO(get_logger(), "%s || %s || %s", f.name.c_str(), f.shape.c_str(), f.color.c_str());
            if (f.shape == object1.shape.SPHERE)
            {
                round_objects_.insert(f.name);
                // RCLCPP_INFO(get_logger(), "Round object detected !");
            }
            object_info_.push_back(f);
        }
        
        RCLCPP_INFO(get_logger(), "Object info is loaded!");
    }
    else
    {
        RCLCPP_WARN(get_logger(), "No object info is provided!");
    }    
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
    pub_pc_object_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("output/pc_object_pose_array", 10);
    pub_rgb_object_pose_array_  = this->create_publisher<geometry_msgs::msg::PoseArray>("output/rgb_object_pose_array", 10);

    // msg_sync_.reset(new Sync(msgSyncPolicy(10), image_sub_, cloud_sub_));
    msg_sync_ = std::make_shared<Sync>(msgSyncPolicy(10), image_sub_, cloud_sub_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MultiModalObjectRecognitionROS::parametersCallback, this, std::placeholders::_1));

    MultiModalObjectRecognitionROS::get_all_parameters();
    MultiModalObjectRecognitionROS::loadObjectInfo(object_info_path_);

    // publish workspace height
    pub_workspace_height_ = this->create_publisher<std_msgs::msg::Float64>("workspace_height", 1);

    // publish debug
    pub_debug_cloud_plane_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/debug_cloud_plane", 1);

    // Publish cloud and images to cloud and rgb recognition topics
    pub_cloud_to_recognizer_ = this->create_publisher<mas_perception_msgs::msg::ObjectList>("recognizer/pc/input/object_list", 1);
    pub_image_to_recognizer_ = this->create_publisher<mas_perception_msgs::msg::ImageList>("recognizer/rgb/input/images", 1);

    test_pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("test/pose", 1);

    // Subscribe to cloud and rgb recognition topics
    sub_recognized_image_list_ = this->create_subscription<mas_perception_msgs::msg::ObjectList>(
        "recognizer/rgb/output/object_list", 1, std::bind(&MultiModalObjectRecognitionROS::recognizedImageCallback, this, std::placeholders::_1));

    sub_recognized_cloud_list_ = this->create_subscription<mas_perception_msgs::msg::ObjectList>(
        "recognizer/pc/output/object_list", 1, std::bind(&MultiModalObjectRecognitionROS::recognizedCloudCallback, this, std::placeholders::_1));

    // Pub combined object_list to object_list merger
    pub_object_list_ = this->create_publisher<mas_perception_msgs::msg::ObjectList>("output/object_list", 1);

    // Pub pose array
    pub_pc_object_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("output/pc_object_pose_array", 10);
    pub_rgb_object_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("output/rgb_object_pose_array", 10);

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
    pub_cloud_to_recognizer_->on_activate();
    std::this_thread::sleep_for(2s);

    //test 
    test_pub_pose_->on_activate();

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
    pub_cloud_to_recognizer_->on_deactivate();

    //test
    test_pub_pose_->on_deactivate();

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
    pub_cloud_to_recognizer_->on_deactivate();

    //test
    test_pub_pose_->on_deactivate();

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

} //end of namespace


/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
// int main(int argc, char *argv[])
// {
//     // force flush of the stdout buffer.
//     // this ensures a correct sync of all prints
//     // even when executed simultaneously within the launch file.
//     setvbuf(stdout, NULL, _IONBF, BUFSIZ);

//     rclcpp::init(argc, argv);

//     rclcpp::executors::SingleThreadedExecutor exe;

//     std::shared_ptr<MultiModalObjectRecognitionROS> mmor_lc_node =
//         std::make_shared<MultiModalObjectRecognitionROS>("multimodal_object_recognition", false);

//     exe.add_node(mmor_lc_node->get_node_base_interface());

//     exe.spin();

//     rclcpp::shutdown();

//     return 0;
// }


RCLCPP_COMPONENTS_REGISTER_NODE(perception_namespace::MultiModalObjectRecognitionROS)
