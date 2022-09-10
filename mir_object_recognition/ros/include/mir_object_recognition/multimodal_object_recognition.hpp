
#ifndef MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H
#define MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H

#include <chrono>
#include <iostream>
#include <filesystem>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <utility>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/logging_macros.h"

#include "rmw/qos_profiles.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "mas_perception_msgs/msg/object_list.hpp"
#include "mas_perception_msgs/msg/bounding_box_list.hpp"
#include "mas_perception_msgs/msg/image_list.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "mir_perception_utils/clustered_point_cloud_visualizer.hpp"
#include "mir_perception_utils/bounding_box_visualizer.hpp"
#include "mir_perception_utils/label_visualizer.hpp"
#include "mir_perception_utils/object_utils_ros.hpp"
#include "mir_perception_utils/pointcloud_utils_ros.hpp"
#include "mir_object_segmentation/scene_segmentation_ros.hpp"
#include "mir_perception_utils/bounding_box.hpp"
#include "multimodal_object_recognition_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace mpu = mir_perception_utils;
using mpu::visualization::BoundingBoxVisualizer;
using mpu::visualization::ClusteredPointCloudVisualizer;
using mpu::visualization::LabelVisualizer;
using mpu::visualization::Color;

struct Object
{
  std::string name;
  std::string shape;
  std::string color;
};
typedef std::vector<Object> ObjectInfo;

namespace perception_namespace
{
class MultiModalObjectRecognitionROS: public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit MultiModalObjectRecognitionROS(const rclcpp::NodeOptions& options);

        /// Transition callback for state configuring
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &);

        /// Transition callback for state activating
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & state);

        /// Transition callback for state deactivating
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & state);

         /// Transition callback for state cleaningup
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &);

        /// Transition callback for state shutting down
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State & state);

    // private:
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

        message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode> image_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode> cloud_sub_;
        
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                sensor_msgs::msg::PointCloud2> msgSyncPolicy;
        typedef message_filters::Synchronizer<msgSyncPolicy> Sync;
        std::shared_ptr<Sync> msg_sync_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        typedef std::shared_ptr<SceneSegmentationROS> SceneSegmentationROSSPtr;
        SceneSegmentationROSSPtr scene_segmentation_ros_;
        typedef std::shared_ptr<MultimodalObjectRecognitionUtils> MultimodalObjectRecognitionUtilsSPtr;
        MultimodalObjectRecognitionUtilsSPtr mm_object_recognition_utils_;
        
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>> pub_workspace_height_;

        // publisher debug
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_debug_cloud_plane_;

        // Publisher for clouds and images recognizer
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<mas_perception_msgs::msg::ObjectList>> pub_cloud_to_recognizer_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<mas_perception_msgs::msg::ImageList>> pub_image_to_recognizer_;

        // Callback group for subscribers
        rclcpp::CallbackGroup::SharedPtr recognized_callback_group_;

        // Subscription options
        rclcpp::SubscriptionOptions recognized_sub_options;

        // Subscriber for clouds and images recognizer
        std::shared_ptr<rclcpp::Subscription<mas_perception_msgs::msg::ObjectList>> sub_recognized_image_list_;
        std::shared_ptr<rclcpp::Subscription<mas_perception_msgs::msg::ObjectList>> sub_recognized_cloud_list_;

        // Publisher object lsit
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<mas_perception_msgs::msg::ObjectList>> pub_object_list_;

        // Publisher pose array (debug_mode only)
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>> pub_pc_object_pose_array_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>> pub_rgb_object_pose_array_;

        // --------------------------- function declarations -----------------------------------

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

        /** @brief method to declare all parameters of this node */
        void declare_all_parameters();

        /** @brief method to load all parameters into node variables */
        void get_all_parameters();
        
        /**
         * \brief Callback for the synchronized image and point cloud messages
         * \param[in] image_msg
         * \param[in] cloud_msg
         */
        void synchronizeCallback(const std::shared_ptr<sensor_msgs::msg::Image> &image, 
                const std::shared_ptr<sensor_msgs::msg::PointCloud2> &cloud);

        // Recognize Clouds and Image callback
        void recognizedImageCallback(const mas_perception_msgs::msg::ObjectList &msg);
        void recognizedCloudCallback(const mas_perception_msgs::msg::ObjectList &msg);

        /** \brief Transform pointcloud to the given target frame id ("base_link" by default)
         * \param[in] PointCloud2 input
        */
        void preprocessPointCloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &cloud_msg);

        /** \brief Add cloud accumulation, segment accumulated pointcloud, find the plane, 
         *     clusters table top objects, find object heights.
         * \param[out] 3D object list with unknown label
         * \param[out] Tabletop pointcloud clusters
         **/
        void segmentPointCloud(mas_perception_msgs::msg::ObjectList &object_list,
                        std::vector<PointCloudBSPtr> &clusters,
                        std::vector<mpu::object::BoundingBox> &boxes);
        
        /** \brief Recognize 2D and 3D objects, estimate their pose, filter them, and publish the object_list */
        virtual void recognizeCloudAndImage();

        /** \brief Adjust object pose, make it flat, adjust container, axis and bolt poses.
         * \param[in] Object_list
         * 
         **/
        void adjustObjectPose(mas_perception_msgs::msg::ObjectList &object_list);

        /** \brief Publish object_list to object_list merger 
         * \param[in] Object_list to publish
         **/
        void publishObjectList(mas_perception_msgs::msg::ObjectList &object_list);
        
        /** \brief Publish debug info such as bbox, poses, labels for both 2D and 3D objects.
         * \param[in] combined object list
         * \param[in] 3D pointcloud cluster from 3D object segmentation
         * \param[in] 3D pointcloud cluster from 2D bounding box proposal
         **/
        void publishDebug(mas_perception_msgs::msg::ObjectList &combined_object_list,
                                                std::vector<PointCloudBSPtr> &clusters_3d,
                                                std::vector<PointCloudBSPtr> &clusters_2d);

        /** \brief Load qualitative object info
         * \param[in] Path to the yaml object file
         * */ 
        void loadObjectInfo(const std::string &filename);

    // protected:
        //visualization
        std::shared_ptr<BoundingBoxVisualizer> bounding_box_visualizer_pc_;
        std::shared_ptr<ClusteredPointCloudVisualizer> cluster_visualizer_rgb_;
        std::shared_ptr<ClusteredPointCloudVisualizer> cluster_visualizer_pc_;
        std::shared_ptr<LabelVisualizer> label_visualizer_rgb_;
        std::shared_ptr<LabelVisualizer> label_visualizer_pc_;

        //Qos profiles
        rclcpp::QoS qos_sensor;
        rclcpp::QoS qos_parameters;
        rclcpp::QoS qos_default;

        //parameters
        bool debug_mode_;
        std::string pointcloud_source_frame_id_;
        std::string target_frame_id_;
        std::set<std::string> round_objects_;
        
        ObjectInfo object_info_;
        std::string objects_info_path_;

        // Used to store pointcloud and image received from callback
        std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloud_msg_;
        std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
        PointCloudBSPtr cloud_;

        // Dynamic parameter
        double voxel_leaf_size_;
        std::string voxel_filter_field_name_;
        double voxel_filter_limit_min_;
        double voxel_filter_limit_max_;
        bool enable_passthrough_filter_;
        std::string passthrough_filter_field_name_;
        double passthrough_filter_limit_min_;
        double passthrough_filter_limit_max_;
        bool enable_cropbox_filter_;
        double cropbox_filter_x_limit_min_;
        double cropbox_filter_x_limit_max_;
        double cropbox_filter_y_limit_min_;
        double cropbox_filter_y_limit_max_;
        double cropbox_filter_z_limit_min_;
        double cropbox_filter_z_limit_max_;
        double normal_radius_search_;
        bool use_omp_;
        int num_cores_;
        int sac_max_iterations_;
        double sac_distance_threshold_;
        bool sac_optimize_coefficients_;
        double sac_x_axis_;
        double sac_y_axis_;
        double sac_z_axis_;
        double sac_eps_angle_;
        double sac_normal_distance_weight_;
        double prism_min_height_;
        double prism_max_height_;
        double outlier_radius_search_;
        int outlier_min_neighbors_;
        double cluster_tolerance_;
        int cluster_min_size_;
        int cluster_max_size_;
        double cluster_min_height_;
        double cluster_max_height_;
        double cluster_max_length_;
        double cluster_min_distance_to_polygon_;

        //cluster
        bool center_cluster_;
        bool pad_cluster_;
        int padded_cluster_size_;

        // logdir for saving debug image
        std::string logdir_;

        // rgb_object_id used to differentiate 2D and 3D objects
        int rgb_object_id_;

        double octree_resolution_;
        double object_height_above_workspace_;
        double container_height_;

        // Flags for object recognition
        bool received_recognized_image_list_flag_;
        bool received_recognized_cloud_list_flag_;

        //Recognized image list
        mas_perception_msgs::msg::ObjectList recognized_cloud_list_; 
        mas_perception_msgs::msg::ObjectList recognized_image_list_;

        // Enable recognizer
        bool enable_rgb_recognizer_;
        bool enable_pc_recognizer_;

        int rgb_roi_adjustment_;
        int rgb_bbox_min_diag_;
        int rgb_bbox_max_diag_;
        double rgb_cluster_filter_limit_min_;
        double rgb_cluster_filter_limit_max_;
        bool rgb_cluster_remove_outliers_;
        bool enable_roi_;
        double roi_base_link_to_laser_distance_;
        double roi_max_object_pose_x_to_base_link_;
        double roi_min_bbox_z_;
};

} // namespace perception_namespace ends
#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H
