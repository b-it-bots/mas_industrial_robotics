
#ifndef MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H
#define MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mas_perception_msgs/msg/object_list.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_ros/transforms.hpp>

#include "mir_perception_utils/clustered_point_clouid_visualizer.hpp"
#include "mir_perception_utils/object_utils_ros.hpp"
#include "mir_perception_utils/pointcloud_utils_ros.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace mpu = mir_perception_utils;
using mpu::visualization::ClusteredPointCloudVisualizer;
using mpu::visualization::Color;

class MultiModalObjectRecognitionROS: public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit MultiModalObjectRecognitionROS(const std::string & node_name, bool intra_process_comms);

        /// Transition callback for state configuring
        /**
         * on_configure callback is being called when the lifecycle node
         * enters the "configuring" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "unconfigured".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

        void declare_all_parameters();

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &);

        /// Transition callback for state activating
        /**
         * on_activate callback is being called when the lifecycle node
         * enters the "activating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "active" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "active"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & state);

        /// Transition callback for state deactivating
        /**
         * on_deactivate callback is being called when the lifecycle node
         * enters the "deactivating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "active".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "active"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & state);

         /// Transition callback for state cleaningup
        /**
         * on_cleanup callback is being called when the lifecycle node
         * enters the "cleaningup" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "unconfigured" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &);

        /// Transition callback for state shutting down
        /**
         * on_shutdown callback is being called when the lifecycle node
         * enters the "shuttingdown" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "finalized" state or stays
         * in its current state.
         * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
         * TRANSITION_CALLBACK_FAILURE transitions to current state
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State & state);

    private:
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<mas_perception_msgs::msg::ObjectList>> obj_list_pub_;
        
        message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode> image_sub_;
        
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode> cloud_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                sensor_msgs::msg::PointCloud2> msgSyncPolicy;
        typedef message_filters::Synchronizer<msgSyncPolicy> Sync;
        std::shared_ptr<Sync> msg_sync_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::string target_frame_id_ = "base_link";
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> publisher_;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        
        void synchronizeCallback(const sensor_msgs::msg::Image &image, 
                const sensor_msgs::msg::PointCloud2 &cloud);

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        
        // void preprocessPointCloud(const sensor_msgs::msg::PointCloud2 &cloud_msg);
        
        bool preprocessPointCloud(const std::shared_ptr<tf2_ros::TransformListener> &tf_listener, 
                                  const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                                  const std::string target_frame, 
                                  const sensor_msgs::msg::PointCloud2 cloud_in,
                                  sensor_msgs::msg::PointCloud2 cloud_out);

    protected:
        // Visualization
        ClusteredPointCloudVisualizer cluster_visualizer_rgb_;
        ClusteredPointCloudVisualizer cluster_visualizer_pc_;
};

#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H