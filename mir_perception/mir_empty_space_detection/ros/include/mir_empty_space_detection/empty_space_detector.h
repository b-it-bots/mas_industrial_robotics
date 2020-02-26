#ifndef EMPTY_SPACE_DETECTOR_H
#define EMPTY_SPACE_DETECTOR_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <mir_object_segmentation/scene_segmentation.h>
#include <mir_object_segmentation/cloud_accumulation.h>
#include <mir_perception_utils/pointcloud_utils_ros.h>

namespace mpu = mir_perception_utils;

class EmptySpaceDetector
{
    public:
        EmptySpaceDetector();
        virtual ~EmptySpaceDetector();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber pc_sub_;
        ros::Subscriber event_in_sub_;
        ros::Publisher pc_pub_;
        ros::Publisher pose_pub_;
        ros::Publisher event_out_pub_;

        std::string output_frame_;
        bool enable_debug_pc_pub_;
        bool add_to_octree_;
        /* bool is_running; */
        /* int retry_attempts_; */
        /* int num_of_retries_; */
        boost::shared_ptr<tf::TransformListener> tf_listener_;

        typedef std::shared_ptr<SceneSegmentation> SceneSegmentationSPtr;
        SceneSegmentationSPtr scene_segmentation_;
        CloudAccumulation::UPtr cloud_accumulation_;

        void pcCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void eventInCallback(const std_msgs::String::ConstPtr &msg);
        void loadParams();
        void findEmptySpace();
};
#endif
