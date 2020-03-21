#ifndef BARRIERTAPEDETECTIONROS_H_
#define BARRIERTAPEDETECTIONROS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <string>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <mir_barrier_tape_detection/BarrierTapeConfig.h>
#include <mir_barrier_tape_detection/barrier_tape_detection.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> ImageSyncPolicy;

class BarrierTapeDetectionRos
{

public:
    BarrierTapeDetectionRos(ros::NodeHandle &nh);
    virtual ~BarrierTapeDetectionRos();
    void dynamicReconfigCallback(mir_barrier_tape_detection::BarrierTapeConfig &config, uint32_t level);
    void eventCallback(const std_msgs::String &event_command);
    /**
     * Get 3D pointcloud and RGB image at the same time
     */
    void synchronizedCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg, const sensor_msgs::Image::ConstPtr &rgb_image_msg);
    void states();
    void initState();
    void idleState();
    void runState();
    void detectBarrierTape();
    /**
     * Converts the input pointcloud message into a 3 channel cv::Mat object 
     * where each channel holds the x, y and z coordinate of each pixel (i.e. the pixel
     * corresponding to that point in the pointcloud. 
     * Points which are NaN and the Z-coordinate is < 0.01 are discarded
     *
     * This output cv::Mat is later used to map pixels in the RGB image to 3D positions
     */
    void convertPointCloudToXYZImage(cv::Mat &output_xyz_image);

private:
    enum States
    {
        INIT,
        IDLE,
        RUNNING
    };

private:
    dynamic_reconfigure::Server<mir_barrier_tape_detection::BarrierTapeConfig> dynamic_reconfigure_server_;
    ros::Time start_time_;
    ros::NodeHandle node_handler_;
    ros::Publisher event_pub_;
    ros::Publisher pub_yellow_barrier_tape_cloud_;
    ros::Subscriber event_sub_;
    ros::Subscriber pointcloud_sub_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud_;
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_image_;
    sensor_msgs::PointCloud2::ConstPtr pointcloud_msg_;
    sensor_msgs::Image::ConstPtr rgb_image_msg_;
    boost::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy> > sync_input_;

    image_transport::ImageTransport image_transporter_;
    image_transport::Publisher image_pub_;

    tf::TransformListener *transform_listener_;
    geometry_msgs::PoseArray pose_array_;
    geometry_msgs::PoseArray pose_array_dubug_;
    ros::Publisher pub_yellow_barrier_tape_pose_array_;

    std_msgs::String event_in_msg_;
    std_msgs::String event_out_msg_;

    BarrierTapeDetection btd_;
    States current_state_;
    cv::Mat debug_image_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr barrier_tape_cloud_;

    bool is_debug_mode_;
    bool has_image_data_;
    std::string target_frame_;
    int num_of_retrial_;
    int num_pixels_to_extrapolate_;
};

#endif /* BARRIERTAPEDETECTIONROS_H_ */
