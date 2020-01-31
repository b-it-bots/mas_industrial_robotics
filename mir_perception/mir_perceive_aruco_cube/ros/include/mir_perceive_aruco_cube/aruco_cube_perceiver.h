#ifndef ARUCO_CUBE_PERCEIVER_H_
#define ARUCO_CUBE_PERCEIVER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> ImageSyncPolicy;

class ArucoCubePerceiver
{

public:
    ArucoCubePerceiver();
    virtual ~ArucoCubePerceiver();
    void synchronizedCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                              const sensor_msgs::Image::ConstPtr &image_msg);

private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    sensor_msgs::PointCloud2::ConstPtr pointcloud_msg_;
    sensor_msgs::Image::ConstPtr image_msg_;
    boost::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy> > sync_input_;

    image_transport::ImageTransport image_transporter_;
    image_transport::Publisher image_pub_;

    tf::TransformListener *transform_listener_;

    std::string target_frame_;


    bool imgToCV(const sensor_msgs::Image::ConstPtr &image_msg, cv_bridge::CvImagePtr &cv_ptr);
};

#endif /* ARUCO_CUBE_PERCEIVER_H_ */
