#ifndef ARUCO_CUBE_PERCEIVER_H_
#define ARUCO_CUBE_PERCEIVER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
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
    boost::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy> > sync_input_;

    image_transport::ImageTransport image_transporter_;
    image_transport::Publisher image_pub_;

    ros::Publisher debug_point_pub_;
    ros::Publisher output_pose_pub_;

    tf::TransformListener tf_listener_;

    std::string target_frame_;
    int num_of_retries_;
    bool debug_;

    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary;

    void calculateCenterOfArucoCube(pcl::PointCloud<pcl::PointXYZ>::Ptr aruco_square, pcl::PointXYZ &cube_center);
    bool transformPose(geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &transformed_pose);
    bool transformPoint(geometry_msgs::PointStamped &point, geometry_msgs::PointStamped &transformed_point);
    bool getBestArucoMarkerCorner(cv_bridge::CvImagePtr &img_ptr, std::vector<cv::Point2f> &corners);
    void calculateVariances(std::vector<std::vector<cv::Point2f>> &marker_corners, std::vector<float> &variances);
    bool imgToCV(const sensor_msgs::Image::ConstPtr &image_msg, cv_bridge::CvImagePtr &cv_ptr);
    float euclideanDistance(pcl::PointXYZ &p1, pcl::PointXYZ &p2);
};

#endif /* ARUCO_CUBE_PERCEIVER_H_ */
