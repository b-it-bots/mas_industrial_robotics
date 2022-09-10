#include "mir_perception_utils/pointcloud_utils_ros.hpp"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv4/opencv2/core/core.hpp>

using namespace mir_perception_utils;

bool pointcloud::transformPointCloudMsg(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                            const std::string &target_frame,
                            const sensor_msgs::msg::PointCloud2 &cloud_in,
                            sensor_msgs::msg::PointCloud2 &cloud_out)
{
    if (tf_buffer){
        geometry_msgs::msg::TransformStamped transformStamped;
        try{
            transformStamped = tf_buffer -> lookupTransform(target_frame, cloud_in.header.frame_id,
                                                            tf2::TimePointZero);

            pcl_ros::transformPointCloud(target_frame, transformStamped, cloud_in, cloud_out);
            cloud_out.header.frame_id = target_frame;
        }
        catch (tf2::TransformException &ex){
            RCLCPP_ERROR(rclcpp::get_logger("mir_perception_utils"), "Transform error: %s", ex.what());
            return (false);
        }
    }
    else{
        rclcpp::Clock steady_clock = rclcpp::Clock();
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("mir_perception_utils_logger"), steady_clock,
                                    2000, "[ObjectUtils]: TF buffer is not initialized.");
        return (false);
    }
    return (true);
}

bool pointcloud::transformPointCloud(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                        const std::string &target_frame,
                        const PointCloud &cloud_in,
                        PointCloud &cloud_out)
{
    if (tf_buffer){
        geometry_msgs::msg::TransformStamped transformStamped;
        try{
            transformStamped = tf_buffer -> lookupTransform(target_frame, cloud_in.header.frame_id,
                                                            tf2::TimePointZero,
                                                            tf2::Duration(std::chrono::nanoseconds(10000000)));
            pcl_ros::transformPointCloud(cloud_in, cloud_out, transformStamped);
            cloud_out.header.frame_id = target_frame;
        }
        catch (tf2::TransformException &ex){
            RCLCPP_ERROR(rclcpp::get_logger("mir_perception_utils"), "Transform error: %s", ex.what());
            return (false);
        }
    }
    else{
        rclcpp::Clock steady_clock = rclcpp::Clock();
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("mir_perception_utils_logger"), steady_clock,
                                    2000, "[ObjectUtils]: TF buffer is not initialized.");
        return (false);
    }
    return (true);
}

bool pointcloud::transformPointCloud2(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                                 const std::string &target_frame,
                                 const pcl::PCLPointCloud2 &cloud_in_pc2,
                                 pcl::PCLPointCloud2 &cloud_out_pc2)
{
    if (tf_buffer){
        geometry_msgs::msg::TransformStamped transformStamped;
        try{
            PointCloud cloud_in;
            pcl::fromPCLPointCloud2(cloud_in_pc2, cloud_in);
            PointCloud cloud_out;
            transformStamped = tf_buffer -> lookupTransform(target_frame, cloud_in.header.frame_id,
                                                            tf2::TimePointZero,
                                                            tf2::Duration(std::chrono::nanoseconds(10000000)));
            pcl_ros::transformPointCloud(cloud_in, cloud_out, transformStamped);
            cloud_out.header.frame_id = target_frame;
            pcl::toPCLPointCloud2(cloud_out, cloud_out_pc2);
        }
        catch (tf2::TransformException &ex){
            RCLCPP_ERROR(rclcpp::get_logger("mir_perception_utils"), "Transform error: %s", ex.what());
            return (false);
        }
    }
    else{
        rclcpp::Clock steady_clock = rclcpp::Clock();
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("mir_perception_utils_logger"), steady_clock,
                                    2000, "[ObjectUtils]: TF buffer is not initialized.");
        return (false);
    }
    return (true);
}

bool pointcloud::getPointCloudROI(const sensor_msgs::msg::RegionOfInterest &roi,
                             const PointCloudBSPtr &cloud_in,
                             PointCloudBSPtr &cloud_roi,
                             float roi_size_adjustment, bool remove_outliers)
{
    if (cloud_in -> height <= 1 || cloud_in -> width <= 1){
        RCLCPP_ERROR(rclcpp::get_logger("mir_perception_utils"), "Cloud is empty.");
        return (false);
    }
    int min_x = roi.x_offset;
    int min_y = roi.y_offset;
    int max_x = roi.x_offset + roi.width;
    int max_y = roi.y_offset + roi.height;
    // Adjust roi
    if (roi.x_offset > roi_size_adjustment) min_x -= roi_size_adjustment;
    if (roi.y_offset > roi_size_adjustment) min_y -= roi_size_adjustment;
    if (roi.width + roi_size_adjustment < cloud_in -> width) max_x += roi_size_adjustment;
    if (roi.height + roi_size_adjustment < cloud_in -> height) max_y += roi_size_adjustment;

    std::vector<cv::Point> pixel_loc;

    for (int i = min_x; i < max_x; i++){
        for (int j = min_y; j < max_y; j++){
            cv::Point loc;
            loc.x = i;
            loc.y = j;
            pixel_loc.push_back(loc);
        }
    }
    for (size_t i = 0; i < pixel_loc.size(); i++){
        // check cloud_in size
        if (pixel_loc[i].x >= cloud_in -> width || pixel_loc[i].y >= cloud_in -> height){
            RCLCPP_ERROR(rclcpp::get_logger("mir_perception_utils"), "Pixel location is out of range.");
            return (false);
        }
        PointT pcl_point = cloud_in -> at(pixel_loc[i].x, pixel_loc[i].y);
        if ((!std::isnan(pcl_point.x)) && (!std::isnan(pcl_point.y)) && (!std::isnan(pcl_point.z)) &&
            (!std::isnan(pcl_point.r)) && (!std::isnan(pcl_point.g)) && (!std::isnan(pcl_point.b))){
            cloud_roi -> push_back(pcl_point);
        }
    }
    cloud_roi->header = cloud_in->header;
    if (remove_outliers){
        if (cloud_roi -> points.size() > 0){
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(cloud_roi);
            sor.setMeanK(50);
            sor.setStddevMulThresh(3.0);
            sor.filter(*cloud_roi);
        }
    }
    return (true);
}