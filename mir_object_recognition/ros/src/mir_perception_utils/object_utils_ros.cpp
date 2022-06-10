#include <fstream>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <utility>

#include <opencv4/opencv2/highgui/highgui.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
// pcl_ros is not converted to ros2 yet, hence commented out
// #include <pcl_ros/point_cloud.hpp>

#include "mir_perception_utils/object_utils_ros.hpp"

using namespace std::chrono_literals;
using namespace mir_perception_utils;

void object::estimatePose(const BoundingBox &box, geometry_msgs::msg::PoseStamped &pose)
{
    BoundingBox::Points vertices = box.getVertices();
    Eigen::Vector3f n1;
    Eigen::Vector3f n2;
    Eigen::Vector3f n3 = (vertices[4] - vertices[0]) / (vertices[4] - vertices[0]).norm();
    if ((vertices[1] - vertices[0]).norm() > (vertices[3] - vertices[0]).norm()) {
        n1 = (vertices[1] - vertices[0]) / (vertices[1] - vertices[0]).norm();
    } else {
        n1 = (vertices[3] - vertices[0]) / (vertices[3] - vertices[0]).norm();
    }
    n2 = n3.cross(n1);
    RCLCPP_INFO(rclcpp::get_logger("mir_perception_utils_logger"), "got norms");
    Eigen::Matrix3f m;
    m << n1, n2, n3;
    Eigen::Quaternion<float> q(m);
    q.normalize();

    Eigen::Vector3f centroid = box.getCenter();
    pose.pose.position.x = centroid(0);
    pose.pose.position.y = centroid(1);
    pose.pose.position.z = (vertices[0](2) + vertices[1](2) + vertices[2](2) + vertices[3](2)) / 4.0;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
}

void object::estimatePose(const PointCloudConstBSPtr &xyz_input_cloud,
                         geometry_msgs::msg::PoseStamped &pose,
                         const std::string shape,
                         float passthrouigh_lim_min_offset,
                         float passthrouigh_lim_max_offset)
{
    // Apply filter to remove points belonging to the plane for non
    // circular/spherical object
    // to find its orientation
    PointCloud filtered_cloud;
    if (shape == "sphere") {
        filtered_cloud = *xyz_input_cloud;
    } else {
        pcl::PassThrough<PointT> pass_through;
        pass_through.setFilterFieldName("z");
        PointT min_pt;
        PointT max_pt;
        pcl::getMinMax3D(*xyz_input_cloud, min_pt, max_pt);
        double limit_min = min_pt.z + passthrouigh_lim_min_offset;
        double limit_max = max_pt.z - passthrouigh_lim_max_offset;
        pass_through.setFilterLimits(limit_min, limit_max);
        pass_through.setInputCloud(xyz_input_cloud);
        pass_through.filter(filtered_cloud);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(filtered_cloud, centroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(filtered_cloud, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

    // swap largest and second largest eigenvector so that y-axis aligns with
    // largest eigenvector and z with the second largest
    eigen_vectors.col(0).swap(eigen_vectors.col(2));
    eigen_vectors.col(1) = eigen_vectors.col(2).cross(eigen_vectors.col(0));

    Eigen::Matrix4f eigen_vector_transform(Eigen::Matrix4f::Identity());
    eigen_vector_transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
    eigen_vector_transform.block<3, 1>(0, 3) = -(eigen_vector_transform.block<3, 3>(0, 0) * centroid.head<3>());

    // transform cloud to eigenvector space
    PointCloud transform_cloud;
    pcl::transformPointCloud(filtered_cloud, transform_cloud, eigen_vector_transform);

    // find mean diagonal
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(transform_cloud, min_pt, max_pt);
    Eigen::Vector3f mean_diag = (max_pt.getVector3fMap() + min_pt.getVector3fMap()) / 2.0;

    // orientation and position of bounding box of cloud
    Eigen::Quaternionf orientation(eigen_vectors);
    Eigen::Vector3f position = eigen_vectors * mean_diag + centroid.head<3>();

    pose.pose.position.x = position(0);
    pose.pose.position.y = position(1);
    pose.pose.position.z = position(2);
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();

    tf2::Quaternion quaternion;
    // convertsion method changed in tf2 - need to verify
    // original - tf::quaternionMsgToTF(pose.pose.orientation, quaternion);
    tf2::convert(pose.pose.orientation, quaternion);
    tf2::Matrix3x3 m(quaternion);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // convertsion method changed in tf2 - need to verify
    // original - pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
}

void object::transformPose(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
                           const std::string &target_frame,
                           const geometry_msgs::msg::PoseStamped &pose,
                           geometry_msgs::msg::PoseStamped &transformed_pose)
{
    if (tf_buffer){
        bool canTransform = tf_buffer -> canTransform(target_frame, pose.header.frame_id, 
                                            rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.1));
        if (canTransform)
        {
            try{
                tf_buffer -> transform(pose, transformed_pose, target_frame);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mir_perception_utils_logger"), "Transform error: %s", ex.what());
                transformed_pose = pose;
            }
        }
        else{
            auto steady_clock = rclcpp::Clock();
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("mir_perception_utils_logger"), steady_clock,
                                    2000, "[ObjectUtils]: Pose cannot be transformed.");
            transformed_pose = pose;
        }
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("mir_perception_utils_logger"), "[ObjectUtils]: TF buffer is not initialized.");
        transformed_pose = pose;
    }
}

void object::get3DBoundingBox(const PointCloudConstBSPtr &cloud,
                              const Eigen::Vector3f &normal,
                              BoundingBox &bbox,
                              mas_perception_msgs::msg::BoundingBox &bounding_box_msg)
{
    bbox = BoundingBox::create(cloud -> points, normal);
    convertBoundingBox(bbox, bounding_box_msg);
}

void convertBboxToMsg(const BoundingBox &bbox,
                        mas_perception_msgs::msg::BoundingBox &bounding_box_msg)
{
    convertBoundingBox(bbox, bounding_box_msg);
}

void object::savePcd(const PointCloudConstBSPtr &pointcloud, std::string log_dir, std::string obj_name)
{
  std::stringstream filename;
  filename.str("");
  filename << log_dir << obj_name << ".pcd";
  pcl::io::savePCDFileASCII(filename.str(), *pointcloud);
}

void object::saveCVImage(const cv_bridge::CvImagePtr &cv_image, std::string log_dir,
                         std::string obj_name)
{
  std::stringstream filename;
  filename.str("");
  filename << log_dir << obj_name << ".jpg";
  cv::imwrite(filename.str(), cv_image->image);
}

bool object::getCVImage(const std::shared_ptr<const sensor_msgs::msg::Image> &image,
                        cv_bridge::CvImagePtr &cv_image)
                        
{
    try{
        cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        return (true);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("mir_perception_utils_logger"), "cv_bridge exception: %s", e.what());
        return (false);
    }
}