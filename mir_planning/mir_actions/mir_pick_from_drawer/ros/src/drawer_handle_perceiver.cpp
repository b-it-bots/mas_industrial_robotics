#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

ros::Publisher cloud_pub;
ros::Publisher pose_pub;
ros::Publisher event_out_pub;
tf::TransformListener *listenerptr;
bool publish_output_pc;
bool listening = false;
std::string output_pc_frame;
float z_threshold_min;
float z_threshold_max;
float y_threshold_min;
float y_threshold_max;
float x_threshold;

void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
    if (!listening)
    {
        return;
    }

    //Downsample by x3
    int scale = 3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_downsampled->width = input->width / scale;
    cloud_downsampled->height = input->height / scale;
    cloud_downsampled->points.resize(cloud_downsampled->width * cloud_downsampled->height);
    for( size_t i = 0, ii = 0; i < cloud_downsampled->height; ii += scale, i++)
    {
        for( size_t j = 0, jj = 0; j < cloud_downsampled->width; jj += scale, j++)
        {
            cloud_downsampled->at(j, i) = input->at(jj, ii); //at(column, row)
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    try
    {
        ros::Time input_stamp;
        pcl_conversions::fromPCL(input->header.stamp, input_stamp);  
        tf::StampedTransform transform;
        listenerptr->waitForTransform(output_pc_frame, input->header.frame_id, input_stamp, ros::Duration(1.0));
        listenerptr->lookupTransform(output_pc_frame, input->header.frame_id, input_stamp, transform);
        pcl_ros::transformPointCloud(*cloud_downsampled, *cloud_transformed, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Could not find transform. Retrying...");
        /* ROS_ERROR("%s",ex.what()); */
        return;
    }

    // Filter points in z direction
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_transformed);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_threshold_min, z_threshold_max);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_pass_through);

    // Filter points in y direction
    pass.setInputCloud (cloud_pass_through);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_threshold_min, y_threshold_max);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through_2(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_pass_through_2);

    //Voxel grid filter for uniform pointcloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_pass_through_2);
    sor.setLeafSize (0.005f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter (*cloud_filtered);

    //Estimate most dominant plane coefficients
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.005);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);

    //Project plane model inliers to plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> project_inliers;
    project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    project_inliers.setInputCloud(cloud_filtered);
    project_inliers.setModelCoefficients(coefficients);
    project_inliers.setIndices(inliers);
    project_inliers.setCopyAllData(false);
    project_inliers.filter(*plane);

    //Compute plane convex hull
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud(plane);
    convex_hull.reconstruct(*hull);

    //Extract points inside polygonal prism of plane convex hull
    pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> extract_polygonal_prism;
    extract_polygonal_prism.setInputPlanarHull(hull);
    extract_polygonal_prism.setInputCloud(cloud_pass_through_2);
    extract_polygonal_prism.setHeightLimits(0.005, 0.1);
    extract_polygonal_prism.segment(*segmented_cloud_inliers);

    // Extract the inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_pass_through_2);
    extract.setIndices (segmented_cloud_inliers);
    extract.setNegative (false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter (*segmented_cloud);

    /* //Find clusters of the inlier points */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusters_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (segmented_cloud);
    ec.extract (clusters_indices);

    float closest_dist = 100.0;
    Eigen::Vector4f closest_centroid(0.0, 0.0, 0.0, 0.0);
    Eigen::Vector4f zero_point(0.0, 0.0, 0.0, 0.0);
    for (size_t i = 0; i < clusters_indices.size(); i++)
    {
        const pcl::PointIndices& cluster_indices = clusters_indices[i];
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*segmented_cloud, cluster_indices, centroid);
        float dist = pcl::L2_Norm(centroid, zero_point, 3);
        if (dist < closest_dist)
        {
            closest_dist = dist;
            closest_centroid = centroid;
        }
    }
    /* std::cout << closest_dist << std::endl; */
    /* std::cout << closest_centroid << std::endl; */

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = closest_centroid[0];
    pose_stamped.pose.position.y = closest_centroid[1];
    pose_stamped.pose.position.z = closest_centroid[2];
    pose_stamped.header.frame_id = output_pc_frame;
    pose_stamped.header.stamp = ros::Time::now();
    pose_pub.publish(pose_stamped);

    std_msgs::String event_out_msg;
    event_out_msg.data = "e_done";
    event_out_pub.publish (event_out_msg);
    ROS_INFO("SUCCESSFULL");

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*segmented_cloud, output);
    output.header.frame_id = output_pc_frame;
    output.header.stamp = ros::Time::now();
    cloud_pub.publish (output);
    
    listening = false;
}

void event_in_cb (const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "e_start")
    {
        ROS_INFO_STREAM("starting listening");
        listening = true;
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "drawer_handle_perceiver");
    ros::NodeHandle nh("~");

    /* ros params */
    nh.param<bool>("publish_output_pc", publish_output_pc, "true");
    nh.param<std::string>("output_pc_frame", output_pc_frame, "base_link_static");
    nh.param<float>("x_threshold", x_threshold, 0.05);
    nh.param<float>("z_threshold_min", z_threshold_min, -0.1);
    nh.param<float>("z_threshold_max", z_threshold_max, 0.3);
    nh.param<float>("y_threshold_min", y_threshold_min, -0.1);
    nh.param<float>("y_threshold_max", y_threshold_max, 0.1);

    tf::TransformListener listener;
    listenerptr = &listener;

    /* Create a ROS subscriber for the input point cloud */
    ros::Subscriber pc_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("input", 1, cloud_cb);
    ros::Subscriber event_in_sub = nh.subscribe<std_msgs::String> ("event_in", 1, event_in_cb);
    event_out_pub = nh.advertise<std_msgs::String> ("event_out", 1);

    if (publish_output_pc)
    {
        // Create a ROS publisher for the output point cloud
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    }

    pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("output_pose", 1);

    // Spin
    ros::spin ();
}
