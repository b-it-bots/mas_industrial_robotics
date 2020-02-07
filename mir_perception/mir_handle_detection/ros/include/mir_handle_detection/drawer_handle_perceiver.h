#ifndef DRAWER_HANDLE_PERCEIVER_H
#define DRAWER_HANDLE_PERCEIVER_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>

#include <Eigen/Eigenvalues>

typedef pcl::PointCloud<pcl::PointXYZ> PCloudT;
class DrawerHandlePerceiver
{
    public:
        DrawerHandlePerceiver();
        virtual ~DrawerHandlePerceiver();

    private:
        ros::NodeHandle nh;
        ros::Subscriber pc_sub;
        ros::Subscriber event_in_sub;
        ros::Publisher pc_pub;
        ros::Publisher pose_pub;
        ros::Publisher event_out_pub;

        std::string output_frame;
        bool enable_debug_pc_pub;
        bool is_running;
        int retry_attempts_;
        int num_of_retries_;
        tf::TransformListener tf_listener;

        pcl::PassThrough<pcl::PointXYZ> passthrough_filter_y;
        pcl::PassThrough<pcl::PointXYZ> passthrough_filter_z;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ProjectInliers<pcl::PointXYZ> project_inliers;
        pcl::ConvexHull<pcl::PointXYZ> convex_hull;
        pcl::ExtractPolygonalPrismData<pcl::PointXYZ> extract_polygonal_prism;
        pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster_extraction;

        void checkFailure();
        void pcCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void eventInCallback(const std_msgs::String::ConstPtr &msg);
        bool transformPC(const sensor_msgs::PointCloud2::ConstPtr &msg, sensor_msgs::PointCloud2 &msg_transformed);
        void passthroughFilterPC(const PCloudT::Ptr &input, PCloudT::Ptr output);
        void extractPlaneOutlier(const PCloudT::Ptr &input, PCloudT::Ptr dense_input, PCloudT::Ptr output);
        bool getClosestCluster(const PCloudT::Ptr &input, Eigen::Vector4f &closest_centroid);
};
#endif
