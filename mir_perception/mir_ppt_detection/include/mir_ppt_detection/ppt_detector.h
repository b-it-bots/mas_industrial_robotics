#ifndef PPT_DETECTOR_H
#define PPT_DETECTOR_H

#include <math.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <mir_ppt_detection/Cavity.h>
#include <mir_ppt_detection/Cavities.h>
#include <mir_ppt_detection/min_distance_to_hull_calculator.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBA PointRGBA;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointRGBA> PointCloudRGBA;
typedef pcl::PointIndices PointIndices;

class PPTDetector
{
    public:
        PPTDetector();

    protected:

        PointRGBA get_point_rgba(const pcl::PointXYZRGB& pt_rgb);
        PointCloudRGBA::Ptr get_point_cloud_rgba(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb);

        template<typename T>
        void downsample_organized_cloud(T cloud_in, PointCloud::Ptr cloud_downsampled, int scale);

        bool compute_dominant_plane_and_hull(PointCloud::Ptr cloud_in,
                                             pcl::ModelCoefficients::Ptr plane_coeffs,
                                             PointCloud::Ptr hull);

        void project_points_to_plane(PointCloud::Ptr cloud_in,
                                     pcl::ModelCoefficients::Ptr plane_coeffs,
                                     PointIndices::Ptr planar_indices, 
                                     PointIndices::Ptr non_planar_indices,
                                     PointCloudRGBA::Ptr cloud_projected);

        void extract_polygonal_prism_inliers(PointCloudRGBA::Ptr cloud_in,
                                             PointIndices::Ptr indices_in,
                                             PointCloud::Ptr cloud_hull,
                                             PointIndices::Ptr hull_inlier_indices);

        static bool customRegionGrowing1 (const PointRGBA& point_a, const PointRGBA& point_b,
                                   float squared_distance);

        static bool customRegionGrowing2 (const PointRGBA& point_a, const PointRGBA& point_b,
                                   float squared_distance);

        float get_non_planar_pt_frac(PointCloudRGBA::Ptr cloud_in);

        void compute_cavity_clusters(PointCloudRGBA::Ptr cloud_in,
                                     PointIndices::Ptr planar_idx,
                                     PointIndices::Ptr non_planar_idx,
                                     pcl::IndicesClustersPtr clusters);

        void cloud_cb (const PointCloud::ConstPtr& input);

        ros::NodeHandle nh_;
        ros::Subscriber pc_sub_;

        bool debug_pub = true;

        int downsample_scale = 3;
        float planar_projection_thresh = 0.015; 
        float cam_cx = 320.0;
        float cam_cy = 245.1;
        float cam_fx = 615.8;
        float cam_fy = 615.6;
        float min_cavity_area = 1e-4;

        ros::Publisher cloud_pub0, cloud_pub1, cloud_pub2;
        ros::Publisher cavity_pub;
        MinDistanceToHullCalculator dist_to_hull;

};

#endif
