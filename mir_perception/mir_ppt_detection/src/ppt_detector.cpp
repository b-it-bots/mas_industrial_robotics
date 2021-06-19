#include <math.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mir_ppt_detection/Cavity.h>
#include <mir_ppt_detection/Cavities.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
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
#include <eigen_conversions/eigen_msg.h>
#include <min_distance_to_hull_calculator.hpp>

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

PointRGBA get_point_rgba(const pcl::PointXYZRGB& pt_rgb){
    PointRGBA pt_rgba;
    pt_rgba.x = pt_rgb.x;
    pt_rgba.y = pt_rgb.y;
    pt_rgba.z = pt_rgb.z;
    pt_rgba.r = pt_rgb.r;
    pt_rgba.g = pt_rgb.g;
    pt_rgba.b = pt_rgb.b;
    return pt_rgba;
}

PointCloudRGBA::Ptr get_point_cloud_rgba(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb){
    PointCloudRGBA::Ptr cloud_rgba(new PointCloudRGBA);
    for( size_t i = 0;  i < cloud_rgb->points.size(); i++){
        cloud_rgba->points.push_back(get_point_rgba(cloud_rgb->points[i]));
    }   
    return cloud_rgba; 
}

template<typename T>
void downsample_organized_cloud(T cloud_in, PointCloud::Ptr cloud_downsampled, int scale) {
    cloud_downsampled->width = cloud_in->width / scale;
    cloud_downsampled->height = cloud_in->height / scale;
    cloud_downsampled->points.resize(cloud_downsampled->width * cloud_downsampled->height);
    for( size_t i = 0, ii = 0; i < cloud_downsampled->height; ii += scale, i++){
        for( size_t j = 0, jj = 0; j < cloud_downsampled->width; jj += scale, j++){
            cloud_downsampled->at(j, i) = cloud_in->at(jj, ii);
        }
    }
}

bool compute_dominant_plane_and_hull(PointCloud::Ptr cloud_in,
                                     pcl::ModelCoefficients::Ptr plane_coeffs,
                                     PointCloud::Ptr hull){
    //Estimate most dominant plane coefficients and inliers
    PointIndices::Ptr inliers (new PointIndices);
    pcl::SACSegmentation<PointT> sac_seg;
    sac_seg.setModelType (pcl::SACMODEL_PLANE);
    sac_seg.setMethodType (pcl::SAC_RANSAC);
    sac_seg.setOptimizeCoefficients (true);
    sac_seg.setDistanceThreshold (0.01);
    sac_seg.setInputCloud (cloud_in);
    sac_seg.segment (*inliers, *plane_coeffs);

    if (inliers->indices.size() > cloud_in->points.size() / 5) {
        //Project plane model inliers to plane
        PointCloud::Ptr cloud_plane(new PointCloud);
        pcl::ProjectInliers<PointT> project_inliers;
        project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
        project_inliers.setInputCloud(cloud_in);
        project_inliers.setModelCoefficients(plane_coeffs);
        project_inliers.setIndices(inliers);
        project_inliers.setCopyAllData(false);
        project_inliers.filter(*cloud_plane);

        //Compute plane convex hull
        pcl::ConvexHull<PointT> convex_hull;
        convex_hull.setInputCloud(cloud_plane);
        convex_hull.reconstruct(*hull);
        hull->points.push_back(hull->at(0));
        return true;
    } else {
        return false;
    }
}

void project_points_to_plane(PointCloud::Ptr cloud_in,
                             pcl::ModelCoefficients::Ptr plane_coeffs,
                             PointIndices::Ptr planar_indices, 
                             PointIndices::Ptr non_planar_indices,
                             PointCloudRGBA::Ptr cloud_projected){ 
    cloud_projected->width = cloud_in->width;
    cloud_projected->height = cloud_in->height;
    for( size_t row = 0;  row < cloud_in->height; row++){
        for( size_t col = 0; col < cloud_in->width; col++){
            PointT pt_in = cloud_in->at(col, row);

            float x_bar = (downsample_scale*col-cam_cx)/cam_fx;
            float y_bar = (downsample_scale*row-cam_cy)/cam_fy;        
            float z = -plane_coeffs->values[3] / (plane_coeffs->values[0]*x_bar + plane_coeffs->values[1]*y_bar + plane_coeffs->values[2]) + 5e-4f*rand()/(RAND_MAX);
            PointRGBA pt_projected = get_point_rgba(pt_in);
            pt_projected.x = x_bar*z;
            pt_projected.y = y_bar*z;
            pt_projected.z = z;

            float pt_in_dist = sqrt(pow(pt_in.x,2) + pow(pt_in.y,2) + pow(pt_in.z,2)); 
            float pt_proj_dist = sqrt(pow(pt_projected.x,2) + pow(pt_projected.y,2) + pow(pt_projected.z,2));
            if (fabs(pt_in_dist - pt_proj_dist) < planar_projection_thresh) {
                planar_indices->indices.push_back(col + row*cloud_in->width);      
                pt_projected.a = 0;
            } else if (std::isnan(pt_in_dist) || pt_in_dist > pt_proj_dist){
                non_planar_indices->indices.push_back(col + row*cloud_in->width);       
                pt_projected.a = 1;
            } 
            cloud_projected->points.push_back(pt_projected);
        }
    }
}

void extract_polygonal_prism_inliers(PointCloudRGBA::Ptr cloud_in,
                                     PointIndices::Ptr indices_in,
                                     PointCloud::Ptr cloud_hull,
                                     PointIndices::Ptr hull_inlier_indices){
    pcl::ExtractPolygonalPrismData<PointRGBA> epp;
    PointCloudRGBA::Ptr cloud_hull_rgba = get_point_cloud_rgba(cloud_hull);
    epp.setInputPlanarHull(cloud_hull_rgba);
    epp.setInputCloud(cloud_in);
    epp.setIndices(indices_in);
    double z_min = -planar_projection_thresh;
    double z_max = planar_projection_thresh;
    epp.setHeightLimits(z_min, z_max);
    epp.segment(*hull_inlier_indices);
}

bool customRegionGrowing1 (const PointRGBA& point_a, const PointRGBA& point_b,
                           float squared_distance){
    float thresh = 5;
    if (fabs(point_a.r-point_b.r) < thresh && fabs(point_a.g-point_b.g) < thresh && fabs(point_a.b-point_b.b) < thresh){
        return true;
    }
    return false;
}

bool customRegionGrowing2 (const PointRGBA& point_a, const PointRGBA& point_b,
                           float squared_distance){
    if (point_a.a == 1 && point_b.a == 1){
        return true; 
    } else if (point_a.a == 1 || point_b.a == 1){
        float thresh = 5;
        if (fabs(point_a.r-point_b.r) < thresh && fabs(point_a.g-point_b.g) < thresh && fabs(point_a.b-point_b.b) < thresh){
            return true;
        } 
    }
    return false;
}

float get_non_planar_pt_frac(PointCloudRGBA::Ptr cloud_in){
    int non_planar_pt_cnt = 0;
    for( size_t i= 0;  i < cloud_in->points.size(); i++){
        if (cloud_in->points[i].a == 1) {non_planar_pt_cnt++;}
    }
    return (float)non_planar_pt_cnt/cloud_in->points.size();
}

void compute_cavity_clusters(PointCloudRGBA::Ptr cloud_in,
                             PointIndices::Ptr planar_idx,
                             PointIndices::Ptr non_planar_idx,
                             pcl::IndicesClustersPtr clusters){
    pcl::ConditionalEuclideanClustering<PointRGBA> cec;
    cec.setInputCloud (cloud_in);

    pcl::IndicesClustersPtr planar_cavity_candidate_idx_clusters (new pcl::IndicesClusters);
    cec.setIndices (planar_idx);
    cec.setClusterTolerance (0.005);
    cec.setMinClusterSize (1);
    cec.setMaxClusterSize (cloud_in->points.size() / 50);
    cec.setConditionFunction (&customRegionGrowing1);
    cec.segment (*planar_cavity_candidate_idx_clusters);

    pcl::PointIndices::Ptr cavity_candidate_indices (new pcl::PointIndices (*non_planar_idx));
    for (std::vector<PointIndices>::const_iterator cluster_it = planar_cavity_candidate_idx_clusters->begin (); 
            cluster_it != planar_cavity_candidate_idx_clusters->end (); ++cluster_it) {
        cavity_candidate_indices->indices.insert(cavity_candidate_indices->indices.end(), cluster_it->indices.begin(), cluster_it->indices.end());
    }

    cec.setIndices (cavity_candidate_indices);
    cec.setClusterTolerance (0.01);
    cec.setMinClusterSize (cloud_in->points.size() / 400);
    cec.setMaxClusterSize (cloud_in->points.size() / 10);
    cec.setConditionFunction (&customRegionGrowing2);
    cec.segment (*clusters);
}

void 
//cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
cloud_cb (const PointCloud::ConstPtr& input)
{
    ros::Time t = ros::Time::now();
    //Downsample by downsample_scale
    PointCloud::Ptr cloud_downsampled(new PointCloud);
    downsample_organized_cloud(input, cloud_downsampled, downsample_scale);

    //Downsample by downsample_scale a second time
    PointCloud::Ptr cloud_downsampled_x2(new PointCloud);
    downsample_organized_cloud(cloud_downsampled, cloud_downsampled_x2, downsample_scale);

    //Estimate most dominant plane coefficients and hull
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
    PointCloud::Ptr cloud_hull(new PointCloud);

    if (compute_dominant_plane_and_hull(cloud_downsampled_x2, plane_coefficients, cloud_hull)) 
    {
        dist_to_hull.setConvexHullPointsAndEdges(get_point_cloud_rgba(cloud_hull));   
        std::cout << "plane segmentation time: " << ros::Time::now().toSec() - t.toSec() << std::endl; 

        PointIndices::Ptr planar_indices (new PointIndices);
        PointIndices::Ptr non_planar_indices (new PointIndices);
        PointCloudRGBA::Ptr cloud_projected(new PointCloudRGBA);
        project_points_to_plane(cloud_downsampled, plane_coefficients, planar_indices,
                                non_planar_indices, cloud_projected);
        std::cout << "plane projection time: "
                  << ros::Time::now().toSec() - t.toSec() << std::endl; 

        PointIndices::Ptr planar_hull_inlier_indices (new PointIndices);
        extract_polygonal_prism_inliers(cloud_projected, planar_indices,
                                        cloud_hull, planar_hull_inlier_indices);
        PointIndices::Ptr non_planar_hull_inlier_indices (new PointIndices);
        extract_polygonal_prism_inliers(cloud_projected, non_planar_indices,
                                        cloud_hull, non_planar_hull_inlier_indices);
        std::cout << "hull inlier extraction time: "
                  << ros::Time::now().toSec() - t.toSec() << std::endl; 

        pcl::IndicesClustersPtr cavity_clusters (new pcl::IndicesClusters);
        compute_cavity_clusters(cloud_projected, planar_hull_inlier_indices,
                                non_planar_hull_inlier_indices, cavity_clusters);
        std::cout << "Number of clusters: " << cavity_clusters->size() << std::endl;
        std::cout << "cavity clustering time: "
                  << ros::Time::now().toSec() - t.toSec() << std::endl; 

        PointIndices::Ptr cavity_cluster_indices (new PointIndices);
        for (std::vector<PointIndices>::const_iterator cluster_it = cavity_clusters->begin ();
             cluster_it != cavity_clusters->end (); ++cluster_it)
        {
            cavity_cluster_indices->indices.insert(cavity_cluster_indices->indices.end(),
                                                   cluster_it->indices.begin(),
                                                   cluster_it->indices.end());
        }

        pcl::ExtractIndices<PointRGBA> extract (true);
        extract.setInputCloud (cloud_projected);
        PointCloudRGBA::Ptr cloud_cavity(new PointCloudRGBA);
        pcl::ConvexHull<PointRGBA> convex_hull;
        convex_hull.setComputeAreaVolume(true);
        PointCloudRGBA::Ptr cavity_hull(new PointCloudRGBA);
        mir_ppt_detection::Cavities cavities_msg;
        for (std::vector<pcl::PointIndices>::const_iterator cluster_it = cavity_clusters->begin ();
                cluster_it != cavity_clusters->end (); ++cluster_it)
        {
            PointIndices::Ptr cavity_indices (new PointIndices (*cluster_it));
            extract.setIndices (cavity_indices);
            extract.filter (*cloud_cavity);
            if (get_non_planar_pt_frac(cloud_cavity) > 0.6){
                convex_hull.setInputCloud(cloud_cavity);
                convex_hull.reconstruct(*cavity_hull);
                if (convex_hull.getTotalArea() > min_cavity_area &&
                    dist_to_hull.computeMinDistanceToHull(cavity_hull) > 0.01) {
                    std::cerr << "Cavity cloud points added: " << cloud_cavity->points.size () << std::endl;
                    PointCloudRGBA::Ptr cavity_cloud_filtered(new PointCloudRGBA);
                    pcl::VoxelGrid<PointRGBA> sor;
                    sor.setInputCloud (cloud_cavity);
                    sor.setLeafSize (0.002f, 0.002f, 0.002f);
                    sor.filter (*cavity_cloud_filtered);

                    Eigen::Vector4f pcaCentroid;
                    pcl::compute3DCentroid(*cavity_cloud_filtered, pcaCentroid);
                    Eigen::Matrix3f covariance;
                    computeCovarianceMatrixNormalized(*cavity_cloud_filtered,
                                                      pcaCentroid, covariance);
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
                            covariance, Eigen::ComputeEigenvectors);
                    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();      

                    int first_principle_component_direction_signum;
                    float first_principle_component_displacement;
                    float second_principle_component_displacement;
                    float displacement_metric = 0.0f;
                    Eigen::Vector3f pt_to_centroid_vec, pcaCentroid_vec3f = pcaCentroid.head<3>();
                    for (std::vector<PointRGBA, Eigen::aligned_allocator<PointRGBA> >::const_iterator cpt = cavity_cloud_filtered->points.begin ();
                            cpt != cavity_cloud_filtered->points.end (); ++cpt){   
                        pt_to_centroid_vec = cpt->getVector3fMap()-pcaCentroid_vec3f;
                        first_principle_component_displacement = eigenVectorsPCA.col(2).dot(pt_to_centroid_vec);
                        second_principle_component_displacement = eigenVectorsPCA.col(1).dot(pt_to_centroid_vec);
                        displacement_metric += first_principle_component_displacement * pow(second_principle_component_displacement,2);
                    }
                    first_principle_component_direction_signum = displacement_metric/fabs(displacement_metric);
                    eigenVectorsPCA.col(0) = first_principle_component_direction_signum * eigenVectorsPCA.col(2);
                    // Ensure orientation z-axis (3rd principle component / eigenvector column) points towards camera
                    if (eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1))[2] > 0){
                        eigenVectorsPCA.col(1) = -eigenVectorsPCA.col(1);
                    } 
                    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));


                    mir_ppt_detection::Cavity cavity_msg;
                    cavity_msg.cov_minor = eigen_solver.eigenvalues()[1];
                    cavity_msg.cov_major = eigen_solver.eigenvalues()[2];
                    geometry_msgs::Pose pose;
                    Eigen::Quaternionf quaternionPCA(eigenVectorsPCA);
                    pose.orientation.x = quaternionPCA.x();
                    pose.orientation.y = quaternionPCA.y();
                    pose.orientation.z = quaternionPCA.z();
                    pose.orientation.w = quaternionPCA.w();
                    pose.position.x = pcaCentroid[0];
                    pose.position.y = pcaCentroid[1];
                    pose.position.z = pcaCentroid[2];
                    cavity_msg.pose = pose;
                    cavities_msg.cavities.push_back(cavity_msg);
                    std::cout << "eigenval minor: " << eigen_solver.eigenvalues()[1]
                              << "   eigenval major: " << eigen_solver.eigenvalues()[2]  << std::endl;      
                }
            }
        }
        cavity_pub.publish(cavities_msg);
        std::cout << "finished processing time: "
                  << ros::Time::now().toSec() - t.toSec() << std::endl; 

        if (debug_pub){
            PointCloudRGBA::Ptr output_cloud(new PointCloudRGBA);
            sensor_msgs::PointCloud2 output;

            extract.setIndices (non_planar_hull_inlier_indices);
            extract.filter (*output_cloud);
            output_cloud->width = output_cloud->points.size ();
            output_cloud->height = 1;
            output_cloud->is_dense = true;
            pcl::toROSMsg(*output_cloud, output);
            output.header.frame_id = input->header.frame_id ;
            output.header.stamp = ros::Time::now();
            cloud_pub0.publish (output);

            extract.setIndices (planar_hull_inlier_indices);
            extract.filter (*output_cloud);
            output_cloud->width = output_cloud->points.size ();
            output_cloud->height = 1;
            output_cloud->is_dense = true;
            pcl::toROSMsg(*output_cloud, output);
            output.header.frame_id = input->header.frame_id ;
            output.header.stamp = ros::Time::now();
            cloud_pub1.publish (output);

            extract.setIndices (cavity_cluster_indices);
            extract.filter (*output_cloud);
            output_cloud->width = output_cloud->points.size ();
            output_cloud->height = 1;
            output_cloud->is_dense = true;
            pcl::toROSMsg(*output_cloud, output);
            output.header.frame_id = input->header.frame_id ;
            output.header.stamp = ros::Time::now();
            cloud_pub2.publish (output);
        }
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ppt_3d_detector");
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<PointCloud> ("points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    cloud_pub0 = nh.advertise<sensor_msgs::PointCloud2> ("cloud_non_planar", 1);
    cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2> ("cloud_planar", 1);
    cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2> ("cloud_cavity_clusters", 1);
    cavity_pub = nh.advertise<mir_ppt_detection::Cavities> ("cavities", 1);

    // Spin
    ros::spin ();
}
