/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MIR_OBJECT_SEGMENTATION_SCENE_SEGMENTATION_ROS_H
#define MIR_OBJECT_SEGMENTATION_SCENE_SEGMENTATION_ROS_H

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <mas_perception_msgs/BoundingBox.h>
#include <mas_perception_msgs/ObjectList.h>

#include <mir_object_segmentation/cloud_accumulation.h>
#include <mir_object_segmentation/scene_segmentation.h>

#include <mir_perception_utils/bounding_box.h>
#include <mir_perception_utils/object_utils_ros.h>

/** \brief This class is a wrapper for table top point cloud segmentation.
 *
 * \author Mohammad Wasil, Santosh Thoduka
 */

class SceneSegmentationROS
{
 public:
  /** \brief Constructor
   * \param[in] NodeHandle in order for this to have access to parameters
   * for example octree_resolution.
   * */
  explicit SceneSegmentationROS(double octree_resolution = 0.0025);

  /** \brief Destructor */
  virtual ~SceneSegmentationROS();

 private:
  ros::NodeHandle nh_;
  ros::ServiceClient recognize_service;

  /** Create unique pointer object of cloud_accumulation */
  CloudAccumulation::UPtr cloud_accumulation_;
  /** Create unique pointer for object scene_segmentation */
  typedef std::unique_ptr<SceneSegmentation> SceneSegmentationUPtr;
  SceneSegmentationUPtr scene_segmentation_;

  pcl::ModelCoefficients::Ptr model_coefficients_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;

  bool add_to_octree_;
  int pcl_object_id_;
  double octree_resolution_;
  double workspace_height_;

 public:
  /** \brief Find plane, segment table top point cloud and cluster them
   * \param[in] Input point cloud
   * \param[out] Object list with unknown labels
   * \param[out] 3D table top object clusters
   * \param[out] Bounding boxes of the clusters
   * */
  void segmentCloud(const PointCloud::ConstPtr &cloud, mas_perception_msgs::ObjectList &obj_list,
                    std::vector<PointCloud::Ptr> &clusters, std::vector<BoundingBox> &boxes);

  /** \brief Find plane
   * \param[in] Input point cloud
   * \param[out] Point cloud debug output
   * */
  void findPlane(const PointCloud::ConstPtr &cloud_in, PointCloud::Ptr &cloud_debug);

  /** \brief Reset accumulated cloud */
  void resetCloudAccumulation();

  /** \brief Accumulate pointcloud
   * \param[in] Pointcloud to accumulate
   * */
  void addCloudAccumulation(const PointCloud::Ptr &cloud);

  /** \brief Get accumulated pointcloud
   * \param[out] Accumulated pointcloud
   * */
  void getCloudAccumulation(PointCloud::Ptr &cloud);

  /** Returns plane normal */
  Eigen::Vector3f getPlaneNormal();

  /** Returns plane height */
  double getWorkspaceHeight();

  /** Reset 3D object id */
  void resetPclObjectId();

  /** \brief Set voxel grid parameters
   * \param[in] Leaf size for x,y,z
   * \param[in] Field name, on which axis the filter will be applied
   * \param[in] The minimum allowed the field value
   * \param[in] The maximum allowed the field value
   * */
  void setVoxelGridParams(double voxel_leaf_size, std::string voxel_filter_field_name,
                          double voxel_filter_limit_min, double voxel_filter_limit_max);

  /** \brief Set passthrough filter parameters
   * \param[in] Enable or disable passthrough filter
   * \param[in] Field name, on which axis the filter will be applied
   * \param[in] The minimum allowed the field value
   * \param[in] The maximum allowed the field value
   * */
  void setPassthroughParams(bool enable_passthrough_filter,
                            std::string passthrough_filter_field_name,
                            double passthrough_filter_limit_min,
                            double passthrough_filter_limit_max);

  /** \brief Set Normal param using radius
   * \param[in] Radius search
   * \param[in] Use Open MP (OMP) for parallel normal estimation using cpu
   * (default False)
   * \param[in] Number of cores to use for computing normal with OMP (default=4)
   * */
  void setNormalParams(double normal_radius_search, bool use_omp = false, int num_cores = 4);

  /** \brief Set SAC parameters
   * \param[in] The maximum number of iterations the algorithm will run for
   * \param[in] The distance to model threshold
   * \param[in] Model coefficient refinement
   * \param[in] The axis to which the plane should be perpendicular
   * \param[in] The maximum allowed difference between the model normal and the
   * given axis in radians
   * \param[in] The relative weight (between 0 and 1) to give to the angular
   * distance (0 to pi/2) between point normals and the plane normal.
   * */
  void setSACParams(int sac_max_iterations, double sac_distance_threshold,
                    bool sac_optimize_coefficients, Eigen::Vector3f axis, double sac_eps_angle,
                    double sac_normal_distance_weight);

  /** \brief Set prism parameters
   * \param[in] The minimum height above the plane from which to construct the
   * polygonal prism
   * \param[in] The maximum height above the plane from which to construct the
   * polygonal prism
   * */
  void setPrismParams(double prism_min_height, double prism_max_height);

  /** \brief Set outliers parameters
   * \param[in] Radius of the sphere that will determine which points are
   * neighbors.
   * \param[in] The number of neighbors that need to be present in order to be
   * classified as an inlier.
   * */
  void setOutlierParams(double outlier_radius_search, double outlier_min_neighbors);

  /** \brief Set cluster parameters
   * \param[in] The spatial tolerance as a measure in the L2 Euclidean space
   * \param[in] The minimum number of points that a cluster must contain in
   * order to be accepted
   * \param[in] The maximum number of points that a cluster must contain in
   * order to be accepted
   * \param[in] The minimum height of the cluster above the given polygon
   * \param[in] The maximum height of the cluster above the given polygon
   * \param[in] The maximum length of the cluster
   * \param[in] The minimum height of the cluster above the given polygon
   * */
  void setClusterParams(double cluster_tolerance, int cluster_min_size, int cluster_max_size,
                        double cluster_min_height, double cluster_max_height,
                        double cluster_max_length, double cluster_min_distance_to_polygon);
};

#endif  // MIR_OBJECT_SEGMENTATION_SCENE_SEGMENTATION_ROS_H
