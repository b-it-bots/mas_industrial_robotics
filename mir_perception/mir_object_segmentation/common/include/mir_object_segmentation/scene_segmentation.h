/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MIR_OBJECT_SEGMENTATION_SCENE_SEGMENTATION_H
#define MIR_OBJECT_SEGMENTATION_SCENE_SEGMENTATION_H

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <mir_perception_utils/aliases.h>
#include <mir_perception_utils/bounding_box.h>

using namespace mir_perception_utils::object;

class SceneSegmentation {
 private:
  pcl::PassThrough<PointT> pass_through_;
  pcl::VoxelGrid<PointT> voxel_grid_;
  pcl::NormalEstimation<PointT, PointNT> normal_estimation_;
  pcl::NormalEstimationOMP<PointT, PointNT> normal_estimation_omp_;

  pcl::SACSegmentationFromNormals<PointT, PointNT> sac_;
  pcl::ProjectInliers<PointT> project_inliers_;
  pcl::ConvexHull<PointT> convex_hull_;
  pcl::ExtractPolygonalPrismData<PointT> extract_polygonal_prism_;

  pcl::EuclideanClusterExtraction<PointT> cluster_extraction_;
  pcl::RadiusOutlierRemoval<PointT> radius_outlier_;

 public:
  /** \brief Constructor */
  SceneSegmentation();
  /** \brief Destructor */
  virtual ~SceneSegmentation();

  /** \brief Segment point cloud
   * \param[in] Point cloud
   * \param[out] A list of point cloud clusters
   * \param[out] A list of bounding boxes
   * \param[out] Model coefficients
   * \param[out] Workspace height
   * */
  PointCloud::Ptr segmentScene(const PointCloud::ConstPtr &cloud,
                               std::vector<PointCloud::Ptr> &clusters,
                               std::vector<BoundingBox> &boxes,
                               pcl::ModelCoefficients::Ptr &coefficients,
                               double &workspace_height);
  /** \brief Find plane
   * \param[in] Point cloud
   * \param[out] Convex hull
   * \param[out] Model coefficients
   * \param[out] Workspace height
   * */
  PointCloud::Ptr findPlane(const PointCloud::ConstPtr &cloud,
                            PointCloud::Ptr &hull, PointCloud::Ptr &plane,
                            pcl::ModelCoefficients::Ptr &coefficients,
                            double &workspace_height);

  /** \brief Set voxel grid parameters
   * \param[in] Leaf size for x,y,z
   * \param[in] Field name, on which axis the filter will be applied
   * \param[in] The minimum allowed the field value
   * \param[in] The maximum allowed the field value
   * */
  void setVoxelGridParams(double leaf_size, const std::string &field_name,
                          double limit_min, double limit_max);
  /** \brief Set passthrough filter parameters
   * \param[in] Enable or disable passthrough filter
   * \param[in] Field name, on which axis the filter will be applied
   * \param[in] The minimum allowed the field value
   * \param[in] The maximum allowed the field value
   * */
  void setPassthroughParams(bool enable_passthrough_filter,
                            const std::string &field_name, double limit_min,
                            double limit_max);
  /** \brief Set Normal param using radius
   * \param[in] Radius search
   * \param[in] Use Open MP (OMP) for parallel normal estimation using cpu
   * (default False)
   * \param[in] Number of cores to use for computing normal with OMP (default=4)
   * */
  void setNormalParams(double radius_search, bool use_omp = false,
                       int num_cores = 4);
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
  void setSACParams(int max_iterations, double distance_threshold,
                    bool optimize_coefficients, Eigen::Vector3f axis,
                    double eps_angle, double normal_distance_weight);
  /** \brief Set prism parameters
   * \param[in] The minimum height above the plane from which to construct the
   * polygonal prism
   * \param[in] The maximum height above the plane from which to construct the
   * polygonal prism
   * */
  void setPrismParams(double min_height, double max_height);
  /** \brief Set outliers parameters
   * \param[in] Radius of the sphere that will determine which points are
   * neighbors.
   * \param[in] The number of neighbors that need to be present in order to be
   * classified as an inlier.
   * */
  void setOutlierParams(double radius_search, int min_neighbors);
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
  void setClusterParams(double cluster_tolerance, int cluster_min_size,
                        int cluster_max_size, double cluster_min_height,
                        double cluster_max_height, double max_length,
                        double cluster_min_distance_to_polygon);

 private:
  bool enable_passthrough_filter_;
  bool use_omp_;
};

#endif  // MIR_OBJECT_SEGMENTATION_SCENE_SEGMENTATION_H
