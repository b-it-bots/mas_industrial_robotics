#include <min_distance_to_hull_calculator.hpp>

#include <math.h>
#include <limits>
#include <iostream>

typedef pcl::PointXYZRGBA PointRGBA;

MinDistanceToHullCalculator::MinDistanceToHullCalculator()
{
    convex_hull_available_ = false;
}

MinDistanceToHullCalculator::~MinDistanceToHullCalculator()
{
}

void MinDistanceToHullCalculator::setConvexHullPointsAndEdges(
        const pcl::PointCloud<PointRGBA>::ConstPtr& convex_hull_cloud)
{   
    hull_points_.clear();
    normalized_hull_edge_vecs_.clear();
    const PointRGBA* pcl_pt;
    Eigen::Vector3f hull_edge_pt_1, hull_edge_pt_2, edge_vec;
    pcl_pt = &convex_hull_cloud->points[0];
    hull_edge_pt_1 << pcl_pt->x, pcl_pt->y, pcl_pt->z;
    for( size_t i = 0; i < convex_hull_cloud->points.size(); i++){
        pcl_pt = &convex_hull_cloud->points[i];
        hull_edge_pt_2 << pcl_pt->x, pcl_pt->y, pcl_pt->z;
        hull_points_.push_back(hull_edge_pt_1);
        edge_vec = hull_edge_pt_2 - hull_edge_pt_1;
        normalized_hull_edge_vecs_.push_back(edge_vec/edge_vec.norm());
        hull_edge_pt_1 = hull_edge_pt_2;
    }
    if (!convex_hull_available_) convex_hull_available_ = true;
}

float MinDistanceToHullCalculator::computeMinDistanceToEdge(
        Eigen::Vector3f& pt, Eigen::Vector3f& edge_orig,
        Eigen::Vector3f& normalized_edge_vec)
{
    Eigen::Vector3f edge_orig_to_pt_vec = pt - edge_orig;
    return pow( ( pow(edge_orig_to_pt_vec.norm(), 2) -
                  pow(normalized_edge_vec.dot(edge_orig_to_pt_vec), 2)
                ),
               0.5);
}

float MinDistanceToHullCalculator::computeMinDistanceToHull(const PointRGBA& object_point)
{
    Eigen::Vector3f obj_pt;
    float pt_to_edge_dist, min_pt_to_edge_dist = std::numeric_limits<float>::max();
    obj_pt << object_point.x, object_point.y, object_point.z; 
    if (convex_hull_available_)
    {
        for( size_t i = 0; i < hull_points_.size(); i++)
        {
            pt_to_edge_dist = computeMinDistanceToEdge(obj_pt, hull_points_[i],
                                                       normalized_hull_edge_vecs_[i]);
            if (pt_to_edge_dist < min_pt_to_edge_dist) min_pt_to_edge_dist = pt_to_edge_dist;
        }
    }
    else 
    {
        std::cout << "!!! The convex hull points of the workspace are yet to be set !!!" << std::endl;
    }
    return min_pt_to_edge_dist;
}

float MinDistanceToHullCalculator::computeMinDistanceToHull(
        const pcl::PointCloud<PointRGBA>::ConstPtr& object_cloud)
{
    float min_pt_to_edge_dist, min_obj_to_hull_dist = std::numeric_limits<float>::max();
    if (convex_hull_available_)
    {
        for( size_t i = 0; i < object_cloud->points.size(); i++)
        {
            min_pt_to_edge_dist = computeMinDistanceToHull(object_cloud->points[i]);
            if (min_pt_to_edge_dist < min_obj_to_hull_dist) min_obj_to_hull_dist = min_pt_to_edge_dist;
        }
    }
    else 
    {
        std::cout << "!!! The convex hull points of the workspace are yet to be set !!!" << std::endl;
    }
    return min_obj_to_hull_dist;
}

