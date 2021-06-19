#ifndef MIN_DISTANCE_TO_HULL_CALCULATOR_H_
#define MIN_DISTANCE_TO_HULL_CALCULATOR_H_

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointRGBA;

class MinDistanceToHullCalculator
{
public:
    /**
     * Constructor
     */
    MinDistanceToHullCalculator();
    /**
     * Destructor
     */
    virtual ~MinDistanceToHullCalculator();   

    /**
     * Set the convex hull points of the workspace
     */
    void setConvexHullPointsAndEdges(const pcl::PointCloud<PointRGBA>::ConstPtr& convex_hull_cloud);
    
    /**
     * Calculate minimum euclidean distance between an object point and hull points 
     */
    float computeMinDistanceToHull(const PointRGBA& object_point);

    /**
     * Calculate minimum euclidean distance between object cloud points and hull points
     */
    float computeMinDistanceToHull(const pcl::PointCloud<PointRGBA>::ConstPtr& object_cloud);
    
private:
    /**
     * Flag to check if convex hull points of a workspace are available
     */
    bool convex_hull_available_;   
 
    /**
     * Convex hull point and edge vectors
     */
    std::vector<Eigen::Vector3f> hull_points_;    
    std::vector<Eigen::Vector3f> normalized_hull_edge_vecs_;    

    /**
     * Calculate minimum euclidean distance between a point and an edge
     */
    float computeMinDistanceToEdge(Eigen::Vector3f& pt, Eigen::Vector3f& edge_orig, Eigen::Vector3f& normalized_edge_vec);
};
#endif
