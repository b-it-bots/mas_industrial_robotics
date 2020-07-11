/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 */
#ifndef MIR_OBJECT_SEGMENTATION_LASERSCAN_SEGMENTATION_H
#define MIR_OBJECT_SEGMENTATION_LASERSCAN_SEGMENTATION_H

#include <geometry_msgs/Pose.h>
#include <mas_perception_msgs/LaserScanSegment.h>
#include <mas_perception_msgs/LaserScanSegmentList.h>
#include <sensor_msgs/LaserScan.h>

class LaserScanSegmentation
{
 public:
  /* dThresholdDistanceBetweenAdajecentPoints in meters */
  LaserScanSegmentation(double dThresholdDistanceBetweenAdajecentPoints,
                        unsigned int unMinimumPointsPerSegment);
  ~LaserScanSegmentation();

  mas_perception_msgs::LaserScanSegmentList getSegments(
      const sensor_msgs::LaserScan::ConstPtr &inputScan, bool store_data_points = false);

 private:
  /* distance threshold between two adjacent laser scan points to determine
   * where a new segment starts in meters */
  double _dThresholdDistanceBetweenAdajecentPoints;
  unsigned int _unMinimumPointsPerSegment;

  double getEuclideanDistance(double dDistanceA, double dAngleA, double dDistanceB, double dAngleB);

  geometry_msgs::Point getCenterOfGravity(unsigned int indexStart, unsigned int indexEnd,
                                          const sensor_msgs::LaserScan::ConstPtr &inputScan);
};

#endif  // MIR_OBJECT_SEGMENTATION_LASERSCAN_SEGMENTATION_H
