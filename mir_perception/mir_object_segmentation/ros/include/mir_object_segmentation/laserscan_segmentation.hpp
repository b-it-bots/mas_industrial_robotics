/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 * ROS2 contributors: Hamsa Datta Perur.
 */

#ifndef MIR_OBJECT_SEGMENTATION_LASERSCAN_SEGMENTATION_HPP
#define MIR_OBJECT_SEGMENTATION_LASERSCAN_SEGMENTATION_HPP

#include <cmath>
#include <rclcpp/clock.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <mir_interfaces/msg/laser_scan_segment.hpp>
#include <mir_interfaces/msg/laser_scan_segment_list.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanSegmentation
{
 public:
  /* dThresholdDistanceBetweenAdajecentPoints in meters */
  LaserScanSegmentation(double dThresholdDistanceBetweenAdajecentPoints,
                        unsigned int unMinimumPointsPerSegment);
  ~LaserScanSegmentation();

  mas_perception_msgs::msg::LaserScanSegmentList getSegments(
      const std::shared_ptr<const sensor_msgs::msg::LaserScan> &inputScan, bool store_data_points = false);

 private:
  /* distance threshold between two adjacent laser scan points to determine
   * where a new segment starts in meters */
  double _dThresholdDistanceBetweenAdajecentPoints;
  unsigned int _unMinimumPointsPerSegment;

  double getEuclideanDistance(double dDistanceA, double dAngleA, double dDistanceB, double dAngleB);

  geometry_msgs::msg::Point getCenterOfGravity(unsigned int indexStart, unsigned int indexEnd,
                                          const std::shared_ptr<const sensor_msgs::msg::LaserScan> &inputScan);
};

#endif  // MIR_OBJECT_SEGMENTATION_LASERSCAN_SEGMENTATION_HPP
