/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 */
#include <mir_object_segmentation/laserscan_segmentation.hpp>
#include <vector>

LaserScanSegmentation::LaserScanSegmentation(double dThresholdDistanceBetweenAdajecentPoints,
                                             unsigned int unMinimumPointsPerSegment)
{
  this->_dThresholdDistanceBetweenAdajecentPoints = dThresholdDistanceBetweenAdajecentPoints;
  this->_unMinimumPointsPerSegment = unMinimumPointsPerSegment;
}

LaserScanSegmentation::~LaserScanSegmentation() = default;

mas_perception_msgs::LaserScanSegmentList LaserScanSegmentation::getSegments(
    const sensor_msgs::LaserScan::ConstPtr &inputScan, bool store_data_points)
{
  mas_perception_msgs::LaserScanSegmentList segments;
  std::vector<geometry_msgs::Point> data_points;

  double dNumberofPointsBetweenStartAndEnd = 0;
  unsigned int unSegmentStartPoint = 0;
  unsigned int unSegmentEndPoint = 0;

  auto scan_size = static_cast<uint32_t>(
      ceil((inputScan->angle_max - inputScan->angle_min) / inputScan->angle_increment));

  if (scan_size == 0) return segments;

  // run over laser scan data
  for (unsigned int i = 0; i < (scan_size - 1); ++i) {
    ++dNumberofPointsBetweenStartAndEnd;

    // first point in laser scan is start point of the first segment
    if (i == 0) unSegmentStartPoint = i;

    double dAngleCur = inputScan->angle_min + (i * inputScan->angle_increment);
    double dDistanceCur = inputScan->ranges[i];
    double dAngleNext = inputScan->angle_min + ((i + 1) * inputScan->angle_increment);
    double dDistanceNext = inputScan->ranges[i + 1];

    if (store_data_points) {
      geometry_msgs::Point cur_point;
      cur_point.x = dDistanceCur * cos(dAngleCur);
      cur_point.y = dDistanceCur * sin(dAngleCur);
      data_points.push_back(cur_point);
    }

    if ((getEuclideanDistance(dDistanceCur, dAngleCur, dDistanceNext, dAngleNext) >
         this->_dThresholdDistanceBetweenAdajecentPoints) ||
        (i == (scan_size - 2))) {
      if (i < (scan_size - 2))
        unSegmentEndPoint = i;
      else
        unSegmentEndPoint = i + 1;

      // if number of points between start and end point is lesser then 3 , it
      // is not a segment
      if (dNumberofPointsBetweenStartAndEnd >= this->_unMinimumPointsPerSegment) {
        geometry_msgs::Point centerPoint;
        centerPoint = getCenterOfGravity(unSegmentStartPoint, unSegmentEndPoint, inputScan);
        double dDistanceToSegment = sqrt(pow(centerPoint.x, 2.0) + pow(centerPoint.y, 2.0));

        if (dDistanceToSegment < 5.0) {
          mas_perception_msgs::LaserScanSegment seg;

          seg.header = inputScan->header;
          seg.header.stamp = ros::Time::now();
          seg.center.x = centerPoint.x;
          seg.center.y = centerPoint.y;

          if (store_data_points) seg.data_points = data_points;

          segments.segments.push_back(seg);
        }
      }

      if (i < (scan_size - 2)) {
        unSegmentStartPoint = i + 1;
        dNumberofPointsBetweenStartAndEnd = 0;

        if (store_data_points) data_points.clear();
      }
    }
  }

  segments.header = inputScan->header;
  segments.header.stamp = ros::Time::now();
  segments.num_segments = static_cast<unsigned int>(segments.segments.size());

  return segments;
}

double LaserScanSegmentation::getEuclideanDistance(double dDistanceA, double dAngleA,
                                                   double dDistanceB, double dAngleB)
{
  return sqrt((dDistanceA * dDistanceA) + (dDistanceB * dDistanceB) -
              (2 * dDistanceA * dDistanceB) * cos(fabs(dAngleA - dAngleB)));
}

geometry_msgs::Point LaserScanSegmentation::getCenterOfGravity(
    unsigned int indexStart, unsigned int indexEnd,
    const sensor_msgs::LaserScan::ConstPtr &inputScan)
{
  geometry_msgs::Point centerPoint;

  centerPoint.x = 0;
  centerPoint.y = 0;
  centerPoint.z = 0;

  unsigned int i = 0, j = 0;
  for (i = indexStart, j = 0; i <= indexEnd; ++i, ++j) {
    double dAngle = inputScan->angle_min + (i * inputScan->angle_increment);
    double dX = inputScan->ranges[i] * cos(dAngle);
    double dY = inputScan->ranges[i] * sin(dAngle);

    centerPoint.x += dX;
    centerPoint.y += dY;
  }

  centerPoint.x /= j;
  centerPoint.y /= j;

  return centerPoint;
}
