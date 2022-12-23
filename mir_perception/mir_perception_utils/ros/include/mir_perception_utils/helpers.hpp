/*
 * Copyright 2020 Bonn-Rhein-Sieg University
 *
 * Author: Iswariya Manivannan, Mohammad Wasil
 * ROS2 contributor: Vamsi Kalagaturu.
 *
 */

#ifndef MIR_PERCCEPTION_UTILS_HELPERS_HPP
#define MIR_PERCCEPTION_UTILS_HELPERS_HPP

#include "mir_interfaces/msg/bounding_box.hpp"
#include "mir_interfaces/msg/planar_polygon.hpp"

#include "mir_perception_utils/aliases.hpp"
#include "mir_perception_utils/bounding_box.hpp"

using namespace mir_perception_utils::object;

/** Convert from PCL PlanarPolygon to ROS message. */
inline void convertPlanarPolygon(const PlanarPolygon &polygon,
                                 mas_perception_msgs::msg::PlanarPolygon &polygon_msg)
{
  for (int i = 0; i < 4; ++i)
  {
    polygon_msg.coefficients[i] = polygon.getCoefficients()[i];
  }

  const PointCloud::VectorType &points = polygon.getContour();
  for (size_t i = 0; i < points.size(); i++)
  {
    geometry_msgs::msg::Point32 pt;
    pt.x = points[i].x;
    pt.y = points[i].y;
    pt.z = points[i].z;
    polygon_msg.contour.push_back(pt);
  }
}

/** Convert from ROS message to PCL PlanarPolygon. */
inline void convertPlanarPolygon(const mas_perception_msgs::msg::PlanarPolygon &polygon_msg,
                                 PlanarPolygon &polygon)
{
  Eigen::Vector4f coefficients(polygon_msg.coefficients._M_elems);
  PointCloud::VectorType contour;
  for (size_t i = 0; i < polygon_msg.contour.size(); i++)
  {
    PointT pt;
    pt.x = polygon_msg.contour[i].x;
    pt.y = polygon_msg.contour[i].y;
    pt.z = polygon_msg.contour[i].z;
    contour.push_back(pt);
  }
  polygon = PlanarPolygon(contour, coefficients);
}

inline double computePlanarPolygonArea(const PlanarPolygon &polygon)
{
  const Eigen::Vector4f &normal = polygon.getCoefficients();
  const PointCloud::VectorType &points = polygon.getContour();
  // Find axis with largest normal component and project onto perpendicular
  // plane
  int k0, k1, k2;
  k0 = (std::fabs(normal[0]) > std::fabs(normal[1])) ? 0 : 1;
  k0 = (std::fabs(normal[k0]) > std::fabs(normal[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;

  double area = 0.0;
  for (size_t i = 0; i < points.size(); i++)
  {
    size_t j = (i + 1) % points.size();
    float p1[3] = {points[i].x, points[i].y, points[i].z};
    float p2[3] = {points[j].x, points[j].y, points[j].z};
    area += p1[k1] * p2[k2] - p1[k2] * p2[k1];
  }
  return std::fabs(area) / (2 * std::fabs(normal[k0]));
}

/** Convert from BoundingBox object to ROS message. */
inline void convertBoundingBox(const BoundingBox &bounding_box,
                               mas_perception_msgs::msg::BoundingBox &bounding_box_msg)
{
  const BoundingBox::Point &center = bounding_box.getCenter();
  bounding_box_msg.center.x = center[0];
  bounding_box_msg.center.y = center[1];
  bounding_box_msg.center.z = center[2];
  bounding_box_msg.dimensions.x = bounding_box.getDimensions()[0];
  bounding_box_msg.dimensions.y = bounding_box.getDimensions()[1];
  bounding_box_msg.dimensions.z = bounding_box.getDimensions()[2];

  const BoundingBox::Points &vertices = bounding_box.getVertices();
  for (size_t i = 0; i < vertices.size(); i++)
  {
    geometry_msgs::msg::Point pt;
    pt.x = vertices[i][0];
    pt.y = vertices[i][1];
    pt.z = vertices[i][2];
    bounding_box_msg.vertices.push_back(pt);
  }
}

#endif // MIR_PERCCEPTION_UTILS_HELPERS_HPP