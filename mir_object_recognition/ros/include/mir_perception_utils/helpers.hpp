#ifndef MIR_PERCCEPTION_UTILS_HELPERS_HPP
#define MIR_PERCCEPTION_UTILS_HELPERS_HPP

#include "mas_perception_msgs/msg/bounding_box.hpp"
#include "mas_perception_msgs/msg/planar_polygon.hpp"

#include "mir_perception_utils/aliases.hpp"
#include "mir_perception_utils/bounding_box.hpp"

using namespace mir_perception_utils::object;

/** Convert from PCL PlanarPolygon to ROS message. */
void convertPlanarPolygon(const PlanarPolygon &polygon,
                                 mas_perception_msgs::msg::PlanarPolygon &polygon_msg);

/** Convert from ROS message to PCL PlanarPolygon. */
void convertPlanarPolygon(const mas_perception_msgs::msg::PlanarPolygon &polygon_msg,
                                 PlanarPolygon &polygon);

double computePlanarPolygonArea(const PlanarPolygon &polygon);

/** Convert from BoundingBox object to ROS message. */
void convertBoundingBox(const BoundingBox &bounding_box,
                               mas_perception_msgs::msg::BoundingBox &bounding_box_msg);

#endif // MIR_PERCCEPTION_UTILS_HELPERS_HPP