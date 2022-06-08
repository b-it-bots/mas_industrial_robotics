#ifndef PLANAR_POLYGON_VISUALIZER_HPP
#define PLANAR_POLYGON_VISUALIZER_HPP

namespace mir_perception_utils
{
namespace visualization
{
PlanarPolygonVisualizer::PlanarPolygonVisualizer(const std::string &topic_name, Color color,
                                                 bool check_subscribers, double thickness)
    : color_(color), check_subscribers_(check_subscribers), thickness_(thickness)
{
  ros::NodeHandle nh("~");
  marker_publisher_ = nh.advertise<visualization_msgs::Marker>(topic_name, 1);
}

template <typename PointT>
void PlanarPolygonVisualizer::publish(const pcl::PlanarPolygon<PointT> &polygon,
                                      const std::string &frame_id)
{
  if (marker_publisher_.getNumSubscribers() == 0) return;
  visualization_msgs::Marker marker;
  buildPolygonMarker<PointT>(polygon.getContour(), marker, frame_id);
  marker_publisher_.publish(marker);
}

template <typename PointT>
void PlanarPolygonVisualizer::buildPolygonMarker(
    const typename pcl::PointCloud<PointT>::VectorType &points, visualization_msgs::Marker &marker,
    const std::string &frame_id, int id)
{
  if (!points.size()) return;

  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = frame_id;
  marker.scale.x = thickness_;
  marker.scale.y = thickness_;
  marker.color.a = 1.0;
  marker.ns = "polygon";
  marker.id = id;
  marker.color = color_;

  geometry_msgs::msg::Point first_point;
  first_point.x = points[0].x;
  first_point.y = points[0].y;
  first_point.z = points[0].z;
  marker.points.push_back(first_point);

  for (size_t i = 1; i < points.size(); i++) {
    geometry_msgs::msg::Point pt;
    pt.x = points[i].x;
    pt.y = points[i].y;
    pt.z = points[i].z;
    marker.points.push_back(pt);
    marker.points.push_back(pt);
  }

  marker.points.push_back(first_point);
}
}
}

#endif /* PLANAR_POLYGON_VISUALIZER_HPP */
