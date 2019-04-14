//get3DBoundingBox
//get2DBoundingBox
//get3DPose
//#include "mcr_scene_segmentation/clustered_point_cloud_visualizer.h"
// #include "mcr_scene_segmentation/bounding_box_visualizer.h"
// #include <mcr_scene_segmentation/label_visualizer.h>

// using mcr::visualization::BoundingBoxVisualizer;
// using mcr::visualization::ClusteredPointCloudVisualizer;
// using mcr::visualization::LabelVisualizer;
// using mcr::visualization::Color;


// Intel ros object analytics
// BBox merger
// double MultimodalObjectRecognitionROS::getMatch(const cv::Rect2d& r1, const cv::Rect2d& r2)
// {
//   cv::Rect2i ir1(r1), ir2(r2);
//   /* calculate center of rectangle #1*/
//   cv::Point2i c1(ir1.x + (ir1.width >> 1), ir1.y + (ir1.height >> 1));
//   /* calculate center of rectangle #2*/
//   cv::Point2i c2(ir2.x + (ir2.width >> 1), ir2.y + (ir2.height >> 1));

//   double a1 = ir1.area(), a2 = ir2.area(), a0 = (ir1 & ir2).area();
//   /* calculate the overlap rate*/
//   double overlap = a0 / (a1 + a2 - a0);
//   /* calculate the deviation between centers #1 and #2*/
//   double deviate = sqrt(powf((c1.x - c2.x), 2) + powf((c1.y - c2.y), 2));
//   /* calculate the length of diagonal for the rectangle in average size*/
//   double len_diag = sqrt(powf(((ir1.width + ir2.width) >> 1), 2) + powf(((ir1.height + ir2.height) >> 1), 2));

//   /* calculate the match rate. The more overlap, the more matching. Contrary, the more deviation, the less matching*/
//   return overlap * len_diag / deviate;
// }
