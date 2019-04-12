// #include <ros/ros.h>
// #include <mir_object_recognition/multimodal_object_recognition_ros.h>
// #include <mcr_scene_segmentation/clustered_point_cloud_visualizer.h>
// #include <mcr_scene_segmentation/bounding_box_visualizer.h>
// #include <mcr_scene_segmentation/label_visualizer.h>

// // using mcr::visualization::BoundingBoxVisualizer;
// // using mcr::visualization::ClusteredPointCloudVisualizer;
// // using mcr::visualization::LabelVisualizer;
// // using mcr::visualization::Color;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "multimodal_object_recognition");
//     ros::NodeHandle nh;

//     // using mcr::visualization::BoundingBoxVisualizer;
//     // using mcr::visualization::ClusteredPointCloudVisualizer;
//     // using mcr::visualization::LabelVisualizer;
//     // using mcr::visualization::Color;
//     mcr::visualization::BoundingBoxVisualizer bounding_box_visualizer(nh, "bounding_boxes", mcr::visualization::Color(mcr::visualization::Color::SEA_GREEN));
//     mcr::visualization::ClusteredPointCloudVisualizer cluster_visualizer("tabletop_clusters");
//     mcr::visualization::LabelVisualizer label_visualizer("labels", mcr::visualization::Color(mcr::visualization::Color::TEAL));

//     int frame_rate = 30;
//     nh.param<int>("frame_rate", frame_rate, 30);
//     ROS_INFO_STREAM("[multimodal_object_recognition] node started");

//     MultimodalObjectRecognitionROS object_recognition(nh, bounding_box_visualizer, cluster_visualizer, label_visualizer);
//     //boost::shared_ptr<MultimodalObjectRecognitionROS> object_recognition = boost::make_shared<MultimodalObjectRecognitionROS>(nh);

//     ros::Rate loop_rate(frame_rate);
//     while (ros::ok())
//     {
//         object_recognition.update();
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }