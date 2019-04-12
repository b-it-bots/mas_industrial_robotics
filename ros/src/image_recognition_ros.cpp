/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <mir_object_recognition/image_recognition_ros.h>


ImageRecognitionROS::ImageRecognitionROS(ros::NodeHandle nh):nh_(nh)
{

}

ImageRecognitionROS::~ImageRecognitionROS()
{
    
}

void ImageRecognitionROS::recognizeImage(const sensor_msgs::ImageConstPtr &msg)
{

}
// void ImageRecognitionROS::extractObjectsInRGBImage(const cv::Mat &input_image,
//                         const cv::Mat &input_segmented_objects_image,
//                         cv::Mat &output_debug_image,
//                         std::vector<cv::Mat> &output_cropped_object_images,
//                         std::vector<float> &output_object_areas,
//                         std::vector<std::vector<cv::Point> > &output_roi_points,
//                         std::vector<cv::Point> &output_center_of_objects)
// {
//     /// Converting to match the intel camera point cloud size
//     // cv::Mat small_image = cv::Mat::zeros( cv::Size(640, 480), CV_8UC3 );
//     // resize(input_image, small_image, small_image.size(), 0, 0 );

//     input_image.copyTo(output_debug_image) ;

//     /// Find contours
//     cv::Mat clone_mask = input_segmented_objects_image.clone();
//     std::vector<std::vector<cv::Point> > contours;
//     std::vector<cv::Vec4i> hierarchy;
//     //Get only the external contour with hierachry level 1
//     findContours( clone_mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
//     std::vector<cv::Rect> roi_rectangle( contours.size() );
//     std::vector<cv::RotatedRect> rotated_roi( contours.size() );

//     for( int i = 0; i< contours.size(); i++ )
//     {
//         //Bounding box
//         roi_rectangle[i] = boundingRect( cv::Mat(contours[i]) );

//         // Find the minimum area enclosing bounding box
//         rotated_roi[i] = minAreaRect( cv::Mat(contours[i]) );

//         // expand rectangle a bit
//         // (move top left by 5x5 pixels, and increase size by 10 x 10)
//         roi_rectangle[i] -= cv::Point(15, 15);
//         roi_rectangle[i] += cv::Size(30, 30);
//         cv::Rect image_rect(0, 0, output_debug_image.cols, output_debug_image.rows);

//         // check if roi is contained within image
//         if (!((roi_rectangle[i] & image_rect) == roi_rectangle[i]))
//         {
//             if (roi_rectangle[i].x < 0)
//             {
//                 roi_rectangle[i].x = 0;
//             }
//             if (roi_rectangle[i].y < 0)
//             {
//                 roi_rectangle[i].y = 0;
//             }
//             if (roi_rectangle[i].x + roi_rectangle[i].width >= output_debug_image.cols)
//             {
//                 roi_rectangle[i].width = output_debug_image.cols - roi_rectangle[i].x - 1;
//             }
//             if (roi_rectangle[i].y + roi_rectangle[i].height >= output_debug_image.rows)
//             {
//                 roi_rectangle[i].height = output_debug_image.cols - roi_rectangle[i].y - 1;
//             }
//         }
 
//         // Drawing the ROI and the contours and the center of the contours
//         cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//         cv::rectangle(output_debug_image, roi_rectangle[i].tl(), roi_rectangle[i].br(), color, 1, 8, 0);
//         cv::drawContours( output_debug_image, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
//         cv::Point tl = roi_rectangle[i].tl();
//         int height = roi_rectangle[i].height;
//         int width = roi_rectangle[i].width;
//         cv::Point center = cv::Point(tl.x + width/2.0, tl.y + height/2.0);
//         cv::circle(output_debug_image, center, 2.0, color);
       
//         // Getting points on the bounding box 
//         std::vector<cv::Point>  bounding_box_points;
//         pointsOnRectangle( roi_rectangle[i], bounding_box_points );

//         //extracting the roi images 
//         cv::Rect rect_intersection = image_rect & roi_rectangle[i] ;
//         cv::Mat cropped_image = input_image(rect_intersection); 

//         ////////////////////////////////DEbug images start
//         // Extract the region
//         // std::stringstream convert; // stringstream used for the conversion
//         // unsigned long int sec= time(NULL);
//         // std::string filename;//string which will contain the result
//         // convert.str(""); //clearing the stringstream
//         // convert <<"/tmp/crop_" << i << "_" << sec <<".jpg";//add the value of Number to the characters in the stream
//         // filename = convert.str();//set Result to the content of the stream
//         // cv::imwrite(filename, cropped_image);
//         ////////////////////////////////DEbug images end


//         //creating output messages to be passed back
//         output_cropped_object_images.push_back( cropped_image );
//         output_object_areas.push_back( rotated_roi[i].size.height * rotated_roi[i].size.width );
//         output_roi_points.push_back( bounding_box_points );
//         output_center_of_objects.push_back( center );
//     }
//     ////////////////////////////////DEbug images start
//     //std::stringstream convert; // stringstream used for the conversion
//     //unsigned long int sec= time(NULL);
//     //std::string filename;//string which will contain the result
//     //convert.str(""); //clearing the stringstream
//     //convert <<"/tmp/debug_img_" << sec <<".jpg";//add the value of Number to the characters in the stream
//     //filename = convert.str();//set Result to the content of the stream
//     //cv::imwrite(filename, output_debug_image);
//     ////////////////////////////////DEbug images end
//     //return 0;
// }