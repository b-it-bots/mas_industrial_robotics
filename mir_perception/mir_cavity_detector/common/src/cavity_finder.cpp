#include <mir_cavity_detector/cavity_finder.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <math.h>
#include <sys/time.h>
#include <opencv2/imgproc/imgproc_c.h>

#define PI 3.14159265

#include <ros/ros.h>

CavityFinder::CavityFinder() : rng(12345),canny_threshold_(220), canny_multiplier_(3),
    binary_threshold_(62),approx_poly_epsilon_(0.017),approx_poly_epsilon_finer_(0.009)
{
}

CavityFinder::~CavityFinder()
{
}

std::vector<cv::Point>  CavityFinder::pointsOnRectangle(cv::Rect rect)
{
    std::vector<cv::Point>  rect_points;
    int num_of_points = 10;
    double delta_width = rect.size().width / num_of_points;
    double delta_height = rect.size().height / num_of_points;

    for (int i=0; i<num_of_points; i++)
    {
        //Top line
        rect_points.push_back(cv::Point (rect.tl().x + i*delta_width ,rect.tl().y));
    }
    for (int i=0; i<num_of_points; i++)
    {
        //right line
        rect_points.push_back(cv::Point (rect.br().x ,rect.br().y - i*delta_height));
    }
    for (int i=0; i<num_of_points; i++)
    {
        //Bottom line
        rect_points.push_back(cv::Point (rect.br().x - i*delta_width ,rect.br().y));
    }
    for (int i=0; i<num_of_points; i++)
    {
        // Left line
        rect_points.push_back(cv::Point (rect.tl().x ,rect.tl().y + i*delta_height));
    }

    return rect_points;
}



std::vector<std::string > CavityFinder::recognize2DCavities(const cv::Mat &image,
                                                            cv::Mat &debug_image,
                                                            std::vector<cv::Mat> &cropped_cavities,
                                                            const std::vector<cv::Point2f> &centroids)

{
{

    //Recognized name of the cavities
    std::vector<std::string> cavities_name(centroids.size());
    //ROI
    std::vector<cv::Rect> crop_box(centroids.size());

    /// Approximate cavities to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > final_cavities( centroids.size() );
    std::vector<std::vector<cv::Point> > cavities_poly( centroids.size() );
    std::vector<cv::Rect> boundRect( centroids.size() );
    std::vector<cv::Point2f> center( centroids.size() );
    std::vector<float>radius( centroids.size() );

    /// Find the rotated rectangles and ellipses for each cavity
    std::vector<cv::RotatedRect> minRect( centroids.size() );

    /// Converting to match the intel camera point cloud size
    cv::Mat small_image = cv::Mat::zeros( cv::Size(640, 480), CV_8UC3 );
    cv::Mat gray_image;
    resize(image, small_image, small_image.size(), 0, 0 );
    cv::cvtColor(small_image, gray_image, CV_BGR2GRAY);
    cv::blur(gray_image, gray_image, cv::Size(3, 3));
    small_image.copyTo(debug_image);

    cv::Size crop_size(120, 120);
    //After trials found that there is a shift of (20,10) between the rgbimage
    //and depth_rgb image
    //Shifting center to compensate this shift
    //Also shifting to the top left of the bounding box to be drawn
    //cv::Point2f shift(20.0f-(crop_size.width/2), 10.0f-(crop_size.height/2)); 
    cv::Point2f shift(-(crop_size.width/2), -(crop_size.height/2)); 
    

    cv::Rect img_rect = cv::Rect(cv::Point(0,0), gray_image.size());
    cv::Mat contour_image = cv::Mat::zeros( cv::Size(640, 480), CV_8UC3 );
    // Finding the cavities in each region
    for( int i = 0; i< centroids.size(); i++ )
    {
        //Finding the ROI
        crop_box[i] = cv::Rect(centroids[i]+shift, crop_size);
        cv::Mat threshold_output;
        /// Detect edges using Threshold OTSU thereshold
        //cv::threshold( gray_image, threshold_output, binary_threshold_, 255, cv::THRESH_BINARY );
        cv::threshold( gray_image, threshold_output, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU );
        // finding crop image
        cv::Rect rect_intersection = img_rect & crop_box[i];
        cv::Mat cropped_image = threshold_output(rect_intersection);

        std::vector<std::vector<cv::Point> > cavities_in_crop;
        std::vector<cv::Vec4i> hierarchy;

        //Subtracting from white, to avoid boundary detection
        cropped_image = 255 - cropped_image;
        /// Find contours
        // cv::findContours( contour_image, cavities_in_crop, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        
        
        cv::findContours( cropped_image, cavities_in_crop, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        // cv::imshow("cavities cropped ", cropped_image);
        // cv::waitKey(0);
        //Finding the maximum cavity size in the corped image and storing it
        double max_area = 0.0; int max_index = 0;
        if (cavities_in_crop.size() == 0)
        {
            //As the inf in depth_rgb image is converted to black
            //many boundary cavities are detected filtering those out
            continue;
        }
        if (cavities_in_crop.size() > 1)
        {
            for( int j = 0; j < cavities_in_crop.size(); j++ )
            {
                if(std::fabs(cv::contourArea(cavities_in_crop[j]))  > max_area)
                {
                    max_area = std::fabs(cv::contourArea(cavities_in_crop[j]));
                    max_index = j;
                }
            }
            final_cavities[i] = cavities_in_crop[max_index];
        } else
        {
            final_cavities[i] = cavities_in_crop[0];
        }

        cv::approxPolyDP( cv::Mat(final_cavities[i]), cavities_poly[i], 
                cv::arcLength(cv::Mat(final_cavities[i]), true)*approx_poly_epsilon_ , true );

        boundRect[i] = cv::boundingRect( cv::Mat(cavities_poly[i]) );
        minEnclosingCircle( (cv::Mat)cavities_poly[i], center[i], radius[i] );
        minRect[i] = cv::minAreaRect( cv::Mat(cavities_poly[i]) );
    }

    
    std::cout<<"[cavity_recognizer] Centroids received : "<<(int)centroids.size() <<std::endl;

    //Recognition
    //All Magic numbers in here are from J48 Weka library
    //The features were learned from the images generated from the cpp file
    for( int i = 0; i< centroids.size(); i++ )
    {
        if (final_cavities[i].size() == 0)
        {
            //Filtering corner cavities for which no cavity is detected 
            //in normal rgb image
            std::cout<<"No contours "<<std::endl;
            continue;
        }
        std::vector<cv::Point> finer_polynomial;
        cv::approxPolyDP( cv::Mat(final_cavities[i]),finer_polynomial ,
                cv::arcLength(cv::Mat(final_cavities[i]), true)*approx_poly_epsilon_finer_ , true );
        double area = std::fabs(cv::contourArea(final_cavities[i]));


        cv::RotatedRect minRect;
        minRect = cv::minAreaRect(cv::Mat(final_cavities[i]));
        double aspect_ratio;
        if(minRect.size.width > minRect.size.height)
        {
            aspect_ratio = minRect.size.width / minRect.size.height;
        }
        else
        {
            aspect_ratio = minRect.size.height / minRect.size.width;
        }

        //Hu moments
        cv::Moments mom = cv::moments(final_cavities[i]);
        double hu[7];
        cv::HuMoments(mom, hu);

	/*
	radius <= 32.8673
	|   aspect_ratio <= 1.125
	|   |   radius <= 23.4876: M20 (13.0)
	|   |   radius > 23.4876: M30 (10.0)
	|   aspect_ratio > 1.125: R20 (18.0)
	radius > 32.8673
	|   aspect_ratio <= 2.27122: S40_40 (20.0)
	|   aspect_ratio > 2.27122
	|   |   radius <= 55.0689: F20_20 (13.0)
	|   |   radius > 55.0689: M20_100 (10.0)
	21 < radius < 62
        1 < aspect_ratio < 4
        
	*/

	if ( (10 < radius[i]) and (radius[i] <= 32.8673)) {
	    if ((0 < aspect_ratio) and (aspect_ratio <= 1.3)) {
            if (radius[i] <= 18.4876) {
		        cavities_name[i] = "M20";
		    } else {
		        cavities_name[i] = "M30";
		    }
        } else if ((4 > aspect_ratio) and (aspect_ratio > 1.125)) {
		    cavities_name[i] = "R20";
	    } else {
	        cavities_name[i] = "DECOY_aspect";
	    }
	} else if((62 > radius[i]) and (radius[i] > 32.8673)) {
	    if ((0.0 < aspect_ratio) and (aspect_ratio <= 2.7122)) {
		     cavities_name[i] = "S40_40";
	    } else if ((5 > aspect_ratio) and (aspect_ratio > 2.7122)) {
            if (radius[i] <= 47.5) {
		        cavities_name[i] = "F20_20";
		    } else {
		        cavities_name[i] = "M20_100";
		    }
	    } else {
	        cavities_name[i] = "DECOY_2";
	   }
	    
	} else {
	        cavities_name[i] = "DECOY_radius";
	   }

        // std::cout<<" aspect_ratio: "<<aspect_ratio<<" radius: "<<radius[i];
        // std::cout<<" NAME: "<<cavities_name[i]<<std::endl;
    }


    //Display debug image
    for( int i = 0; i< centroids.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::circle( debug_image, centroids[i]+shift, 10, color, 2, 8, 0 );
        cv::rectangle( debug_image, crop_box[i].tl(), crop_box[i].br(), color, 1, 8, 0 );
        std::cout<<"Drawing contours debug image"<<std::endl;
        /// Draw the cavities
        cv::drawContours( debug_image, final_cavities, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, centroids[i]+shift );

        std::string label;//string which will contain the result
        std::stringstream convert; // stringstream used for the conversion
        convert <<"V: "<< cavities_poly[i].size();//add the value of Number to the characters in the stream
        convert <<cavities_name[i];
        label = convert.str();//set Result to the content of the stream

        cv::putText( debug_image, label, centroids[i]+shift, 0, 0.8, color );

        ////////////////////////////////DEbug images start
        cv::Rect rect_intersection = img_rect & crop_box[i];
        cv::Mat rgb_crop_image = small_image(rect_intersection);
        unsigned long int sec= time(NULL);
        std::string filename;//string which will contain the result
        convert.str(""); //clearing the stringstream
        convert <<"/tmp/crop_img_"<< cavities_name[i] << sec <<".jpg";//add the value of Number to the characters in the stream
        filename = convert.str();//set Result to the content of the stream

        cropped_cavities.push_back(rgb_crop_image);
        // cv_image = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::BGR8);
        // image_list.images[i] = img_msg;
        cv::imwrite(filename, rgb_crop_image);
        ////////////////////////////////DEbug images end
    }

    return cavities_name;
}




}

std::vector<std::vector<cv::Point> > CavityFinder::find2DCavities(const cv::Mat &image, 
                                                                   cv::Mat &debug_image,
                                                                   std::vector<cv::Point2f> &centroids)
{
    /// Converting to match the intel camera point cloud size
    cv::Mat small_image = cv::Mat::zeros( cv::Size(640, 480), CV_8UC3 );
    cv::Mat gray_image;
    resize(image, small_image, small_image.size(), 0, 0 );

    //Croping image to select ROIv
    cv:: Rect roi_without_arm;
    roi_without_arm.x = 0;
    roi_without_arm.y = 0;
    roi_without_arm.width = 640;
    roi_without_arm.height = 400;
    small_image = small_image(roi_without_arm);
    cv::cvtColor(small_image, gray_image, CV_BGR2GRAY);

    cv::blur(gray_image, gray_image, cv::Size(3, 3));

        ////////////////////////////////DEbug images start
        unsigned long int sec= time(NULL);
        std::string filename;//string which will contain the result
        std::stringstream convert; // stringstream used for the conversion
        convert <<"/tmp/depth_rgb_"<< sec<<".jpg";//add the value of Number to the characters in the stream
        filename = convert.str();//set Result to the content of the stream
        cv::imwrite(filename, small_image);
        ////////////////////////////////DEbug images end
    cv::Mat canny_output;
    cv::Mat threshold_output;

    std::vector<std::vector<cv::Point> > cavities;
    std::vector<std::vector<cv::Point> > filtered_cavities;

    std::vector<cv::Vec4i> hierarchy;

    /// Detect edges using Threshold
    cv::threshold( gray_image, threshold_output, binary_threshold_, 255, cv::THRESH_BINARY );

    // Dilate edges so that the cavities are expanded slightly
    int dilation_size = 10;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                      cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                      cv::Point(dilation_size, dilation_size));

    cv::erode(threshold_output, threshold_output, element);
    cv::dilate(threshold_output, threshold_output, element);

    /// Find contours
    cv::findContours( threshold_output, cavities, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );


    /*
    // Canny edge detection
    cv::Canny(gray_image, canny_output, canny_threshold_, canny_threshold_ * canny_multiplier_, 3);
    canny_output.copyTo(debug_image);

    */

    //Filtering cavities based on area
    //This works for detecting cavity since our cavities lie in specific range
    for (int i = 0; i < cavities.size(); i++)
    {
        double area = std::fabs(cv::contourArea(cavities[i]));
        //Filter on the area of the cavity from data collection
        if (area < min_area_depth_image_ || area > max_area_depth_image_)
            continue;

        std::vector<cv::Point> approx_cavity;
        cv::approxPolyDP( cv::Mat(cavities[i]), approx_cavity, 
                cv::arcLength(cv::Mat(cavities[i]), true)*approx_poly_epsilon_ , true );
        //Minimum cavities with 4 sides to be considered
        if (approx_cavity.size() < 4)
            continue;

        filtered_cavities.push_back(cavities[i]);
    }


    /// Approximate cavities to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > cavities_poly( filtered_cavities.size() );
    std::vector<cv::Rect> boundRect( filtered_cavities.size() );
    std::vector<cv::Point2f> center( filtered_cavities.size() );
    std::vector<float>radius( filtered_cavities.size() );

    // filter the cavities so we keep only those in the bottom half of the image
    for (int i = 0; i < filtered_cavities.size(); i++)
    {
        cv::approxPolyDP( cv::Mat(filtered_cavities[i]), cavities_poly[i], 
                cv::arcLength(cv::Mat(filtered_cavities[i]), true)*approx_poly_epsilon_ , true );

        boundRect[i] = cv::boundingRect( cv::Mat(cavities_poly[i]) );
        //Increasing teh Bounded rectangel size by 30%
        cv::Size deltaSize( boundRect[i].width * 0.3f, boundRect[i].height * 0.3f ); // 0.1f = 10/100
        cv::Point offset( deltaSize.width/2, deltaSize.height/2);
        boundRect[i] += deltaSize;
        boundRect[i] -= offset; 

        //Finding minimum enclosing circle
        minEnclosingCircle( (cv::Mat)cavities_poly[i], center[i], radius[i] );
        //Returnging the center of the cavities
        centroids.push_back(center[i]);
    }




    //Converting bounding rectto points
    std::vector<std::vector<cv::Point> > bounding_rect_points( filtered_cavities.size() );

    /// Draw polygonal cavity + bonding rects + circles
    small_image.copyTo(debug_image);
    for( int i = 0; i< filtered_cavities.size(); i++ )
    {
        bounding_rect_points[i] = pointsOnRectangle(boundRect[i]);
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::rectangle( debug_image, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        cv::drawContours( debug_image, bounding_rect_points, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
    }

    return bounding_rect_points;
}

std::vector<pcl::PCLPointCloud2::Ptr> CavityFinder::get3DCavities(const std::vector<std::vector<cv::Point> > &cavities, pcl::PCLPointCloud2::Ptr input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*input_cloud, *xyz_input_cloud);

    // loop through points in the 2D cavity and find their 3D positions in the given pointcloud
    std::vector<pcl::PCLPointCloud2::Ptr> pcl_cavities;

    for (size_t i = 0; i < cavities.size(); i++)
    {
        pcl::PCLPointCloud2::Ptr pcl_cavity(new pcl::PCLPointCloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cavity(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t j = 0; j < cavities[i].size(); j++)
        {
            pcl::PointXYZ pcl_point;
            try {
                pcl_point = xyz_input_cloud->at(cavities[i][j].x, cavities[i][j].y);
            } catch (...) {
                std::cout << "is empty"; 
                std::cout << xyz_input_cloud->points.empty() << std::endl;
                std::cout << "x : " << cavities[i][j].x << "y : " << cavities[i][j].y << std::endl;
                std::cout << "#########################" << std::endl; 
                std::cout << "ERROR :i : " << i << "j : " << j << std::endl;
                std::cout << "#########################" << std::endl; 
            }
            if ((!std::isnan(pcl_point.x)) && (!std::isnan(pcl_point.y)) && (!std::isnan(pcl_point.z) && (pcl_point.z >0.01)))
            {
                xyz_cavity->points.push_back(pcl_point);
            }
        
        }

        // remove outliers in the pointcloud. this ensures the points are roughly on the same plane
        xyz_cavity->header = xyz_input_cloud->header;

        if (xyz_cavity->points.size() > 0)
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(xyz_cavity);
            sor.setMeanK(50);
            sor.setStddevMulThresh(3.0);
            sor.filter(*xyz_cavity);
        }
        pcl::toPCLPointCloud2(*xyz_cavity, *pcl_cavity);
        pcl_cavities.push_back(pcl_cavity);
    }

    return pcl_cavities;
}

void CavityFinder::setCannyThreshold(double canny_threshold)
{
    canny_threshold_ = canny_threshold;
}

void CavityFinder::setCannyMultiplier(double canny_multiplier)
{
    canny_multiplier_ = canny_multiplier;
}

void CavityFinder::setBinaryThreshold(double binary_threshold)
{
    binary_threshold_ = binary_threshold;
}


void CavityFinder::setEpsilonApproxPoly(double epsilon_approx_poly)
{
    approx_poly_epsilon_ = epsilon_approx_poly;
}

void CavityFinder::setEpsilonFinerPoly(double epsilon_finer_poly)
{
    approx_poly_epsilon_finer_ = epsilon_finer_poly;
}

void CavityFinder::setMinArea(double min_area_depth_image)
{
    min_area_depth_image_ = min_area_depth_image;
}

void CavityFinder::setMaxArea(double max_area_depth_image)
{
    max_area_depth_image_ = max_area_depth_image;
}
