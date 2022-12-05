#include<iostream>
#include<fstream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<dirent.h>
#include<string.h>

using namespace std;
using namespace cv;

RNG rng(12345);
void extract_features(string image_path, ofstream &output, string image_name);
int main()
{
    string dirName = "/home/deebuls/ros/indigo/src/unmerged_packages_for_testing/perception/mir_cavity_detector/tools/dataset_new/";
    DIR *dir;
    dir = opendir(dirName.c_str());
    string obj_name;
    string image_name;
    struct dirent *ent;
    ofstream OutputFileName ;
    OutputFileName.open("cavityFeatures_m20_100.csv");         //Opening file to print info to
    OutputFileName << "Object_name,Contour1,contour2,contour3,radius,area,aspect_ratio,isConvex,hu0,hu1,hu2,hu3" << endl;          //Headings for file
    if (dir != NULL) {
    while ((ent = readdir (dir)) != NULL) {
        obj_name= ent->d_name;
        //I found some . and .. files here so I reject them.
        if(obj_name.compare(".")!= 0 && obj_name.compare("..")!= 0)
        {
            struct dirent *image_ptr;
            string object_path;
            object_path.append(dirName);
            object_path.append(obj_name);
            DIR *obj_dir;
            obj_dir = opendir(object_path.c_str());
            while ((image_ptr = readdir (obj_dir)) != NULL) {

                image_name= image_ptr->d_name;
                if(image_name.compare(".")!= 0 && image_name.compare("..")!= 0)
                {
                    string image_path;
                    image_path.append(dirName);
                    image_path.append(obj_name);
                    image_path.append("/");
                    image_path.append(image_name);
                    OutputFileName << obj_name <<",";
                    extract_features(image_path, OutputFileName, image_name);
                    //Mat image= imread(image_path);
                    //imshow(obj_name,image);
                    //waitKey(0);
                }
            }
            closedir (obj_dir);
        }
    }
    closedir (dir);
    OutputFileName.close();
} else {
    cout<<"not present"<<endl;
    }
}

void extract_features(string image_path, ofstream &output, string image_name)
{

    cout << image_path << endl;
    int thresh = 52;
    double epsilon_1 = 0.03;
    double epsilon_2 = 0.007;
    double epsilon_3 = 0.037;
    int morph_elem = 0;
    int morph_size = 0;
    int morph_operator = 0;
    int operation = morph_operator + 2;
    /// Global variables
    Mat src, dst, src_gray;
    /// Load source image and convert it to gray
    src = imread( image_path , CV_LOAD_IMAGE_COLOR);
    if  (! src.data)
    {
        cout<<"Read Failed" << endl;
        return;
    }
/// Convert image to gray and blur it
    cvtColor( src, src_gray, cv::COLOR_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );

    //Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

    /// Apply the specified morphology operation
    //morphologyEx( src_gray, dst, operation, element );

    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<vector<Point> > raw_contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    threshold( src_gray, threshold_output, 0, 255, THRESH_BINARY | THRESH_OTSU );

    // you can change parameters to your result 
    //erode(threshold_output,threshold_output, Mat(), Point(-1,-1),5);
    // you can change parameters to your result 
    //erode(threshold_output,threshold_output, Mat(), Point(-1,-1),5);

    threshold_output = 255 - threshold_output; //Subtracting from white, so that contour detection doesnot selects the boundary
    if  (! threshold_output.data)
    {
        cout<<"Read Failed" << endl;
        return;
    }
    /// Find contours
    findContours( threshold_output, raw_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    double max_area = 0.0; int max_index = 0;
    if (raw_contours.size() > 1)
    {
        for( int i = 0; i < raw_contours.size(); i++ )
        {
            if(std::fabs(cv::contourArea(raw_contours[i]))  > max_area)
            {
               max_area = std::fabs(cv::contourArea(raw_contours[i]));
               max_index = i;
            }
        }
        contours.push_back(raw_contours[max_index]);
    } else
    {
        contours.push_back(raw_contours[0]);
    }

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<vector<Point> > contours_poly_2( contours.size() );
    vector<vector<Point> > contours_poly_3( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );
    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    {
        cout<<"Epsilon : "<<epsilon_1<<" arc length :"<<cv::arcLength(Mat(contours[i]), true)*epsilon_1<<endl;
        approxPolyDP( Mat(contours[i]), contours_poly[i], cv::arcLength(Mat(contours[i]), true)*epsilon_1, true );
        output << contours_poly[i].size() << ",";
        approxPolyDP( Mat(contours[i]), contours_poly_2[i], cv::arcLength(Mat(contours[i]), true)*epsilon_2, true );
        output << contours_poly_2[i].size() << ",";
        approxPolyDP( Mat(contours[i]), contours_poly_3[i], cv::arcLength(Mat(contours[i]), true)*epsilon_3, true );
        output << contours_poly_3[i].size() << ",";

        boundRect[i] = boundingRect( Mat(contours_poly[i]) );

        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        output << radius[i] << "," ;
        output << std::fabs(cv::contourArea(contours[i])) << "," ;
        minRect[i] = minAreaRect( Mat(contours[i])) ;
        double aspect_ratio;
        if (minRect[i].size.width > minRect[i].size.height)
        {
            aspect_ratio = minRect[i].size.width / minRect[i].size.height;
        }
        else
        {
            aspect_ratio = minRect[i].size.height / minRect[i].size.width;
        }
        output << aspect_ratio << ",";
        output << (int)cv::isContourConvex(contours_poly[i]) << ",";
	cv::Moments mom = cv::moments(contours[0]);
	double hu[7];
	cv::HuMoments(mom, hu);
	output << hu[0] << "," <<hu[1] << "," << hu[2] << "," << hu[3] ;
    }
    output << endl;


  Mat drawing ;
  src.copyTo(drawing) ;
  for( int i = 0; i< contours.size(); i++ )
     {
        // number of vertices of polygonal curve
        int vtc = contours_poly[i].size();
        double aspect_ratio;
        if (minRect[i].size.width > minRect[i].size.height)
            aspect_ratio = minRect[i].size.width / minRect[i].size.height;
        else
            aspect_ratio = minRect[i].size.height / minRect[i].size.width;
        string Result;//string which will contain the result
        stringstream convert; // stringstream used for the conversion
        convert <<"V:";
        convert << vtc ;//add the value of Number to the characters in the stream
        convert << ",A:" ;
        convert << std::fabs(cv::contourArea(contours[i]));
        convert << ",R";
        convert << aspect_ratio;
        Result = convert.str();//set Result to the content of the stream
        if (4 == contours_poly[i].size())
            cout<<"Aspect Ratio : "<<aspect_ratio<<endl;
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours_poly, i, color, 0, 8, vector<Vec4i>(), 0, Point() );
        // rotated rectangle
       Point2f rect_points[4]; minRect[i].points( rect_points );
       for( int j = 0; j < 4; j++ )
          line( drawing, rect_points[j], rect_points[(j+1)%4], color, 2, 8 );

       //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
       // ellipse
       //ellipse( drawing, minEllipse[i], color, 1, 8 );
       //if (std::fabs(cv::contourArea(contours[i])) < 1000 
       //        || std::fabs(cv::contourArea(contours[i])) > 50000)
       //     continue;
       putText( drawing, Result, center[i], 2, 0.5, color );
       putText( drawing, image_name, Point(10,10), 1, 0.5, color );
     }

  /// Show in a window
  /// Create Window
  namedWindow( "Contours", CV_WINDOW_NORMAL );
  resizeWindow( "Contours", 800, 600 );
  imshow( "Contours", drawing );
  waitKey(0);
     return;
}
