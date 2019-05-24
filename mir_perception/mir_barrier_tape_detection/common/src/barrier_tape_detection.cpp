#include <mir_barrier_tape_detection/barrier_tape_detection.h>

BarrierTapeDetection::BarrierTapeDetection()
{
}

BarrierTapeDetection::~BarrierTapeDetection()
{
}

void BarrierTapeDetection::preprocessImage(const cv::Mat &input_img, cv::Mat &output_img)
{
    cv::Mat hsv_img;
    cv::Mat filtered_img;

    cv::cvtColor(input_img, hsv_img, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_img, color_thresh_min_, color_thresh_max_, filtered_img);
    cv::GaussianBlur(filtered_img, output_img, cv::Size(7, 7), 0, 0 );
}

bool BarrierTapeDetection::detectBarrierTape(const cv::Mat &input_img, cv::Mat &output_img, std::vector< std::vector<std::vector<int> > > &barrier_tape_pts)
{
    cv::Mat preprocessed_img;
    cv::Mat edge_img;
    cv::RotatedRect box;
    std::vector<std::vector<cv::Point> > contours;
    int count = 0;
    bool has_detected_barrier_tape = false;

    preprocessImage(input_img, preprocessed_img);

    if (is_debug_mode_)
    {
        output_img = cv::Mat::zeros(preprocessed_img.size(), preprocessed_img.type());
    }

    cv::Canny(preprocessed_img, edge_img, 50, 100);
    cv::findContours(edge_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        std::vector<int> barrier_tape_box_center;
        std::vector<std::vector<int> > barrier_tape_box_pts;

        box = minAreaRect(cv::Mat(contours[i]));

        if ((box.size.height * box.size.width) < min_area_)
        {
            continue;
        }

        barrier_tape_box_center.push_back(box.center.x);
        barrier_tape_box_center.push_back(box.center.y);
        barrier_tape_box_pts.push_back(barrier_tape_box_center);
        //std::cout << "box center: " << box.center.x << ", " << box.center.y << std::endl;

        for (int j = 0; j < contours[i].size(); j++)
        {
            std::vector<int> contour_pt;
            contour_pt.push_back(contours[i][j].x);
            contour_pt.push_back(contours[i][j].y);
            barrier_tape_box_pts.push_back(contour_pt);
        }
        barrier_tape_pts.push_back(barrier_tape_box_pts);

        if (is_debug_mode_)
        {
            cv::drawContours(output_img, contours, i, cv::Scalar(255, 255, 255), 2);
        }

        count++;
    }
    if (count > 0)
    {
        has_detected_barrier_tape = true;
    }
    return has_detected_barrier_tape;
}

void BarrierTapeDetection::updateDynamicVariables(bool debug_mode, double min_area, int color_thresh_min_h, int color_thresh_min_s,
                                                  int color_thresh_min_v, int color_thresh_max_h, int color_thresh_max_s, int color_thresh_max_v)
{
    is_debug_mode_ = debug_mode;
    min_area_ = min_area;
    // convert the given HSV threshold from the standard (360, 100, 100) range to the (180, 255, 255) opencv range
    color_thresh_min_ = cv::Scalar(color_thresh_min_h*0.5, color_thresh_min_s*2.55, color_thresh_min_v*2.55);
    color_thresh_max_ = cv::Scalar(color_thresh_max_h*0.5, color_thresh_max_s*2.55, color_thresh_max_v*2.55);
}
