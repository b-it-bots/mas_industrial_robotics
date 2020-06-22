#ifndef BARRIERTAPEDETECTION_H_
#define BARRIERTAPEDETECTION_H_

#include <opencv2/opencv.hpp>
#include <vector>

class BarrierTapeDetection {
 public:
  BarrierTapeDetection();
  virtual ~BarrierTapeDetection();
  /**
   * Convert to HSV, filter based on color_thresh_min and color_thresh_max, blur
   */
  void preprocessImage(const cv::Mat &input_img, cv::Mat &output_img);
  /**
   * Canny edge detection, contour detection, filter contours within certain
   * area range
   */
  bool detectBarrierTape(
      const cv::Mat &input_img, cv::Mat &output_img,
      std::vector<std::vector<std::vector<int>>> &barrier_tape_pts);
  /**
   * Set thresholds for min and max HSV values and min area of contours
   */
  void updateDynamicVariables(bool debug_mode, double min_area,
                              int color_thresh_min_h, int color_thresh_min_s,
                              int color_thresh_min_v, int color_thresh_max_h,
                              int color_thresh_max_s, int color_thresh_max_v);

 private:
  /**
   * Lower range of H, S and V values for barrier tape
   */
  cv::Scalar color_thresh_min_;
  /**
   * Upper range for H, S and V values for barrier tape
   */
  cv::Scalar color_thresh_max_;

  /**
   * If in debug mode, draw contours on output image
   */
  bool is_debug_mode_;
  /**
   * minimum area of detected contours which are considered
   * to be part of the barrier tape
   */
  double min_area_;
};

#endif /* BARRIERTAPEDETECTION_H_ */
