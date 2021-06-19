#ifndef CONTOUR_FINDER_H_
#define CONTOUR_FINDER_H_

#include <pcl/PCLPointCloud2.h>
#include <opencv2/core/core.hpp>
#include <mas_perception_msgs/ImageList.h>

/**
 * Finds 2D cavities after applying edge detection
 * Finds corresponding 3D cavity using the given pointcloud
 */
class CavityFinder
{
public:
    /**
     * Constructor
     */
    CavityFinder();
    /**
     * Destructor
     */
    virtual ~CavityFinder();
    /**
     * Finds 2D cavities in the image using edge detection. Displays the edges in the debug image
     *
     * @param image
     *          image in which to find the cavities
     * @param debug_image
     *          the found edges are displayed in this image
     * @return found cavities
     */


    /**
     * 
     * 
     * 
     * 
     * /
     * 
     * */


    std::vector<std::vector<cv::Point> > find2DCavities(const cv::Mat &image,
                                                        cv::Mat &debug_image,
                                                        std::vector<cv::Point2f> &centroids);


    /**
     * Recognize 2D cavities in the image. Displays the name in the debug image
     *
     * @param image
     *          image in which to find the cavities
     * @param debug_image
     *          the recognized cavity are displayed in this image
     * @param cavities
     *          the detected cavities for recognition
     * @return found cavitiesName
     */
    std::vector<std::string > recognize2DCavities(const cv::Mat &image,
                                                  cv::Mat &debug_image,
                                                  std::vector<cv::Mat> &cropped_cavities,
                                                  const std::vector<cv::Point2f> &centroids);
    /**
     * Finds 3D cavities corresponding to the given 2D cavities in the input cloud
     *
     * @param cavities
     *          input 2D cavities
     * @param input_cloud
     *          input pointcloud
     * @return found 3D cavities
     */
    std::vector<pcl::PCLPointCloud2::Ptr> get3DCavities(const std::vector<std::vector<cv::Point> > &cavities, pcl::PCLPointCloud2::Ptr input_cloud);

    /**
     * Given a rectangle with 2 points converts to set of points on the
     * boundary
     *
     * @param rect
     *          input Rectanle
     * @return vector points on the perimeter
     */
    std::vector<cv::Point> pointsOnRectangle(cv::Rect rect);

    /**
     * Set threshold1 for Canny edge detector
     * See http://docs.opencv.org/modules/imgproc/doc/feature_detection.html#canny
     *
     * @param canny_threshold
     * Threshold 1 for canny edge detection
     */
    void setCannyThreshold(double canny_threshold);

    /**
     * Set multiplier to calculate threshold2 for Canny edge detector
     * threshold2 = canny_multiplier * canny_threshold_
     * See http://docs.opencv.org/modules/imgproc/doc/feature_detection.html#canny
     *
     * @param canny_multiplier
     * Multiplier used to calculate threshold 2 for canny edge detection
     */
    void setCannyMultiplier(double canny_multiplier);

    /**
     * Set threshold1 for Binary threshold filter
     * See http://docs.opencv.org/2.4/doc/tutorials/imgproc/threshold/threshold.html
     * @param binary_threshold
     * Threshold for binary filter
     */
    void setBinaryThreshold(double binary_threshold);

    /**
     * Epsilon for approxpoly  and for finer poly
     * http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
     * @param epsilon
     * Epsilon for the approximation the polygon curve
     */
    void setEpsilonApproxPoly(double epsilon_approx_poly);
    void setEpsilonFinerPoly(double epsilon_finer_poly);


    /**
     * min and max area size filter for cavities in Depth_RGB image
     * @param min_area
     * min area to filter cavities in Depth_RGB image
     */
    void setMinArea(double min_area_depth_image);
    void setMaxArea(double max_area_depth_image);
private:
    /**
     * Copy constructor.
     */
    CavityFinder(const CavityFinder &other);

    /**
     * Copy assignment operator.
     */
    CavityFinder &operator=(CavityFinder other);


    cv::RNG rng;
private:
    /**
     * Threshold for canny edge detector
     */
    double canny_threshold_;
    /**
     * Multiplier for canny edge detector
     */
    double canny_multiplier_;
    /**
     * Threshold for binary thershold filter
     */
    double binary_threshold_;
    /**
     * Multiplier for approx polynomial fit
     */
    double approx_poly_epsilon_;
    /**
     * Multiplier for finer approx polynomial fit
     */
    double approx_poly_epsilon_finer_;
    /**
     * Min area for filtering cavities in DepthRGB image
     */
    double min_area_depth_image_;
    /**
     * Max area for filtering cavities in DepthRGB image
     */
    double max_area_depth_image_;
};
#endif
