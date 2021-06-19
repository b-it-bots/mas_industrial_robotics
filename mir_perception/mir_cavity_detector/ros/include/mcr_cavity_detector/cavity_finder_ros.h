#ifndef CONTOUR_FINDER_ROS_H_
#define CONTOUR_FINDER_ROS_H_

#include <ros/ros.h>
#include <mcr_cavity_detector/cavity_finder.h>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <mcr_cavity_detector/CavityFinderConfig.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <mas_perception_msgs/ImageList.h>
#include <mas_perception_msgs/ObjectList.h>

/**
 * ROS interface for cavity finder
 * Subscribes to:
 *  -pointcloud: pointcloud in which to find cavities
 *
 * Publishes:
 *  -cavity pointclouds: array of cavities as pointclouds
 *  -debug rgb image: debug image showing detected edges in the 2D rgb image
 *  -debug depth image: debug image showing detected edges in the 2D depth image
 */
class CavityFinderROS
{
public:
    /**
     * Constructor
     */
    CavityFinderROS();
    /**
     * Destructor
     */
    virtual ~CavityFinderROS();
    /**
     * If pointcloud message has been received, the findCavities function is called.
     * This function can be called once or periodically.
     */
    void update();


private:
    /**
     * Copy constructor.
     */
    CavityFinderROS(const CavityFinderROS &other);

    /**
     * Copy assignment operator.
     */
    CavityFinderROS &operator=(CavityFinderROS other);

    /**
     * Callback for pointcloud. Saves the pointcloud message.
     *
     * @param msg
     *          sensor_msgs::PointCloud2 message
     */
    void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg);

    /**
     * Callback for rgb image
     *
     * @param msg
     *          sensor_msgs::Image message
     */
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void dummyImageCallback(const sensor_msgs::ImageConstPtr &msg);
    /**
     * Callback for cavities list
     *
     * @param msg
     *          sensor_msgs::ObjectList message
     */
    void cavitiesCallback(const mas_perception_msgs::ObjectList &msg);

    /**
     * Callback for event_in topic. Starts subscription to pointcloud if event_in is "e_trigger"
     */
    void eventInCallback(const std_msgs::String &msg);

    /**
     * Callback for dynamic reconfigure server to set canny threshold and multiplier
     */
    void dynamicReconfigCallback(mcr_cavity_detector::CavityFinderConfig &config, uint32_t level);

    /**
     * Finds 2D cavities and the corresponding 3D cavities and publishes the array of 3D cavities as pointclouds
     */
    void findCavities();



    /**
     * estimate pose of 3d cavities
     */
    geometry_msgs::PoseArray estimatePose(std::vector<pcl::PCLPointCloud2::Ptr> &pcl_cavities);
    /**
     *  Publish cavity message
     */
    void publish_cavity_msg(geometry_msgs::PoseArray& pose_array, std::vector<std::string>& cavities_name);
private:
    /**
     * Object of CavityFinder
     */
    CavityFinder cavity_finder_;

    ros::Publisher pub_cropped_cavities_;

    /**
     * Node handle
     */
    ros::NodeHandle nh_;

    /**
     * Image transport handle
     */
    image_transport::ImageTransport it_;
    /**
     * Subscriber for input pointcloud
     */
    ros::Subscriber sub_pointcloud_;

    /**
     * Subscriber for input image
     */
    image_transport::Subscriber sub_image_;
    image_transport::Subscriber tmp_sub_image_;
    /**
     * Subscriber for event_in topic
     */
    ros::Subscriber sub_event_in_;
    /**
    * Subscribe cavities name
    */
    ros::Subscriber sub_cavities_name;
    /**
     * Publisher for event_out topic
     */
    ros::Publisher pub_event_out_;

    /**
     * Publisher for 3D cavities list
     */
    ros::Publisher pub_cavity_pointclouds_;

    /**
     * Publisher for 3D cavities as a single pointcloud
     */
    ros::Publisher pub_cavity_pointclouds_combined_;

    /**
     * Publisher for pose of 3D cavities
     */
    ros::Publisher pub_pose_;
    /**
     * Publisher for cavity message
     */
    ros::Publisher pub_cavity_ ;
    /**
     * Publisher for debug image showing edges
     */
    image_transport::Publisher pub_rgb_debug_image_;
    /**
     * Publisher for debug image showing edges
     */
    image_transport::Publisher pub_depth_debug_image_;
    /**
     * Used to store pointcloud message received in callback
     */
    sensor_msgs::PointCloud2::Ptr pointcloud_msg_;

    /**
     * Used to store rgb image message received in callback
     */
    sensor_msgs::ImageConstPtr rgb_image_;
    /**
     * Dynamic reconfigure server
     */
    dynamic_reconfigure::Server<mcr_cavity_detector::CavityFinderConfig> dynamic_reconfigure_server_;

    /**
     * Flag indicating whether pointcloud has been received
     */
    bool pointcloud_msg_received_;
    /**
     * Flag indicating whether  has been received
     */
    bool img_msg_received_;

    /**
    * int indicating cavities received
    */
    int cavity_msg_received_count_;
    /**
     * Flag indicating whether  has been received
     */
    double offset_in_z_;

    /**
     *  Hack Bug in Realsnese camera
     *  https://mas.b-it-center.de/gitgate/mas-group/realsense_camera/issues/2
     *  Need to receive the cloud image twice
     */
    int pointcloud_msg_received_count_;
    int img_msg_received_count_ ;

    /**
     * Flag indicating whether debug image should be published
     */
    bool publish_debug_image_;
    /**
     * Stores the target frame to which transformation has to be made
     */
    std::string target_frame_;

    /**
     * Stores the source frame from which the transformation has to be made
     */
    std::string source_frame_;

    /**
    * vector to store cavities name
    */
   // std::vector<std::string> cavities_names_;
    mas_perception_msgs::ObjectList cavity_list_;
    /**
     * Object to TransformLister class handle frame transformations
     */
    tf::TransformListener listener_;

    /**
     * filter 2d cavities based on 3d cavities
     */
    void filterCavities(std::vector<std::vector<cv::Point> > &cavities, std::vector<pcl::PCLPointCloud2::Ptr> &pcl_cavities, std::vector<cv::Point2f> &centroids);
};

#endif
